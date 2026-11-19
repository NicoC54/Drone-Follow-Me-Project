#include "follow_me_core/mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace follow_me {

const char* toString(MissionPhase p) {
  switch (p) {
    case MissionPhase::Wait: return "WAIT";
    case MissionPhase::Midcourse: return "MIDCOURSE";
    case MissionPhase::Handover: return "HANDOVER";
    case MissionPhase::Follow: return "FOLLOW";
    case MissionPhase::Lost: return "LOST";
  }
  return "?";
}

Rendezvous predictedIntercept(const Vec3& p_f, const Vec3& p_t, const Vec3& v_t, const Vec3& offset,
                              double V) {
  const Vec3 r = p_t + offset - p_f;
  const double a = v_t.squaredNorm() - V * V, b = 2.0 * r.dot(v_t), c = r.squaredNorm();
  double tau = std::numeric_limits<double>::infinity();
  if (std::abs(a) < 1e-9) {
    if (b < 0) tau = -c / b;
  } else {
    const double disc = b * b - 4 * a * c;
    if (disc >= 0) {
      const double s = std::sqrt(disc);
      for (double root : {(-b - s) / (2 * a), (-b + s) / (2 * a)})
        if (root > 0 && root < tau) tau = root;
    }
  }
  Rendezvous rv;
  rv.feasible = std::isfinite(tau);
  rv.t_go = tau;
  rv.p_aim = rv.feasible ? Vec3(p_t + v_t * tau + offset) : Vec3(p_t + offset);  // else pure pursuit
  const Vec3 d = rv.p_aim - p_f;
  rv.v_des = d.norm() > 1e-6 ? Vec3(V * d.normalized()) : Vec3::Zero();
  return rv;
}

Vec3 standoffOffset(const Vec3& p_f, const Vec3& p_t, const Vec3& v_t, const GuidanceParams& g) {
  const Vec2 vh = v_t.head<2>();
  Vec2 dir;
  if (vh.norm() > g.v_heading_hi) {
    dir = -vh.normalized();  // behind the target
  } else {
    const Vec2 rel = (p_f - p_t).head<2>();
    dir = rel.norm() > 1e-6 ? Vec2(rel.normalized()) : Vec2(-1.0, 0.0);
  }
  return Vec3(g.follow_dist * dir.x(), g.follow_dist * dir.y(), g.height_offset);
}

MissionManager::Output MissionManager::update(const Input& in, double dt) {
  const double range = (in.p_t - in.p_f).norm();
  // ---- transitions ----
  if (!in.has_track) {
    if (phase_ != MissionPhase::Wait) phase_ = MissionPhase::Lost;
  } else {
    if (in.camera_locked) t_lock_ok_ = in.t;
    switch (phase_) {
      case MissionPhase::Wait:
      case MissionPhase::Lost:
        phase_ = range < ip_.handover_range ? MissionPhase::Handover : MissionPhase::Midcourse;
        break;
      case MissionPhase::Midcourse:
        if (range < ip_.handover_range) phase_ = MissionPhase::Handover;
        break;
      case MissionPhase::Handover:
        if (in.camera_locked) {
          phase_ = MissionPhase::Follow;
          guidance_.reset();
        } else if (range > 1.5 * ip_.handover_range) {
          phase_ = MissionPhase::Midcourse;  // the target escaped
        }
        break;
      case MissionPhase::Follow:
        if (in.t - t_lock_ok_ > ip_.lock_lost_time) phase_ = MissionPhase::Handover;
        break;
    }
  }

  if (phase_ != last_phase_) {
    last_phase_ = phase_;
    t_phase_change_ = in.t;
  }
  Output out = setpoint(in, dt);
  if (ip_.transition_jerk > 0.0 && in.t - t_phase_change_ < ip_.transition_time)
    out.jerk_max = out.jerk_max > 0.0 ? std::min(out.jerk_max, ip_.transition_jerk) : ip_.transition_jerk;
  const double since = in.t - t_phase_change_;
  // Only on entering FOLLOW: applied at every phase change, it also altered the MIDCOURSE -> HANDOVER
  // approach (offline run 142: hand-over closing at 5.5 m/s at 13 m, minimum 3.96 m).
  if (ip_.bumpless_tau > 0.0 && since < ip_.bumpless_time && phase_ == MissionPhase::Follow) {
    out.v_cmd_tau = ip_.bumpless_tau * (1.0 - since / ip_.bumpless_time);
    if (ip_.bumpless_close_speed > 0.0)
      out.v_rel_max = out.v_rel_max > 0.0 ? std::min(out.v_rel_max, ip_.bumpless_close_speed) : ip_.bumpless_close_speed;
  }
  return out;
}

MissionManager::Output MissionManager::setpoint(const Input& in, double dt) {
  const double range = (in.p_t - in.p_f).norm();
  // ---- setpoint of the phase ----
  Output out;
  out.phase = phase_;
  out.v_max = 14.0;
  out.hold = false;
  switch (phase_) {
    case MissionPhase::Wait:
    case MissionPhase::Lost:
      v_cmd_.reset();
      if (!hold_) hold_ = in.p_f;
      out.p_des = *hold_;
      out.v_des = Vec3::Zero();
      out.p_aim = *hold_;
      out.hold = true;
      return out;
    case MissionPhase::Midcourse:
    case MissionPhase::Handover: {
      hold_.reset();
      const bool handover = phase_ == MissionPhase::Handover;
      const double V = handover ? ip_.handover_speed : ip_.approach_speed;
      const Vec3 off = standoffOffset(in.p_f, in.p_t, in.v_t, gp_);
      const Rendezvous rv = predictedIntercept(in.p_f, in.p_t, in.v_t, off, V);
      // Velocity setpoint = target velocity + closing velocity limited to V (and 0 when blind
      // and already close: wait for the camera at a safe distance).
      Vec3 closing = rv.v_des;
      double hold = ip_.blind_hold_range;
      if (in.target_size > 0.0)
        hold = std::clamp(ip_.camera_fx * in.target_size / ip_.resolve_px, ip_.hold_min, ip_.blind_hold_range);
      hold = std::max(hold, ip_.d_min + ip_.hold_margin + ip_.k_range_sigma * in.range_sigma);
      if (handover && range < hold) closing.setZero();
      const Vec3 v_raw = handover ? Vec3(in.v_t + closing) : closing;
      if (!v_cmd_) v_cmd_ = in.v_f;  // start from the current velocity: no step
      const double tau = handover ? ip_.cmd_tau_handover : ip_.cmd_tau_midcourse;
      *v_cmd_ += dt / (tau + dt) * (v_raw - *v_cmd_);
      out.v_des = *v_cmd_;
      out.p_des = in.p_f + out.v_des * 0.5;  // velocity tracking (no position error to amplify)
      out.p_aim = rv.p_aim;
      out.t_go = std::isfinite(rv.t_go) ? rv.t_go : 0.0;
      out.v_max = handover ? in.v_t.norm() + ip_.handover_speed : ip_.approach_speed;
      if (handover) out.a_ff = ip_.a_ff_gain * in.a_t;
      return out;
    }
    case MissionPhase::Follow: {
      hold_.reset();
      v_cmd_.reset();
      const auto sp = guidance_.compute(in.p_f, in.p_t, in.v_t, dt);
      out.p_des = sp.p;
      out.v_des = sp.v;
      out.p_aim = sp.p;
      out.a_ff = ip_.a_ff_gain * in.a_t;
      out.a_max = ip_.follow_a_max;
      out.jerk_max = ip_.follow_jerk_max;
      out.v_rel_max = ip_.follow_close_speed;
      return out;
    }
  }
  return out;
}

}  // namespace follow_me
