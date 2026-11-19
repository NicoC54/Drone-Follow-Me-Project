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

Vec2 trackCoordinates(const Vec3& p_f, const Vec3& p_t, const Vec3& v_t, double v_heading_min) {
  const Vec2 vh = v_t.head<2>();
  if (vh.norm() <= v_heading_min) return Vec2(std::nan(""), std::nan(""));
  const Vec2 u = vh.normalized(), n(-u.y(), u.x());
  const Vec2 rel = (p_f - p_t).head<2>();
  return Vec2(rel.dot(u), rel.dot(n));
}

double MissionManager::followDistance() const {
  if (ip_.agile_gain <= 0.0) return gp_.follow_dist;
  return std::min(gp_.follow_dist + ip_.agile_gain * std::max(agility_ - ip_.agile_ref, 0.0),
                  std::max(ip_.agile_max_dist, gp_.follow_dist));
}

MissionManager::Output MissionManager::update(const Input& in, double dt) {
  const double range = (in.p_t - in.p_f).norm();
  // ---- observed agility of the target (decaying maximum of its horizontal acceleration) ----
  if (in.has_track && dt > 0.0) {
    const Vec3 v_prev = v_t_filt_ ? *v_t_filt_ : in.v_t;
    v_t_filt_ = v_prev + dt / (ip_.agile_tau + dt) * (in.v_t - v_prev);
    const double acc = ((*v_t_filt_ - v_prev) / dt).head<2>().norm();
    agility_ = std::max(acc, agility_ * std::exp(-dt / ip_.agile_memory));
  }
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
      case MissionPhase::Handover: {
        if (in.camera_locked && t_lock_first_ < 0.0) t_lock_first_ = in.t;
        // Rear rejoin: enter FOLLOW from behind the target (no turn-around), never later than
        // follow_entry_timeout after the lock; a slow target has no "behind".
        const Vec2 tc = trackCoordinates(in.p_f, in.p_t, in.v_t, ip_.rejoin_min_speed);
        const bool behind = !ip_.rejoin || !std::isfinite(tc.x()) || tc.x() < -0.5 * gp_.follow_dist;
        const bool waited = t_lock_first_ >= 0.0 && in.t - t_lock_first_ > ip_.follow_entry_timeout;
        if (in.camera_locked && (behind || waited)) {
          phase_ = MissionPhase::Follow;
          guidance_.reset();
        } else if (range > 1.5 * ip_.handover_range) {
          phase_ = MissionPhase::Midcourse;  // the target escaped
        }
        break;
      }
      case MissionPhase::Follow:
        if (in.t - t_lock_ok_ > ip_.lock_lost_time) phase_ = MissionPhase::Handover;
        break;
    }
  }

  if (phase_ != last_phase_) {
    last_phase_ = phase_;
    t_phase_change_ = in.t;
    // Bumpless transfer only for an entry FAR from the follow point (the turn-around case): an
    // entry already at the standoff distance has nothing to smooth, and smoothing there delayed the
    // reaction to a target turning towards the follower (offline scripted run 278: 2.64 m).
    bumpless_entry_ = phase_ == MissionPhase::Follow && range > followDistance() + ip_.bumpless_min_excess;
    if (phase_ != MissionPhase::Handover) t_lock_first_ = -1.0;
    if (phase_ == MissionPhase::Follow || phase_ == MissionPhase::Lost) rejoin_sign_ = 0.0;
  }
  Output out = setpoint(in, dt);
  if (ip_.transition_jerk > 0.0 && in.t - t_phase_change_ < ip_.transition_time)
    out.jerk_max = out.jerk_max > 0.0 ? std::min(out.jerk_max, ip_.transition_jerk) : ip_.transition_jerk;
  const double since = in.t - t_phase_change_;
  // Only on entering FOLLOW: applied at every phase change, it also altered the MIDCOURSE -> HANDOVER
  // approach (offline run 142: hand-over closing at 5.5 m/s at 13 m, minimum 3.96 m).
  if (ip_.bumpless_tau > 0.0 && since < ip_.bumpless_time && phase_ == MissionPhase::Follow && bumpless_entry_) {
    out.v_cmd_tau = ip_.bumpless_tau * (1.0 - since / ip_.bumpless_time);
    if (range > 1e-6) out.to_target = (in.p_t - in.p_f) / range;
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
      Vec3 off = standoffOffset(in.p_f, in.p_t, in.v_t, gp_);
      // Rejoin only behind a target that really flies: "behind" a hovering target is undefined, its
      // noisy heading flipped the rejoin point from side to side (offline L1 run 131: 1.6 m).
      const Vec2 tc = trackCoordinates(in.p_f, in.p_t, in.v_t, ip_.rejoin_min_speed);
      const bool rejoining = ip_.rejoin && std::isfinite(tc.x()) && tc.x() > -0.5 * gp_.follow_dist;
      if (rejoining) {
        // not behind yet: aim at the rejoin point, behind and on the side we are already on
        if (rejoin_sign_ == 0.0) rejoin_sign_ = tc.y() >= 0.0 ? 1.0 : -1.0;
        const Vec2 u = in.v_t.head<2>().normalized(), n(-u.y(), u.x());
        // The side offset keeps the follower CLEAR of the target while it gets past and drops behind:
        // full until 0.4 rejoin_back behind, then fading to 0 at the rejoin point. Measured: fading
        // it as soon as abeam left the follower alongside at 8-10 m while it dropped back, and a
        // target turning towards it closed the gap (offline scripted runs: 2.1 m, then 3.7 m).
        const double fade = std::clamp((tc.x() + ip_.rejoin_back) / (0.6 * ip_.rejoin_back), 0.0, 1.0);
        const double side = rejoin_sign_ * ip_.rejoin_side * fade;
        const Vec2 o = -ip_.rejoin_back * u + side * n;
        off = Vec3(o.x(), o.y(), gp_.height_offset);
      }
      const Rendezvous rv = predictedIntercept(in.p_f, in.p_t, in.v_t, off, V);
      // Velocity setpoint = target velocity + closing velocity limited to V (and 0 when blind
      // and already close: wait for the camera at a safe distance).
      Vec3 closing = rv.v_des;
      double hold = ip_.blind_hold_range;
      if (in.target_size > 0.0)
        hold = std::clamp(ip_.camera_fx * in.target_size / ip_.resolve_px, ip_.hold_min, ip_.blind_hold_range);
      hold = std::max(hold, ip_.d_min + ip_.hold_margin + ip_.k_range_sigma * in.range_sigma);
      if (handover && range < hold) {
        if (rejoining) {  // keep going around the target, never closer
          const Vec3 los = (in.p_t - in.p_f) / std::max(range, 1e-6);
          closing -= std::max(closing.dot(los), 0.0) * los;
        } else {
          closing.setZero();
        }
      }
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
      guidance_.setFollowDist(followDistance());
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
