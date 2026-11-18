#include "follow_me_core/mission.hpp"

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

  // ---- setpoint of the phase ----
  Output out;
  out.phase = phase_;
  out.v_max = 14.0;
  out.hold = false;
  switch (phase_) {
    case MissionPhase::Wait:
    case MissionPhase::Lost:
      if (!hold_) hold_ = in.p_f;
      out.p_des = *hold_;
      out.v_des = Vec3::Zero();
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
      if (handover && range < ip_.blind_hold_range) closing.setZero();
      out.v_des = handover ? Vec3(in.v_t + closing) : closing;
      out.p_des = in.p_f + out.v_des * 0.5;  // velocity tracking (no position error to amplify)
      out.v_max = handover ? in.v_t.norm() + ip_.handover_speed : ip_.approach_speed;
      return out;
    }
    case MissionPhase::Follow: {
      hold_.reset();
      const auto sp = guidance_.compute(in.p_f, in.p_t, in.v_t, dt);
      out.p_des = sp.p;
      out.v_des = sp.v;
      return out;
    }
  }
  return out;
}

}  // namespace follow_me
