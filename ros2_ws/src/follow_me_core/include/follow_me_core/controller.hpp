// Control: cascaded position (P) / velocity (PI) controller + separation safety filter.
#pragma once

#include <optional>

#include "follow_me_core/params.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

// v_cmd = v_des + kp (p_des - p)          (saturated to v_max)
// a_cmd = kv (v_cmd - v) + integral        (integral cancels drag and wind)
class PositionController {
 public:
  PositionController(const ControlParams& p, double a_max, double v_max)
      : p_(p), a_max_(a_max), v_max_(v_max) {}

  void reset() { integral_.setZero(); }
  // v_max_override > 0: speed limit for this call (mission phases), otherwise v_max.
  // a_ff: acceleration feed-forward (e.g. the target's estimated acceleration while following it).
  // v_rel_max > 0: the position correction kp (p_des - p) is limited to this speed, i.e. the
  // follower closes on a moving setpoint at most v_rel_max faster than the setpoint moves.
  // v_cmd_tau > 0: bumpless transfer. The velocity command starts from its previous value and
  // reaches the new law's with this time constant (the mission lets it decay to 0 after a phase
  // change): the acceleration command is continuous although the guidance law switched.
  Vec3 compute(const Vec3& p, const Vec3& v, const Vec3& p_des, const Vec3& v_des, double dt,
               double v_max_override = -1.0, const Vec3& a_ff = Vec3::Zero(), double v_rel_max = -1.0,
               double v_cmd_tau = -1.0, const Vec3& to_target = Vec3::Zero());
  // to_target (unit vector towards the target, or zero): the smoothing never delays a command
  // that closes in more slowly; along that axis the new law applies at once (safety first).

 private:
  ControlParams p_;
  double a_max_, v_max_;
  Vec3 integral_ = Vec3::Zero();
  std::optional<Vec3> v_cmd_prev_;  // last velocity command (bumpless transfer)
};

// Last layer before the motors. Works on the ESTIMATED leader position, made conservative by
// k_sigma standard deviations along the line of sight.
//  1. Everywhere: the closing speed must stay below sqrt(2 a_brake (d - d_min)), the speed from
//     which braking still stops before d_min. Any excess is braked (stopping-distance limit).
//  2. Inside the buffer zone: remove the command component towards the leader, add repulsion +
//     closing-speed damping.
// Limits how fast the acceleration command may change (jerk): a smooth ride. Applied to the
// NOMINAL command only; the safety filter acts after it, with full authority.
class JerkLimiter {
 public:
  explicit JerkLimiter(double jerk_max) : jerk_max_(jerk_max) {}
  Vec3 apply(const Vec3& a, double dt) {
    if (!prev_ || jerk_max_ <= 0.0) return *(prev_ = a);
    const Vec3 d = a - *prev_;
    const double step = jerk_max_ * dt;
    *prev_ += d.norm() > step ? Vec3(d * (step / d.norm())) : d;
    return *prev_;
  }
  void reset(const std::optional<Vec3>& a = std::nullopt) { prev_ = a; }
  void setMax(double jerk_max) { jerk_max_ = jerk_max; }

 private:
  double jerk_max_;
  std::optional<Vec3> prev_;
};

class SafetyFilter {
 public:
  SafetyFilter(const SafetyParams& p, double a_max) : p_(p), a_max_(a_max) {}

  struct Result {
    Vec3 a;
    bool active;
  };
  Result filter(const Vec3& a_cmd, const Vec3& p_f, const Vec3& v_f, const Vec3& p_t,
                const Vec3& v_t, const Mat3& P_pos) const;

 private:
  SafetyParams p_;
  double a_max_;
};

}  // namespace follow_me
