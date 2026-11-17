// Control: cascaded position (P) / velocity (PI) controller + separation safety filter.
#pragma once

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
  Vec3 compute(const Vec3& p, const Vec3& v, const Vec3& p_des, const Vec3& v_des, double dt);

 private:
  ControlParams p_;
  double a_max_, v_max_;
  Vec3 integral_ = Vec3::Zero();
};

// Last layer before the motors. Works on the ESTIMATED leader position, made conservative by
// k_sigma standard deviations along the line of sight.
//  1. Everywhere: the closing speed must stay below sqrt(2 a_brake (d - d_min)), the speed from
//     which braking still stops before d_min. Any excess is braked (stopping-distance limit).
//  2. Inside the buffer zone: remove the command component towards the leader, add repulsion +
//     closing-speed damping.
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
