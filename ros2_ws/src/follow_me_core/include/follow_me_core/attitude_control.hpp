// Inner loops of the flight controller: acceleration -> thrust + attitude -> rates -> torques
// -> motor speeds.
//
//   a_des (world, from the position controller), yaw_des
//      |  F = m (a_des + g z)           thrust vector, tilt limited
//      |  b3 = F/|F|, heading -> R_des  desired attitude
//      |  T = F . (R b3)                collective thrust along the CURRENT body z
//   attitude P loop (quaternion error)  -> desired body rates (saturated)
//   rate PI loop + gyroscopic term      -> body torques
//   mixer (allocation inverse)          -> rotor thrusts -> motor speeds (with desaturation)
#pragma once

#include "follow_me_core/quadrotor.hpp"

namespace follow_me {

class MulticopterController {
 public:
  MulticopterController(const AttitudeControlParams& c, const QuadrotorParams& q);

  struct Output {
    Vec4 motor_cmd;      // [rad/s]
    double thrust;       // collective [N]
    Quat q_des;          // desired attitude
    Vec3 rate_des;       // desired body rates [rad/s]
    Vec3 torque;         // requested body torques [N m]
    bool saturated;      // mixer could not deliver the request exactly
  };
  Output update(const Vec3& a_des, double yaw_des, const Quat& q, const Vec3& w_body, double dt);
  void reset() { integral_.setZero(); }

  // Mixer alone: rotor thrusts that best realise (T, tau) within [0, f_max].
  Vec4 mix(double thrust, const Vec3& torque, bool* saturated) const;

 private:
  AttitudeControlParams c_;
  QuadrotorParams q_;
  Mat4 A_inv_;
  double f_max_;
  Vec3 integral_ = Vec3::Zero();
};

}  // namespace follow_me
