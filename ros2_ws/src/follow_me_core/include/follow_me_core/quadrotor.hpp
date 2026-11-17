// Quadrotor: rotor model, allocation (mixer) and 6-DOF rigid-body dynamics.
//
// The rotor model (MotorModel + rotorWrench) is shared by the offline simulator and the Gazebo
// flight-controller plugin, which applies the same forces to the Gazebo rigid body.
#pragma once

#include <Eigen/Geometry>
#include <random>

#include "follow_me_core/params.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

using Vec4 = Eigen::Vector4d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;

// Geometry of the X frame: [T, tau_x, tau_y, tau_z]^T = A * [f0, f1, f2, f3]^T (body FLU).
Mat4 allocationMatrix(const QuadrotorParams& p);

// First-order motor speed dynamics (faster up than down).
class MotorModel {
 public:
  explicit MotorModel(const QuadrotorParams& p) : p_(p) {}
  void reset(const Vec4& w) { w_ = w; }
  void step(const Vec4& w_cmd, double dt);
  const Vec4& speeds() const { return w_; }

 private:
  QuadrotorParams p_;
  Vec4 w_ = Vec4::Zero();
};

// Collective thrust [N] (along body z) and body torques [N m] produced by the rotors.
struct RotorWrench {
  double thrust;
  Vec3 torque;
};
RotorWrench rotorWrench(const QuadrotorParams& p, const Vec4& motor_speeds);

struct QuadState {
  Vec3 p = Vec3::Zero();               // world ENU [m]
  Vec3 v = Vec3::Zero();               // world [m/s]
  Quat q = Quat::Identity();           // body -> world
  Vec3 w = Vec3::Zero();               // body angular rate [rad/s]
};

// Full 6-DOF simulation (offline sim only; in Gazebo the physics engine does this part).
class Quadrotor {
 public:
  Quadrotor(const QuadrotorParams& p, const Vec3& p0, double yaw0, std::uint64_t seed);

  // Integrates with internal sub-steps of at most 1 ms.
  void step(const Vec4& motor_cmd, double dt);

  const QuadState& state() const { return s_; }
  const Vec4& motorSpeeds() const { return motors_.speeds(); }
  // Specific force in body frame (what an ideal accelerometer measures): R^T (a - g).
  const Vec3& specificForce() const { return f_body_; }
  double hoverMotorSpeed() const;

 private:
  void subStep(const Vec4& motor_cmd, double dt);

  QuadrotorParams p_;
  MotorModel motors_;
  QuadState s_;
  Vec3 f_body_ = Vec3::Zero();
  Vec3 gust_ = Vec3::Zero();
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

}  // namespace follow_me
