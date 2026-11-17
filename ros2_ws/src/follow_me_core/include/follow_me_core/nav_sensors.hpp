// Simulated navigation sensors of the follower, generated from the true state.
//   IMU  : specific force + angular rate in body frame, white noise + slowly drifting biases
//   GNSS : position (ENU) + velocity, white noise
//   Baro : altitude, white noise
//   Mag  : heading (yaw) derived from the magnetic field, white noise
#pragma once

#include <random>

#include "follow_me_core/params.hpp"
#include "follow_me_core/quadrotor.hpp"

namespace follow_me {

class NavSensors {
 public:
  NavSensors(const NavSensorParams& p, std::uint64_t seed);

  struct Imu {
    Vec3 accel;  // specific force, body [m/s^2]
    Vec3 gyro;   // body rate [rad/s]
  };
  Imu imu(const Vec3& specific_force_body, const Vec3& rate_body, double dt);

  struct Gnss {
    Vec3 pos;
    Vec3 vel;
  };
  Gnss gnss(const Vec3& p, const Vec3& v);
  double baro(double z);
  double magYaw(const Quat& q);

  // True biases (for tests and for the monitor: the filter must find them).
  const Vec3& accelBias() const { return ba_; }
  const Vec3& gyroBias() const { return bg_; }
  const NavSensorParams& params() const { return p_; }

 private:
  Vec3 randn3() { return Vec3(n_(rng_), n_(rng_), n_(rng_)); }
  NavSensorParams p_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> n_{0.0, 1.0};
  Vec3 ba_, bg_;
};

double yawOf(const Quat& q);

}  // namespace follow_me
