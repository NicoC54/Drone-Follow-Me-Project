#include "follow_me_core/nav_sensors.hpp"

namespace follow_me {

double yawOf(const Quat& q) {
  const Vec3 x = q.toRotationMatrix().col(0);
  return std::atan2(x.y(), x.x());
}

NavSensors::NavSensors(const NavSensorParams& p, std::uint64_t seed) : p_(p), rng_(seed) {
  ba_ = p.accel_bias_sigma0 * randn3();
  bg_ = p.gyro_bias_sigma0 * randn3();
}

NavSensors::Imu NavSensors::imu(const Vec3& f_body, const Vec3& w_body, double dt) {
  ba_ += p_.accel_bias_rw * std::sqrt(dt) * randn3();
  bg_ += p_.gyro_bias_rw * std::sqrt(dt) * randn3();
  return {f_body + ba_ + p_.accel_sigma * randn3(), w_body + bg_ + p_.gyro_sigma * randn3()};
}

NavSensors::Gnss NavSensors::gnss(const Vec3& p, const Vec3& v) {
  const Vec3 np(p_.gnss_pos_sigma_h * n_(rng_), p_.gnss_pos_sigma_h * n_(rng_),
                p_.gnss_pos_sigma_v * n_(rng_));
  return {p + np, v + p_.gnss_vel_sigma * randn3()};
}

double NavSensors::baro(double z) { return z + p_.baro_sigma * n_(rng_); }

double NavSensors::magYaw(const Quat& q) {
  return wrapAngle(yawOf(q) + deg2rad(p_.mag_yaw_sigma_deg) * n_(rng_));
}

}  // namespace follow_me
