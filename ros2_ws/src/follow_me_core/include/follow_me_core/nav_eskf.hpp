// Navigation of the follower: error-state EKF (ESKF), IMU-driven, 15 error states.
//
// Nominal state (propagated with the IMU, non-linear):
//   p, v (world ENU), q (body -> world), b_a (accelerometer bias), b_g (gyro bias)
// Error state (Kalman filter, linear):
//   dx = [dp(3) dv(3) dtheta(3) dba(3) dbg(3)],   q_true = q_hat * Exp(dtheta)  (body frame)
//
// Prediction at IMU rate:      a = R (f_m - b_a) + g,  w = w_m - b_g
// Error dynamics:              dp' = dv
//                              dv' = -R [f]x dtheta - R dba
//                              dtheta' = -[w]x dtheta - dbg
//                              dba' = noise, dbg' = noise (random walks)
// Corrections: GNSS position + velocity, baro altitude, magnetometer heading.
// After each correction the error is injected into the nominal state and reset to zero.
#pragma once

#include "follow_me_core/nav_sensors.hpp"

namespace follow_me {

using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat15 = Eigen::Matrix<double, 15, 15>;

class NavEskf {
 public:
  NavEskf(const NavSensorParams& sensors, const NavFilterParams& filter);

  // Start: position from GNSS, heading from the magnetometer, level attitude, at rest.
  void init(const Vec3& p, double yaw);
  bool initialised() const { return init_; }

  void predict(const NavSensors::Imu& imu, double dt);
  bool updateGnss(const NavSensors::Gnss& g);  // returns false if rejected by the chi2 gate
  bool updateBaro(double z);
  bool updateMagYaw(double yaw);

  const Vec3& position() const { return p_; }
  const Vec3& velocity() const { return v_; }
  const Quat& attitude() const { return q_; }
  const Vec3& accelBias() const { return ba_; }
  const Vec3& gyroBias() const { return bg_; }
  Vec3 rate() const { return w_; }  // gyro corrected for its bias [rad/s]
  const Mat15& P() const { return P_; }

 private:
  template <int M>
  bool correct(const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, 15>& H,
               const Eigen::Matrix<double, M, M>& R, double gate);
  void inject(const Vec15& dx);

  NavSensorParams s_;
  NavFilterParams f_;
  bool init_ = false;
  Vec3 p_ = Vec3::Zero(), v_ = Vec3::Zero(), ba_ = Vec3::Zero(), bg_ = Vec3::Zero();
  Vec3 w_ = Vec3::Zero();
  Quat q_ = Quat::Identity();
  Mat15 P_ = Mat15::Identity();
  double g_ = 9.81;
};

}  // namespace follow_me
