#include "follow_me_core/nav_eskf.hpp"

namespace follow_me {

namespace {
Mat3 skew(const Vec3& v) {
  Mat3 S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}
Quat expQ(const Vec3& dtheta) {
  const double a = dtheta.norm();
  return a < 1e-12 ? Quat::Identity() : Quat(Eigen::AngleAxisd(a, dtheta / a));
}
// Error-state block indices
constexpr int kP = 0, kV = 3, kTH = 6, kBA = 9, kBG = 12;
}  // namespace

NavEskf::NavEskf(const NavSensorParams& sensors, const NavFilterParams& filter)
    : s_(sensors), f_(filter) {}

void NavEskf::init(const Vec3& p, double yaw) {
  p_ = p;
  v_.setZero();
  q_ = Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
  ba_.setZero();
  bg_.setZero();
  P_.setZero();
  P_.block<3, 3>(kP, kP) = std::pow(f_.init_pos_sigma, 2) * Mat3::Identity();
  P_.block<3, 3>(kV, kV) = std::pow(f_.init_vel_sigma, 2) * Mat3::Identity();
  const double tilt = deg2rad(f_.init_tilt_sigma_deg), yaw_s = deg2rad(f_.init_yaw_sigma_deg);
  P_.block<3, 3>(kTH, kTH) = Vec3(tilt * tilt, tilt * tilt, yaw_s * yaw_s).asDiagonal();
  P_.block<3, 3>(kBA, kBA) = std::pow(s_.accel_bias_sigma0, 2) * Mat3::Identity();
  P_.block<3, 3>(kBG, kBG) = std::pow(s_.gyro_bias_sigma0, 2) * Mat3::Identity();
  init_ = true;
}

void NavEskf::predict(const NavSensors::Imu& imu, double dt) {
  if (!init_) return;
  const Mat3 R = q_.toRotationMatrix();
  const Vec3 f = imu.accel - ba_;
  w_ = imu.gyro - bg_;

  // --- nominal state (non-linear) ---
  const Vec3 a = R * f + Vec3(0, 0, -g_);
  p_ += v_ * dt + 0.5 * a * dt * dt;
  v_ += a * dt;
  q_ = (q_ * expQ(w_ * dt)).normalized();

  // --- error-state covariance (first-order discretisation) ---
  Mat15 F = Mat15::Identity();
  F.block<3, 3>(kP, kV) = Mat3::Identity() * dt;
  F.block<3, 3>(kV, kTH) = -R * skew(f) * dt;
  F.block<3, 3>(kV, kBA) = -R * dt;
  F.block<3, 3>(kTH, kTH) = expQ(w_ * dt).toRotationMatrix().transpose();
  F.block<3, 3>(kTH, kBG) = -Mat3::Identity() * dt;

  // Process noise: per-sample sensor noise enters as an increment; biases random-walk.
  Mat15 Q = Mat15::Zero();
  Q.block<3, 3>(kV, kV) = std::pow(s_.accel_sigma * dt, 2) * Mat3::Identity();
  Q.block<3, 3>(kTH, kTH) = std::pow(s_.gyro_sigma * dt, 2) * Mat3::Identity();
  Q.block<3, 3>(kBA, kBA) = s_.accel_bias_rw * s_.accel_bias_rw * dt * Mat3::Identity();
  Q.block<3, 3>(kBG, kBG) = s_.gyro_bias_rw * s_.gyro_bias_rw * dt * Mat3::Identity();
  P_ = F * P_ * F.transpose() + Q;
}

template <int M>
bool NavEskf::correct(const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, 15>& H,
                      const Eigen::Matrix<double, M, M>& R, double gate) {
  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  if (y.dot(S.ldlt().solve(y)) > gate) return false;  // chi2 gate: reject glitches
  const Eigen::Matrix<double, 15, M> K = P_ * H.transpose() * S.inverse();
  inject(K * y);
  const Mat15 IKH = Mat15::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();  // Joseph form
  return true;
}

void NavEskf::inject(const Vec15& dx) {
  p_ += dx.segment<3>(kP);
  v_ += dx.segment<3>(kV);
  q_ = (q_ * expQ(dx.segment<3>(kTH))).normalized();
  ba_ += dx.segment<3>(kBA);
  bg_ += dx.segment<3>(kBG);
  // The reset Jacobian G = I - [dtheta/2]x is ~ I for small corrections; neglected here.
}

bool NavEskf::updateGnss(const NavSensors::Gnss& g) {
  if (!init_) return false;
  Eigen::Matrix<double, 6, 1> y;
  y << g.pos - p_, g.vel - v_;
  Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
  H.block<3, 3>(0, kP) = Mat3::Identity();
  H.block<3, 3>(3, kV) = Mat3::Identity();
  Eigen::Matrix<double, 6, 1> r;
  r << s_.gnss_pos_sigma_h, s_.gnss_pos_sigma_h, s_.gnss_pos_sigma_v, Vec3::Constant(s_.gnss_vel_sigma);
  const Eigen::Matrix<double, 6, 6> R = r.cwiseProduct(r).asDiagonal();
  return correct<6>(y, H, R, f_.gate_chi2_gnss);
}

bool NavEskf::updateBaro(double z) {
  if (!init_) return false;
  Eigen::Matrix<double, 1, 1> y(z - p_.z());
  Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
  H(0, kP + 2) = 1.0;
  Eigen::Matrix<double, 1, 1> R(s_.baro_sigma * s_.baro_sigma);
  return correct<1>(y, H, R, f_.gate_chi2_1d);
}

bool NavEskf::updateMagYaw(double yaw) {
  if (!init_) return false;
  Eigen::Matrix<double, 1, 1> y(wrapAngle(yaw - yawOf(q_)));
  // A body-frame error dtheta changes the world heading by (R dtheta)_z.
  Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
  H.block<1, 3>(0, kTH) = q_.toRotationMatrix().row(2);
  const double s = deg2rad(s_.mag_yaw_sigma_deg);
  Eigen::Matrix<double, 1, 1> R(s * s);
  return correct<1>(y, H, R, f_.gate_chi2_1d);
}

}  // namespace follow_me
