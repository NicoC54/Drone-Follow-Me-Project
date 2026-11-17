#include "follow_me_core/ekf.hpp"

namespace follow_me {

void TargetEkf::predict(double dt) {
  const Mat3 I = Mat3::Identity();
  Mat6 F = Mat6::Identity();
  F.topRightCorner<3, 3>() = dt * I;
  Mat6 Q;
  Q << dt * dt * dt / 3.0 * I, dt * dt / 2.0 * I,
       dt * dt / 2.0 * I,      dt * I;
  Q *= q_;
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

std::optional<TargetEkf::Innovation> TargetEkf::innovation(const Vec3& z, const PinholeCamera& cam,
                                                           const Vec3& cam_pos,
                                                           const Mat3& R_cw,
                                                           const Mat3& R_extra) const {
  const Vec3 rc = PinholeCamera::toCamera(position(), cam_pos, R_cw);
  if (rc.z() <= 0.1) return std::nullopt;
  Innovation in;
  in.y = z - cam.project(rc);
  in.H.setZero();
  in.H.leftCols<3>() = cam.jacobianPos(rc, R_cw);  // pixels do not depend on velocity
  in.S = in.H * P_ * in.H.transpose() + cam.noiseCov() + R_extra;
  return in;
}

void TargetEkf::update(const Innovation& in, const Mat3& R_meas) {
  const Mat63 K = P_ * in.H.transpose() * in.S.inverse();
  x_ += K * in.y;
  const Mat6 IKH = Mat6::Identity() - K * in.H;
  P_ = IKH * P_ * IKH.transpose() + K * R_meas * K.transpose();
}

}  // namespace follow_me
