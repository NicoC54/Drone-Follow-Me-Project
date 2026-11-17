#include "follow_me_core/camera_model.hpp"

namespace follow_me {

Mat3 rotWorldToCam(double yaw, double pitch) {
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const Vec3 forward(cp * cy, cp * sy, sp);
  const Vec3 right(sy, -cy, 0.0);
  const Vec3 down = forward.cross(right);
  Mat3 R;
  R.row(0) = right.transpose();
  R.row(1) = down.transpose();
  R.row(2) = forward.transpose();
  return R;
}

std::pair<double, double> lookAt(const Vec3& from, const Vec3& to) {
  const Vec3 r = to - from;
  return {std::atan2(r.y(), r.x()), std::atan2(r.z(), std::hypot(r.x(), r.y()))};
}

PinholeCamera::PinholeCamera(const CameraParams& p)
    : p_(p), cx_(p.width / 2.0), cy_(p.height / 2.0) {
  R_ = Vec3(p.sigma_px * p.sigma_px, p.sigma_px * p.sigma_px, p.sigma_size_px * p.sigma_size_px)
           .asDiagonal();
}

Vec3 PinholeCamera::project(const Vec3& rc) const {
  const double z = rc.z();
  return Vec3(p_.fx * rc.x() / z + cx_, p_.fy * rc.y() / z + cy_, p_.fx * p_.target_size / z);
}

Mat3 PinholeCamera::jacobianPos(const Vec3& rc, const Mat3& R_cw) const {
  const double x = rc.x(), y = rc.y(), z = rc.z();
  Mat3 J;
  J << p_.fx / z, 0.0, -p_.fx * x / (z * z),
       0.0, p_.fy / z, -p_.fy * y / (z * z),
       0.0, 0.0, -p_.fx * p_.target_size / (z * z);
  return J * R_cw;
}

PinholeCamera::BackProjection PinholeCamera::backproject(const Vec3& z, const Vec3& cam_pos,
                                                         const Mat3& R_cw) const {
  const double depth = p_.fx * p_.target_size / z(2);
  const Vec3 rc((z(0) - cx_) * depth / p_.fx, (z(1) - cy_) * depth / p_.fy, depth);
  const double sig_lat = depth * p_.sigma_px / p_.fx;
  const double sig_depth = depth * p_.sigma_size_px / z(2);  // grows with depth^2
  const Mat3 P_cam = Vec3(sig_lat * sig_lat, sig_lat * sig_lat, sig_depth * sig_depth).asDiagonal();
  return {cam_pos + R_cw.transpose() * rc, R_cw.transpose() * P_cam * R_cw};
}

bool PinholeCamera::visible(const Vec3& z, const Vec3& rc) const {
  return rc.z() > 0.1 && rc.z() <= p_.max_range && z(0) >= 0 && z(0) < p_.width && z(1) >= 0 &&
         z(1) < p_.height && z(2) >= p_.min_size_px;
}

}  // namespace follow_me
