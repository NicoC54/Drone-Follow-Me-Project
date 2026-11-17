// Pinhole camera model shared by the detector simulation and the EKF.
//
// Measurement z = [u, v, s]:  u, v = pixel centre of the target, s = apparent size [px].
// Camera frame: x right, y down, z along the optical axis.
#pragma once

#include <utility>

#include "follow_me_core/params.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

// R_cw such that r_cam = R_cw * r_world, for a gimbal pointing at (yaw, pitch) in world frame.
Mat3 rotWorldToCam(double yaw, double pitch);

// Yaw and pitch (world frame) that point the optical axis from `from` to `to`.
std::pair<double, double> lookAt(const Vec3& from, const Vec3& to);

class PinholeCamera {
 public:
  explicit PinholeCamera(const CameraParams& p);

  // World point -> camera frame.
  static Vec3 toCamera(const Vec3& p_target, const Vec3& cam_pos, const Mat3& R_cw) {
    return R_cw * (p_target - cam_pos);
  }
  // Camera frame -> measurement [u, v, s]  (this is h(x)).
  Vec3 project(const Vec3& rc) const;
  // d h / d p_target = d h/d rc * d rc/d p = J * R_cw.
  Mat3 jacobianPos(const Vec3& rc, const Mat3& R_cw) const;

  // Inverse of h: measurement -> world position and its first-order covariance.
  struct BackProjection {
    Vec3 p;
    Mat3 P;
  };
  BackProjection backproject(const Vec3& z, const Vec3& cam_pos, const Mat3& R_cw) const;

  // In front, in range, inside the image and big enough to be detected.
  bool visible(const Vec3& z, const Vec3& rc) const;

  const Mat3& noiseCov() const { return R_; }
  const CameraParams& params() const { return p_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }

 private:
  CameraParams p_;
  double cx_, cy_;
  Mat3 R_;  // measurement noise covariance diag(sigma_px^2, sigma_px^2, sigma_size^2)
};

}  // namespace follow_me
