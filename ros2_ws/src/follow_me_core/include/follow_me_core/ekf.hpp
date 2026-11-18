// Extended Kalman filter on the LEADER state x = [p (3), v (3)], world frame.
// Constant-velocity process model driven by white acceleration noise of density q.
#pragma once

#include <optional>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

class TargetEkf {
 public:
  explicit TargetEkf(double q_accel) : q_(q_accel) {
    x_.setZero();
    P_.setIdentity();
  }

  void reset(const Vec6& x, const Mat6& P) {
    x_ = x;
    P_ = P;
  }

  // x <- F x,  P <- F P F^T + Q
  void predict(double dt);

  struct Innovation {
    Vec3 y;    // z - h(x)
    Mat36 H;   // d h / d x
    Mat3 S;    // H P H^T + R
  };
  // Empty if the predicted target is behind the camera (linearisation meaningless).
  // R_extra: additional measurement covariance, e.g. the camera pose uncertainty coming from
  // the follower's navigation (see TargetTracker::cameraPoseNoise).
  std::optional<Innovation> innovation(const Vec3& z, const PinholeCamera& cam,
                                       const Vec3& cam_pos, const Mat3& R_cw,
                                       const Mat3& R_extra = Mat3::Zero()) const;

  // Kalman update, Joseph form (keeps P symmetric positive).
  void update(const Innovation& in, const Mat3& R_meas);

  // Generic update for any measurement size M (radar: M = 4). Returns the NIS; the state is
  // updated only if NIS <= gate (chi2 test).
  template <int M>
  double updateGeneric(const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, 6>& H,
                       const Eigen::Matrix<double, M, M>& R, double gate) {
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    const double nis = y.dot(S.ldlt().solve(y));
    if (nis > gate) return nis;
    const Eigen::Matrix<double, 6, M> K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    const Mat6 IKH = Mat6::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    return nis;
  }

  const Vec6& x() const { return x_; }
  const Mat6& P() const { return P_; }
  Vec3 position() const { return x_.head<3>(); }
  Vec3 velocity() const { return x_.tail<3>(); }
  Mat3 positionCov() const { return P_.topLeftCorner<3, 3>(); }

 private:
  double q_;
  Vec6 x_;
  Mat6 P_;
};

}  // namespace follow_me
