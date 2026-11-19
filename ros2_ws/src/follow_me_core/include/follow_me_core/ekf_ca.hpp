// Target EKF with ACCELERATION in the state, x = [p (3), v (3), a (3)], world frame, for the IMM of
// the FusionTracker. Two process models share this state so that the IMM can mix them:
//   - CV     : constant velocity, acceleration forced to 0 (same p, v as TargetEkf exactly);
//   - Singer : acceleration as a first-order Markov process (time constant tau), driven by white
//              jerk of density q. A braking or turning target is modelled instead of averaged.
// Why: with constant-velocity models only, the estimate lagged a target that stopped or started
// (the follower braked late, overshot and tilted to 30 deg; Gazebo NEES ~13 from the same lag).
// The acceleration also gives the guidance an anticipation term.
#pragma once

#include <cmath>
#include <optional>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/ekf.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat9 = Eigen::Matrix<double, 9, 9>;

class TargetEkfCA {
 public:
  enum class Model { CV, Singer };

  TargetEkfCA(Model model, double q, double tau = 3.0) : model_(model), q_(q), tau_(tau) {
    x_.setZero();
    P_.setIdentity();
  }

  void reset(const Vec9& x, const Mat9& P) {
    x_ = x;
    P_ = P;
  }

  void predict(double dt) {
    const Mat3 I = Mat3::Identity();
    Mat9 F = Mat9::Zero();
    Mat9 Q = Mat9::Zero();
    F.block<3, 3>(0, 0) = I;
    F.block<3, 3>(0, 3) = dt * I;
    F.block<3, 3>(3, 3) = I;
    if (model_ == Model::CV) {  // a -> 0, white acceleration noise on (p, v): TargetEkf exactly
      Q.block<3, 3>(0, 0) = q_ * dt * dt * dt / 3.0 * I;
      Q.block<3, 3>(0, 3) = Q.block<3, 3>(3, 0) = q_ * dt * dt / 2.0 * I;
      Q.block<3, 3>(3, 3) = q_ * dt * I;
    } else {  // Singer: a decays with tau, white jerk of density q (small-dt form)
      const double e = std::exp(-dt / tau_);
      F.block<3, 3>(0, 6) = 0.5 * dt * dt * I;
      F.block<3, 3>(3, 6) = dt * I;
      F.block<3, 3>(6, 6) = e * I;
      const double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt, d5 = d4 * dt;
      const double q[3][3] = {{d5 / 20, d4 / 8, d3 / 6}, {d4 / 8, d3 / 3, d2 / 2}, {d3 / 6, d2 / 2, dt}};
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Q.block<3, 3>(3 * i, 3 * j) = q_ * q[i][j] * I;
    }
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
  }

  struct Innovation {
    Vec3 y;
    Eigen::Matrix<double, 3, 9> H;
    Mat3 S;
  };
  std::optional<Innovation> innovation(const Vec3& z, const PinholeCamera& cam, const Vec3& cam_pos,
                                       const Mat3& R_cw, const Mat3& R_extra = Mat3::Zero()) const {
    const Vec3 rc = PinholeCamera::toCamera(position(), cam_pos, R_cw);
    if (rc.z() <= 0.1) return std::nullopt;
    Innovation in;
    in.y = z - cam.project(rc);
    in.H.setZero();
    in.H.leftCols<3>() = cam.jacobianPos(rc, R_cw);
    in.S = in.H * P_ * in.H.transpose() + cam.noiseCov() + R_extra;
    return in;
  }

  void update(const Innovation& in, const Mat3& R_meas) { updateGain<3>(in.y, in.H, R_meas, std::nullopt); }

  template <int M>
  double updateGeneric(const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, 9>& H,
                       const Eigen::Matrix<double, M, M>& R, double gate) {
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    const double nis = y.dot(S.ldlt().solve(y));
    if (nis <= gate) updateGain<M>(y, H, R, std::nullopt);
    return nis;
  }

  // Bearing-only update that may not change the estimate along the line of sight (see TargetEkf).
  void updateBearingKeepRange(const Eigen::Vector2d& y, const Eigen::Matrix<double, 2, 9>& H,
                              const Eigen::Matrix2d& R, const Vec3& los) {
    updateGain<2>(y, H, R, los);
  }

  const Vec9& x() const { return x_; }
  const Mat9& P() const { return P_; }
  Vec3 position() const { return x_.segment<3>(0); }
  Vec3 velocity() const { return x_.segment<3>(3); }
  Vec3 acceleration() const { return x_.segment<3>(6); }
  Mat3 positionCov() const { return P_.topLeftCorner<3, 3>(); }
  // The usual [p, v] view (what the rest of the code consumes).
  TargetEkf pv() const {
    TargetEkf e(q_);
    e.reset(x_.head<6>(), P_.topLeftCorner<6, 6>());
    return e;
  }

 private:
  template <int M>
  void updateGain(const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, 9>& H,
                  const Eigen::Matrix<double, M, M>& R, const std::optional<Vec3>& keep_range_los) {
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 9, M> K = P_ * H.transpose() * S.inverse();
    if (keep_range_los) {
      const Mat3 across = Mat3::Identity() - *keep_range_los * keep_range_los->transpose();
      for (int b = 0; b < 3; ++b) K.template block<3, M>(3 * b, 0) = across * K.template block<3, M>(3 * b, 0);
    }
    x_ += K * y;
    const Mat9 IKH = Mat9::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();  // Joseph: valid for any gain
  }

  Model model_;
  double q_, tau_;
  Vec9 x_;
  Mat9 P_;
};

}  // namespace follow_me
