// The follower as seen by the outer loops (shared by the offline simulators).
#pragma once

#include <algorithm>
#include <cmath>

#include "follow_me_core/attitude_control.hpp"
#include "follow_me_core/nav_eskf.hpp"
#include "follow_me_core/offline_sim.hpp"
#include "follow_me_core/sim_models.hpp"

namespace follow_me {

// The follower seen by the outer loops: either the phase-1 point mass, or the 6-DOF quadrotor
// with its flight controller (navigation ESKF + attitude/rate loops) running at 500 Hz inside
// each outer step. position()/velocity()/attitude() return what the GNC BELIEVES (navigation);
// truePosition() is for the simulator (detector, metrics) only.
class FollowerSim {
 public:
  explicit FollowerSim(const SimConfig& cfg, Vec3 start = Vec3::Constant(NAN), double yaw0 = 0.0)
      : quad_on_(cfg.quadrotor),
        nav_on_(cfg.quadrotor && cfg.use_nav),
        pm_(cfg.follower, cfg.seed),
        quad_(cfg.quad, std::isnan(start.x()) ? cfg.follower.start_pos : start, yaw0, cfg.seed),
        att_(cfg.attitude, cfg.quad),
        sensors_(cfg.nav_sensors, cfg.seed + 11),
        nav_(cfg.nav_sensors, cfg.nav_filter) {
    if (nav_on_)  // start: GNSS position, magnetometer heading, level, at rest
      nav_.init(sensors_.gnss(quad_.state().p, quad_.state().v).pos, sensors_.magYaw(quad_.state().q));
  }

  Vec3 truePosition() const { return quad_on_ ? quad_.state().p : pm_.position(); }
  Vec3 position() const { return nav_on_ ? nav_.position() : truePosition(); }
  Vec3 velocity() const {
    if (nav_on_) return nav_.velocity();
    return quad_on_ ? quad_.state().v : pm_.velocity();
  }
  Quat attitude() const { return nav_on_ ? nav_.attitude() : trueAttitude(); }
  Quat trueAttitude() const { return quad_on_ ? quad_.state().q : Quat::Identity(); }
  double yaw() const { return yawOf(attitude()); }
  // Navigation uncertainty seen by the tracker: position covariance + attitude std [rad].
  Mat3 navPosCov() const { return nav_on_ ? Mat3(nav_.P().topLeftCorner<3, 3>()) : Mat3::Zero(); }
  double navAttSigma() const {
    return nav_on_ ? std::sqrt(nav_.P().block<3, 3>(6, 6).diagonal().maxCoeff()) : 0.0;
  }

  // The tracker only knows the camera orientation through the navigation attitude:
  // R_believed = R_cw_true * R_body_true * R_body_est^T  (camera fixed w.r.t. the body here).
  Mat3 believedCameraRotation(const Mat3& R_cw_true) const {
    if (!nav_on_) return R_cw_true;
    return R_cw_true * trueAttitude().toRotationMatrix() * attitude().toRotationMatrix().transpose();
  }

  void step(const Vec3& a_cmd, double yaw_des, double dt) {
    if (!quad_on_) {
      pm_.step(a_cmd, dt);
      return;
    }
    const double h = 0.002;  // inner loop 500 Hz
    const int n = std::max(1, static_cast<int>(std::lround(dt / h)));
    for (int i = 0; i < n; ++i, ++k_) {
      // Navigation: IMU 250 Hz, mag 50 Hz, baro 25 Hz, GNSS 10 Hz.
      const auto& s = quad_.state();
      if (nav_on_ && k_ % 2 == 0) {
        nav_.predict(sensors_.imu(quad_.specificForce(), s.w, 2 * h), 2 * h);
        if (k_ % 10 == 0) nav_.updateMagYaw(sensors_.magYaw(s.q));
        if (k_ % 20 == 0) nav_.updateBaro(sensors_.baro(s.p.z()));
        if (k_ % 50 == 0) nav_.updateGnss(sensors_.gnss(s.p, s.v));
      }
      const Quat q = nav_on_ ? nav_.attitude() : s.q;
      const Vec3 w = nav_on_ ? nav_.rate() : s.w;
      const auto out = att_.update(a_cmd, yaw_des, q, w, h);
      quad_.step(out.motor_cmd, h);
      ++n_inner_;
      n_sat_ += out.saturated;
      const double tilt = std::acos(std::clamp(quad_.state().q.toRotationMatrix()(2, 2), -1.0, 1.0));
      max_tilt_ = std::max(max_tilt_, rad2deg(tilt));
      if (nav_on_) {
        s_np2_ += (nav_.position() - quad_.state().p).squaredNorm();
        Quat e = nav_.attitude().conjugate() * quad_.state().q;
        if (e.w() < 0) e.coeffs() *= -1;
        s_na2_ += (2.0 * e.vec()).squaredNorm();
      }
    }
  }

  double maxTiltDeg() const { return max_tilt_; }
  double saturationPct() const { return n_inner_ ? 100.0 * n_sat_ / n_inner_ : 0.0; }
  double navPosRmse() const { return n_inner_ ? std::sqrt(s_np2_ / n_inner_) : 0.0; }
  double navAttRmseDeg() const { return n_inner_ ? rad2deg(std::sqrt(s_na2_ / n_inner_)) : 0.0; }

 private:
  bool quad_on_, nav_on_;
  FollowerDynamics pm_;
  Quadrotor quad_;
  MulticopterController att_;
  NavSensors sensors_;
  NavEskf nav_;
  long k_ = 0, n_inner_ = 0, n_sat_ = 0;
  double max_tilt_ = 0.0, s_np2_ = 0.0, s_na2_ = 0.0;
};

}  // namespace follow_me
