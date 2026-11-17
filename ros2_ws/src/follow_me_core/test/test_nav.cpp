#include <gtest/gtest.h>

#include "follow_me_core/attitude_control.hpp"
#include "follow_me_core/nav_eskf.hpp"

using namespace follow_me;

namespace {
// Flies the quadrotor (controller on the TRUE state) along an exciting trajectory while the
// ESKF runs on simulated sensors. Returns statistics of the navigation error.
struct NavRun {
  double pos_rmse = 0, vel_rmse = 0, att_rmse_deg = 0, yaw_rmse_deg = 0, mean_nees = 0;
  Vec3 gyro_bias_err, accel_bias_err;
  int gnss_rejected = 0;
};

Vec3 attitudeError(const Quat& q_true, const Quat& q_est) {
  Quat e = q_est.conjugate() * q_true;
  if (e.w() < 0) e.coeffs() *= -1;
  return 2.0 * e.vec();
}

NavRun fly(double duration, std::uint64_t seed) {
  QuadrotorParams qp;
  Quadrotor quad(qp, Vec3(0, 0, 20), 0.3, seed);
  MulticopterController ctrl(AttitudeControlParams{}, qp);
  NavSensorParams sp;
  NavSensors sensors(sp, seed + 7);
  NavEskf nav(sp, NavFilterParams{});
  nav.init(sensors.gnss(quad.state().p, quad.state().v).pos, sensors.magYaw(quad.state().q));

  const double dt = 0.004;  // IMU 250 Hz; controller 250 Hz too here
  const int n = static_cast<int>(duration / dt);
  NavRun r;
  int n_stat = 0;
  for (int k = 0; k < n; ++k) {
    const double t = k * dt;
    const Vec3 a_des(2.5 * std::sin(0.6 * t), 2.0 * std::cos(0.4 * t), 0.8 * std::sin(0.9 * t));
    const double yaw_des = 0.3 + 1.2 * std::sin(0.15 * t);
    const auto& s = quad.state();
    const auto out = ctrl.update(a_des, yaw_des, s.q, s.w, dt);
    quad.step(out.motor_cmd, dt);

    nav.predict(sensors.imu(quad.specificForce(), quad.state().w, dt), dt);
    if (k % 25 == 0 && !nav.updateGnss(sensors.gnss(quad.state().p, quad.state().v))) ++r.gnss_rejected;
    if (k % 10 == 0) nav.updateBaro(sensors.baro(quad.state().p.z()));
    if (k % 5 == 0) nav.updateMagYaw(sensors.magYaw(quad.state().q));

    if (t > 10.0) {  // after convergence
      Vec15 e;
      e << nav.position() - quad.state().p, nav.velocity() - quad.state().v,
          -attitudeError(quad.state().q, nav.attitude()), nav.accelBias() - sensors.accelBias(),
          nav.gyroBias() - sensors.gyroBias();
      r.pos_rmse += e.segment<3>(0).squaredNorm();
      r.vel_rmse += e.segment<3>(3).squaredNorm();
      r.att_rmse_deg += e.segment<2>(6).squaredNorm();
      r.yaw_rmse_deg += std::pow(wrapAngle(yawOf(nav.attitude()) - yawOf(quad.state().q)), 2);
      r.mean_nees += e.dot(nav.P().ldlt().solve(e));
      ++n_stat;
    }
  }
  r.pos_rmse = std::sqrt(r.pos_rmse / n_stat);
  r.vel_rmse = std::sqrt(r.vel_rmse / n_stat);
  r.att_rmse_deg = rad2deg(std::sqrt(r.att_rmse_deg / n_stat));
  r.yaw_rmse_deg = rad2deg(std::sqrt(r.yaw_rmse_deg / n_stat));
  r.mean_nees /= n_stat;
  r.gyro_bias_err = nav.gyroBias() - sensors.gyroBias();
  r.accel_bias_err = nav.accelBias() - sensors.accelBias();
  return r;
}
}  // namespace

TEST(NavEskf, TracksPositionVelocityAttitudeAndBiases) {
  const NavRun r = fly(60.0, 3);
  std::printf("pos %.2f m, vel %.3f m/s, tilt %.2f deg, yaw %.2f deg, NEES %.1f (ideal 15)\n",
              r.pos_rmse, r.vel_rmse, r.att_rmse_deg, r.yaw_rmse_deg, r.mean_nees);
  std::printf("gyro bias err %.4f rad/s, accel bias err %.3f m/s^2\n", r.gyro_bias_err.norm(),
              r.accel_bias_err.norm());
  EXPECT_LT(r.pos_rmse, 0.6);            // better than raw GNSS (0.5 m h, 0.8 m v)
  EXPECT_LT(r.vel_rmse, 0.15);
  EXPECT_LT(r.att_rmse_deg, 0.5);        // tilt observable through the accelerometer
  EXPECT_LT(r.yaw_rmse_deg, 2.0);
  EXPECT_LT(r.gyro_bias_err.norm(), 0.003);  // initial bias spread was 0.01 rad/s per axis
  EXPECT_GT(r.mean_nees, 5.0);           // consistent filter: NEES around 15
  EXPECT_LT(r.mean_nees, 40.0);
  // A 99.9 % gate rejects ~0.1 % of GOOD measurements by chance: 2400 fixes -> ~2.4 expected.
  EXPECT_LE(r.gnss_rejected, 10);
}

TEST(NavEskf, RejectsAGnssGlitch) {
  NavSensorParams sp;
  NavEskf nav(sp, NavFilterParams{});
  nav.init(Vec3::Zero(), 0.0);
  NavSensors::Imu at_rest{Vec3(0, 0, 9.81), Vec3::Zero()};
  for (int k = 0; k < 250; ++k) nav.predict(at_rest, 0.004);
  EXPECT_TRUE(nav.updateGnss({Vec3(0.3, -0.2, 0.1), Vec3::Zero()}));
  EXPECT_FALSE(nav.updateGnss({Vec3(30.0, 0.0, 0.0), Vec3::Zero()}));  // 30 m jump
  EXPECT_LT(nav.position().norm(), 1.0);
}
