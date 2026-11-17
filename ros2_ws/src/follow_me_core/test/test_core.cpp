#include <gtest/gtest.h>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/controller.hpp"
#include "follow_me_core/ekf.hpp"
#include "follow_me_core/offline_sim.hpp"
#include "follow_me_core/time_series.hpp"
#include "follow_me_core/timed_tracker.hpp"
#include "follow_me_core/track_manager.hpp"

using namespace follow_me;

TEST(Camera, RotationIsOrthonormalAndPointsForward) {
  const Mat3 R = rotWorldToCam(0.7, -0.3);
  EXPECT_TRUE((R * R.transpose()).isApprox(Mat3::Identity(), 1e-12));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
  const Vec3 axis(std::cos(-0.3) * std::cos(0.7), std::cos(-0.3) * std::sin(0.7), std::sin(-0.3));
  EXPECT_TRUE((R * axis).isApprox(Vec3(0, 0, 1), 1e-12));
}

TEST(Camera, JacobianMatchesFiniteDifferences) {
  const PinholeCamera cam{CameraParams{}};
  const Mat3 R = rotWorldToCam(0.3, 0.1);
  const Vec3 cam_pos = Vec3::Zero(), p(8.0, 3.0, 1.5);
  const Mat3 H = cam.jacobianPos(PinholeCamera::toCamera(p, cam_pos, R), R);
  const double eps = 1e-6;
  for (int j = 0; j < 3; ++j) {
    const Vec3 e = Vec3::Unit(j) * eps;
    const Vec3 col = (cam.project(PinholeCamera::toCamera(p + e, cam_pos, R)) -
                      cam.project(PinholeCamera::toCamera(p - e, cam_pos, R))) / (2 * eps);
    EXPECT_TRUE(H.col(j).isApprox(col, 1e-5)) << "column " << j;
  }
}

TEST(Camera, BackprojectionInvertsProjection) {
  const PinholeCamera cam{CameraParams{}};
  const Mat3 R = rotWorldToCam(-1.0, 0.2);
  const Vec3 cam_pos(1, 2, 3), p(-5, -12, 5);
  const Vec3 z = cam.project(PinholeCamera::toCamera(p, cam_pos, R));
  EXPECT_TRUE(cam.backproject(z, cam_pos, R).p.isApprox(p, 1e-12));
}

// Same numbers as the Python reference (follow_me/estimation.py), to 1e-9.
TEST(Ekf, MatchesPythonReference) {
  const PinholeCamera cam{CameraParams{}};
  const Mat3 R = rotWorldToCam(0.3, -0.1);
  const Vec3 cam_pos(1, 2, 20);
  const auto bp = cam.backproject(Vec3(700, 380, 48), cam_pos, R);
  TargetEkf ekf(1.5);
  Vec6 x0;
  x0 << bp.p, 1.0, 0.5, 0.0;
  Mat6 P0 = Mat6::Zero();
  P0.topLeftCorner<3, 3>() = bp.P;
  P0.bottomRightCorner<3, 3>() = 25.0 * Mat3::Identity();
  ekf.reset(x0, P0);
  ekf.predict(0.1);
  const auto in = ekf.innovation(Vec3(705, 377, 47.5), cam, cam_pos, R);
  ASSERT_TRUE(in.has_value());
  ekf.update(*in, cam.noiseCov());

  Vec6 x_ref;
  x_ref << 9.191455036363749, 3.817522687420208, 18.981060895953917, 1.073106144570107,
      -0.2891690084905677, 0.20067880723640516;
  Vec6 p_ref;
  p_ref << 0.02684529065697882, 0.001847447529587944, 0.000872010524948325, 4.6438718012568625,
      0.4701002829286678, 0.19981111716050676;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(ekf.x()(i), x_ref(i), 1e-9) << "x[" << i << "]";
    EXPECT_NEAR(ekf.P()(i, i), p_ref(i), 1e-9) << "P[" << i << "," << i << "]";
  }
  EXPECT_NEAR(ekf.P()(0, 3), 0.24046044711423425, 1e-9);
  EXPECT_NEAR(ekf.P()(1, 2), -0.0007757037396016559, 1e-9);
}

class ClosedLoop : public ::testing::TestWithParam<int> {};

TEST_P(ClosedLoop, KeepsSafeDistanceAndTracks) {
  SimConfig cfg;
  cfg.seed = GetParam();
  const auto m = runOfflineSim(cfg);
  EXPECT_GT(m.min_distance, cfg.safety.d_min);
  EXPECT_LT(m.est_pos_rmse, 0.5);
  EXPECT_LT(m.dist_err_rms, 2.0);
  EXPECT_GT(m.valid_track_pct, 95.0);
  EXPECT_EQ(m.outliers_rejected, m.outliers_total);
}

INSTANTIATE_TEST_SUITE_P(Seeds, ClosedLoop, ::testing::Values(0, 1, 2));

TEST(ClosedLoop, RecoversAfterOcclusion) {
  SimConfig cfg;
  cfg.occlusions = {{40.0, 46.0}};
  const auto m = runOfflineSim(cfg);
  EXPECT_GT(m.min_distance, cfg.safety.d_min);
  EXPECT_GT(m.valid_track_pct, 85.0);  // lost for a while, then reacquired
}

// ---------------- Delayed measurements ----------------

TEST(TimeSeries, InterpolatesAndRejectsUncoveredTimes) {
  TimeSeries<Vec3> h(1.0, 0.05);
  const auto lerp = [](const Vec3& a, const Vec3& b, double w) -> Vec3 { return (1 - w) * a + w * b; };
  h.add(1.0, Vec3(0, 0, 0));
  h.add(1.1, Vec3(1, 0, 0));
  h.add(1.05, Vec3(9, 9, 9));  // out of order: ignored
  const auto p = h.at(1.025, lerp);
  ASSERT_TRUE(p.has_value());
  EXPECT_TRUE(p->isApprox(Vec3(0.25, 0, 0), 1e-12));
  EXPECT_TRUE(h.at(1.12, lerp).has_value());   // slightly after the last sample: last value
  EXPECT_FALSE(h.at(0.9, lerp).has_value());   // before the history
  EXPECT_FALSE(h.at(1.3, lerp).has_value());   // too far after the last sample
}

// The bug seen in Gazebo with the GUI: 0.3 s of detection latency.
TEST(Latency, FusingWithCapturePoseKeepsTheTrack) {
  SimConfig cfg;
  cfg.detection_latency = 0.3;
  const auto m = runOfflineSim(cfg);
  EXPECT_GT(m.valid_track_pct, 95.0);
  EXPECT_LT(m.est_pos_rmse, 0.5);
  EXPECT_GT(m.min_distance, cfg.safety.d_min);
}

TEST(Latency, FusingWithArrivalPoseLosesTheTrack) {
  SimConfig cfg;
  cfg.detection_latency = 0.3;
  cfg.naive_latency = true;
  const auto m = runOfflineSim(cfg);
  EXPECT_LT(m.valid_track_pct, 80.0);  // documents the failure mode
  EXPECT_GT(m.est_pos_rmse, 2.0);
}

// ---------------- Quadrotor + navigation in the loop ----------------
// The tracker only knows its camera pose through the follower's navigation. Ignoring that
// uncertainty makes it over-confident; propagating it restores consistency.
TEST(NavInTheLoop, CameraPoseUncertaintyKeepsTheTrackerConsistent) {
  SimConfig cfg;  // quadrotor + ESKF navigation (defaults)
  ASSERT_TRUE(cfg.quadrotor && cfg.use_nav);
  cfg.nav_noise_in_tracker = false;
  const auto naive = runOfflineSim(cfg);
  cfg.nav_noise_in_tracker = true;
  const auto m = runOfflineSim(cfg);
  EXPECT_GT(naive.mean_nees, 25.0);
  EXPECT_LT(m.mean_nees, 15.0);
  EXPECT_LT(m.nav_pos_rmse, 0.3);
  EXPECT_LT(m.nav_att_rmse_deg, 1.5);
  EXPECT_GT(m.min_distance, cfg.safety.d_min);
  EXPECT_LT(m.max_tilt_deg, cfg.attitude.max_tilt_deg + 2.0);
}

// ---------------- Safety: stopping-distance speed limit ----------------

TEST(SafetyFilter, BrakesWhenClosingFasterThanTheStoppingDistanceAllows) {
  SafetyParams sp;  // d_min 4, a_brake 3
  SafetyFilter f(sp, 6.0);
  const Vec3 p_t = Vec3::Zero(), v_t = Vec3::Zero();
  const Mat3 P = 0.01 * Mat3::Identity();
  const Vec3 p_f(10.0, 0, 0);  // 10 m away: margin ~6 m -> v_safe = sqrt(2*3*6) = 6 m/s
  // Closing at 3 m/s while commanding 6 m/s^2 towards the leader: allowed (far enough).
  auto r = f.filter(Vec3(-6, 0, 0), p_f, Vec3(-3, 0, 0), p_t, v_t, P);
  EXPECT_FALSE(r.active);
  // Closing at 9 m/s: cannot stop in time -> the command must point away from the leader.
  r = f.filter(Vec3(-6, 0, 0), p_f, Vec3(-9, 0, 0), p_t, v_t, P);
  EXPECT_TRUE(r.active);
  EXPECT_GT(r.a.x(), 0.0);
  // Moving away fast: nothing to do.
  r = f.filter(Vec3(1, 0, 0), p_f, Vec3(9, 0, 0), p_t, v_t, P);
  EXPECT_FALSE(r.active);
}

// ---------------- Truncated beacon: bearing-only update ----------------

TEST(TargetTracker, TruncatedBeaconUpdatesBearingButNotRange) {
  const PinholeCamera cam{CameraParams{}};
  TargetTracker tr(EstimatorParams{}, cam);
  const Mat3 R = rotWorldToCam(0.0, 0.0);
  const Vec3 c = Vec3::Zero(), p_true(10.0, 0.0, 0.0);
  const Vec3 z = cam.project(PinholeCamera::toCamera(p_true, c, R));
  ASSERT_FALSE(tr.onMeasurement(z, 0.00, c, R));  // 1st detection: candidate only
  ASSERT_TRUE(tr.onMeasurement(z, 0.03, c, R));   // confirmed: track started
  const double range0 = tr.ekf().position().norm();
  // The beacon is half hidden: size 30 % too small (as if 14 m away) and shifted by 10 px.
  const Vec3 z_bad(z.x() + 10.0, z.y(), 0.7 * z.z());
  EXPECT_TRUE(tr.onMeasurement(z_bad, 0.06, c, R, Mat3::Zero(), /*size_valid=*/false));
  EXPECT_NEAR(tr.ekf().position().norm(), range0, 0.05);  // range untouched
  EXPECT_GT(tr.ekf().position().y(), -1.0);
  EXPECT_LT(tr.ekf().position().y(), 0.0);               // bearing moved to the right (u+)
  // Trusted fully, the same measurement fails the chi2 gate: the bearing information is lost too.
  // (In Gazebo the occlusion grew frame after frame, each small step passed the gate and the
  // range drifted out by metres: hence the truncation check in the detector.)
  TargetTracker tr2(EstimatorParams{}, cam);
  tr2.onMeasurement(z, 0.00, c, R);
  tr2.onMeasurement(z, 0.03, c, R);
  EXPECT_FALSE(tr2.onMeasurement(z_bad, 0.06, c, R));
  EXPECT_NEAR(tr2.ekf().position().y(), 0.0, 1e-9);
  // A slowly growing truncation (3 % smaller per frame) is accepted step by step: range drifts.
  TargetTracker tr3(EstimatorParams{}, cam);
  tr3.onMeasurement(z, 0.00, c, R);
  tr3.onMeasurement(z, 0.03, c, R);
  for (int k = 1; k <= 10; ++k)
    tr3.onMeasurement(Vec3(z.x(), z.y(), std::pow(0.97, k) * z.z()), 0.03 * (k + 1), c, R);
  EXPECT_GT(tr3.ekf().position().norm(), range0 + 0.5);  // ~0.9 m after 10 frames
}
