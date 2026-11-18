#include <gtest/gtest.h>

#include "follow_me_core/intercept_sim.hpp"
#include "follow_me_core/mission.hpp"

using namespace follow_me;

// The rendezvous point must be reachable in a straight line at speed V exactly when the target's
// standoff point gets there.
TEST(Pip, SatisfiesTheMeetingCondition) {
  const Vec3 p_f(0, 0, 20), p_t(300, 200, 40), v_t(-6, 2, 0), off(8, -3, -2);
  const double V = 12.0;
  const Rendezvous rv = predictedIntercept(p_f, p_t, v_t, off, V);
  ASSERT_TRUE(rv.feasible);
  EXPECT_TRUE(rv.p_aim.isApprox(p_t + v_t * rv.t_go + off, 1e-9));
  EXPECT_NEAR((rv.p_aim - p_f).norm(), V * rv.t_go, 1e-6);
  EXPECT_NEAR(rv.v_des.norm(), V, 1e-9);
}

TEST(Pip, FallsBackToPursuitWhenTheTargetIsFasterAndFleeing) {
  const Vec3 p_f(0, 0, 20), p_t(100, 0, 20), v_t(15, 0, 0);  // 15 m/s away, we fly at 12
  const Rendezvous rv = predictedIntercept(p_f, p_t, v_t, Vec3::Zero(), 12.0);
  EXPECT_FALSE(rv.feasible);
  EXPECT_TRUE(rv.v_des.normalized().isApprox(Vec3::UnitX(), 1e-9));  // head for it anyway
}

TEST(Pip, StandoffIsBehindAMovingTargetAndBelowIt) {
  GuidanceParams g;
  g.height_offset = -2.0;
  const Vec3 off = standoffOffset(Vec3(-50, 0, 0), Vec3::Zero(), Vec3(0, 8, 0), g);
  EXPECT_TRUE(off.isApprox(Vec3(0, -8, -2), 1e-9));  // behind (-y) and 2 m below
}

// Whole phase-4 chain: ground radar -> fusion (OOSM) -> PIP midcourse -> camera hand-over ->
// follow, with the quadrotor flying on its own navigation.
class Intercept : public ::testing::TestWithParam<int> {};

TEST_P(Intercept, AcquiresInterceptsHandsOverAndFollowsSafely) {
  InterceptConfig cfg;
  cfg.base.seed = GetParam();
  const auto m = runInterceptSim(cfg);
  EXPECT_GT(m.initial_range, 400.0);
  EXPECT_GT(m.t_track, 0.0);
  EXPECT_LT(m.t_track, 3.0);           // radar track within a few scans
  EXPECT_GT(m.t_follow, 0.0);
  EXPECT_LT(m.t_follow, 40.0);         // ~480 m covered at 12 m/s + hand-over
  EXPECT_GT(m.follow_pct, 60.0);
  EXPECT_GT(m.min_distance, cfg.base.safety.d_min);
  EXPECT_LT(m.follow_dist_err_rms, 1.5);
  EXPECT_NEAR(m.size_est, cfg.target_size, 3.0 * m.size_sigma);
  EXPECT_GT(m.fusion.radar_own, 100);  // own echo recognised all along
  EXPECT_GT(m.fusion.replays, 100);    // late radar plots fused by replay
}

INSTANTIATE_TEST_SUITE_P(Seeds, Intercept, ::testing::Values(1, 4));
