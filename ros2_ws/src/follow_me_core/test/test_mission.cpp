#include <gtest/gtest.h>

#include "follow_me_core/controller.hpp"
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
  // ~480 m at 12 m/s (~35 s) + hand-over. The hand-over waits for the size to be known to 8 %,
  // learnt mostly from well-resolved views (size_res_px: far views weigh little), ~20 s.
  EXPECT_LT(m.t_follow, 70.0);
  EXPECT_GT(m.follow_pct, 40.0);
  EXPECT_LT(m.follow_nees, 12.0);      // consistent filter (ideal 6; 2x margin for 1 run)
  EXPECT_GT(m.min_distance, cfg.base.safety.d_min);
  EXPECT_LT(m.follow_dist_err_rms, 1.5);
  EXPECT_NEAR(m.size_est, cfg.target_size, 3.0 * m.size_sigma);
  EXPECT_GT(m.fusion.radar_own, 100);  // own echo recognised all along
  EXPECT_GT(m.fusion.replays, 100);    // late radar plots fused by replay
}

INSTANTIATE_TEST_SUITE_P(Seeds, Intercept, ::testing::Values(1, 4));

TEST(RandomScenario, DeterministicAndBounded) {
  const ManeuverParams mp = ManeuverParams::level(2);
  const auto a = randomScenario(42, 120.0, mp), b = randomScenario(42, 120.0, mp);
  ASSERT_EQ(a.size(), b.size());
  double z = mp.z0;
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].speed, b[i].speed);
    EXPECT_LE(a[i].speed, mp.v_max);
    EXPECT_LE(std::abs(a[i].turn_rate_deg), mp.turn_max_deg);
    const double len = (i + 1 < a.size() ? a[i + 1].t_start : 120.0) - a[i].t_start;
    z += a[i].climb_rate * len;
    EXPECT_GE(z, mp.z_min - 1e-9);
    EXPECT_LE(z, mp.z_max + 1e-9);
  }
  EXPECT_NE(randomScenario(43, 120.0, mp)[1].speed, a[1].speed);
}

// While the range is uncertain (radar only), the hand-over waits further away.
TEST(Mission, HandoverHoldGrowsWithRangeUncertainty) {
  const InterceptParams ip;
  GuidanceParams gp;
  auto closing = [&](double range_sigma) {
    MissionManager mm(ip, gp);
    MissionManager::Input in{0.0, Vec3(0, 0, 40), Vec3::Zero(), true, false,
                             Vec3(16, 0, 40), Vec3::Zero(), 0.4, range_sigma};
    MissionManager::Output out{};
    for (int k = 0; k < 300; ++k) out = mm.update(in, 0.01);  // 3 s: past the command low-pass
    EXPECT_EQ(out.phase, MissionPhase::Handover);
    return out.v_des.x();  // > 0: still closing in
  };
  // 0.4 m target: resolved (20 px) at 8 m -> hold_min 12 m.
  EXPECT_GT(closing(0.5), 0.1);          // max(12, 4 + 3 + 3 x 0.5) = 12 m < 16 m: closes in
  EXPECT_NEAR(closing(4.0), 0.0, 1e-9);  // max(12, 4 + 3 + 3 x 4) = 19 m > 16 m: holds
}

// Bumpless transfer (journal n°29): the follower arrives in front of the target, the hand-over law
// pulls it one way, the follow law the other. Entering FOLLOW must not make the acceleration
// command jump in one step (Gazebo: 4.5-9.8 m/s^2 before, 0.4-0.7 after).
TEST(Mission, EnteringFollowDoesNotStepTheCommand) {
  auto max_step = [](double bumpless_tau) {
    InterceptParams ip;
    ip.bumpless_tau = bumpless_tau;
    MissionManager mm(ip, GuidanceParams{});
    PositionController ctl(ControlParams{}, 6.0, 14.0);
    const double dt = 0.01;
    Vec3 p(0, 0, 40), v(0, -6, 0);  // 15 m in front of a target flying +x, moving sideways
    const Vec3 p_t(-15, 0, 42), v_t(8, 0, 0);
    Vec3 a_prev = Vec3::Zero();
    double step = 0.0;
    for (int k = 0; k < 200; ++k) {
      const bool locked = k >= 100;  // camera lock at 1 s: HANDOVER -> FOLLOW
      const auto out = mm.update({k * dt, p, v, true, locked, p_t + v_t * (k * dt), v_t, 0.6, 0.3}, dt);
      const Vec3 a = ctl.compute(p, v, out.p_des, out.v_des, dt, out.v_max, Vec3::Zero(), out.v_rel_max,
                                 out.v_cmd_tau);
      if (k >= 99 && k <= 101) step = std::max(step, (a - a_prev).norm());
      a_prev = a;  // kinematics frozen: only the command's own continuity is tested
    }
    return step;
  };
  EXPECT_GT(max_step(-1.0), 2.0);  // without: the command jumps at the switch (3 m/s^2 here)
  EXPECT_LT(max_step(0.8), 1.0);   // with: continuous
}
