#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include "follow_me_core/radar_model.hpp"

using namespace follow_me;

TEST(Radar, JacobianMatchesFiniteDifferences) {
  const Vec3 site(-60, -40, 3), p(150, 90, 35), v(-6, 3, 1);
  const Mat46 H = radarJacobian(p, v, site);
  const double eps = 1e-6;
  Eigen::Matrix<double, 6, 1> x;
  x << p, v;
  for (int j = 0; j < 6; ++j) {
    Eigen::Matrix<double, 6, 1> dx = Eigen::Matrix<double, 6, 1>::Zero();
    dx(j) = eps;
    const Eigen::Matrix<double, 6, 1> xp = x + dx, xm = x - dx;
    const Vec4 col = (radarMeasure(xp.head<3>(), xp.tail<3>(), site) -
                      radarMeasure(xm.head<3>(), xm.tail<3>(), site)) / (2 * eps);
    EXPECT_TRUE(H.col(j).isApprox(col, 1e-6)) << "column " << j;
  }
}

TEST(Radar, PolarToCartesianInvertsTheMeasurement) {
  GroundRadarParams prm;
  const Vec3 p(150, 90, 35);
  const Vec4 z = radarMeasure(p, Vec3::Zero(), prm.position);
  RadarPlot pl{z(0), z(1), z(2), z(3)};
  EXPECT_TRUE(radarToCartesian(pl, prm).p.isApprox(p, 1e-9));
}

// Radar uncertainty: thin along the line of sight (2 m), wide across it (r * 0.5 deg).
TEST(Radar, UncertaintyIsAPancakeOrthogonalToTheLineOfSight) {
  GroundRadarParams prm;
  const Vec3 p = prm.position + Vec3(300, 0, 0);  // 300 m due East, at the radar height
  const Vec4 z = radarMeasure(p, Vec3::Zero(), prm.position);
  const auto c = radarToCartesian(RadarPlot{z(0), z(1), z(2), z(3)}, prm);
  EXPECT_NEAR(std::sqrt(c.P(0, 0)), prm.sigma_range, 1e-6);                       // along LOS
  EXPECT_NEAR(std::sqrt(c.P(1, 1)), 300 * deg2rad(prm.sigma_az_deg), 1e-6);       // ~2.6 m
  EXPECT_NEAR(std::sqrt(c.P(2, 2)), 300 * deg2rad(prm.sigma_el_deg), 1e-6);       // ~4.2 m
}

TEST(Radar, SimulatorDetectsObjectsAndAddsClutter) {
  GroundRadarParams prm;
  prm.p_detect = 1.0;
  prm.clutter_per_scan = 3.0;
  GroundRadarSim sim(prm, 1);
  int n_target = 0, n_own = 0, n_clutter = 0;
  for (int k = 0; k < 200; ++k) {
    const auto s = sim.scan(0.5 * k, {{Vec3(200, 100, 40), Vec3(5, 0, 0), true, false},
                                      {Vec3(-40, -30, 20), Vec3::Zero(), false, true}});
    for (const auto& pl : s.plots) {
      n_target += pl.is_target;
      n_own += pl.is_own;
      n_clutter += !pl.is_target && !pl.is_own;
    }
  }
  EXPECT_EQ(n_target, 200);
  EXPECT_EQ(n_own, 200);
  EXPECT_NEAR(n_clutter / 200.0, 3.0, 0.4);  // Poisson mean
}
