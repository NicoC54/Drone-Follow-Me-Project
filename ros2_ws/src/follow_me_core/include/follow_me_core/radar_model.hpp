// Ground radar: polar measurement model (for the EKF) and plot generator (for the simulation).
//
// Measurement of an object at p with velocity v, seen from the radar site c (ENU):
//   d = p - c,  r = |d|,  rho = |d_xy|
//   z = [ r, az = atan2(d_y, d_x), el = atan2(d_z, rho), rdot = d.v / r ]
// The radar measures range and radial velocity well, angles coarsely: its uncertainty in
// Cartesian space is a flat "pancake" orthogonal to the line of sight, whose lateral size grows
// with range (r * sigma_angle): 2.6 m at 300 m for 0.5 deg. The camera is the opposite.
#pragma once

#include <random>
#include <vector>

#include "follow_me_core/params.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

using Vec4 = Eigen::Vector4d;
using Mat4 = Eigen::Matrix4d;
using Mat46 = Eigen::Matrix<double, 4, 6>;

struct RadarPlot {
  double range, az, el, rdot;  // [m, rad, rad, m/s]
  bool is_target = false;      // simulation ground truth, for metrics only
  bool is_own = false;         // echo of the follower itself (ground truth, metrics only)
  Vec4 z() const { return Vec4(range, az, el, rdot); }
};

struct RadarScan {
  double t;                    // time the scan was taken (stamp) [s]
  std::vector<RadarPlot> plots;
};

// z = h(p, v) seen from the site, and its Jacobian with respect to [p, v].
Vec4 radarMeasure(const Vec3& p, const Vec3& v, const Vec3& site);
Mat46 radarJacobian(const Vec3& p, const Vec3& v, const Vec3& site);
Mat4 radarNoiseCov(const GroundRadarParams& prm);

// Plot -> Cartesian position and its (first-order) covariance; used to start a track.
struct PolarToCartesian {
  Vec3 p;
  Mat3 P;
};
PolarToCartesian radarToCartesian(const RadarPlot& plot, const GroundRadarParams& prm);

// Simulated radar: one scan of all real objects (detection probability, field of view) plus
// Poisson clutter uniformly spread in the coverage volume.
class GroundRadarSim {
 public:
  GroundRadarSim(const GroundRadarParams& p, std::uint64_t seed) : p_(p), rng_(seed) {}

  struct Object {
    Vec3 p, v;
    bool is_target, is_own;
  };
  RadarScan scan(double t, const std::vector<Object>& objects);

 private:
  GroundRadarParams p_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> n_{0.0, 1.0};
  std::uniform_real_distribution<double> u_{0.0, 1.0};
};

}  // namespace follow_me
