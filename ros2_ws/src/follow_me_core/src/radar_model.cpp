#include "follow_me_core/radar_model.hpp"

#include <cmath>

namespace follow_me {

Vec4 radarMeasure(const Vec3& p, const Vec3& v, const Vec3& site) {
  const Vec3 d = p - site;
  const double r = d.norm(), rho = d.head<2>().norm();
  return Vec4(r, std::atan2(d.y(), d.x()), std::atan2(d.z(), rho), d.dot(v) / r);
}

Mat46 radarJacobian(const Vec3& p, const Vec3& v, const Vec3& site) {
  const Vec3 d = p - site;
  const double r = d.norm(), r2 = r * r;
  const double rho2 = d.x() * d.x() + d.y() * d.y(), rho = std::sqrt(rho2);
  const double rdot = d.dot(v) / r;
  Mat46 H = Mat46::Zero();
  H.block<1, 3>(0, 0) = d.transpose() / r;                              // d r / d p
  H.block<1, 3>(1, 0) << -d.y() / rho2, d.x() / rho2, 0.0;              // d az / d p
  H.block<1, 3>(2, 0) << -d.x() * d.z() / (rho * r2), -d.y() * d.z() / (rho * r2), rho / r2;  // d el / d p
  H.block<1, 3>(3, 0) = (v - rdot * d / r).transpose() / r;             // d rdot / d p
  H.block<1, 3>(3, 3) = d.transpose() / r;                              // d rdot / d v
  return H;
}

Mat4 radarNoiseCov(const GroundRadarParams& prm) {
  const double a = deg2rad(prm.sigma_az_deg), e = deg2rad(prm.sigma_el_deg);
  return Vec4(prm.sigma_range * prm.sigma_range, a * a, e * e, prm.sigma_rdot * prm.sigma_rdot)
      .asDiagonal();
}

PolarToCartesian radarToCartesian(const RadarPlot& z, const GroundRadarParams& prm) {
  const double ca = std::cos(z.az), sa = std::sin(z.az), ce = std::cos(z.el), se = std::sin(z.el);
  const Vec3 u(ce * ca, ce * sa, se);
  // d p / d (r, az, el)
  Mat3 J;
  J.col(0) = u;
  J.col(1) = z.range * Vec3(-ce * sa, ce * ca, 0.0);
  J.col(2) = z.range * Vec3(-se * ca, -se * sa, ce);
  const Mat4 R = radarNoiseCov(prm);
  return {prm.position + z.range * u, J * R.topLeftCorner<3, 3>() * J.transpose()};
}

RadarScan GroundRadarSim::scan(double t, const std::vector<Object>& objects) {
  RadarScan s;
  s.t = t;
  const double sa = deg2rad(p_.sigma_az_deg), se = deg2rad(p_.sigma_el_deg);
  for (const auto& o : objects) {
    const Vec4 z = radarMeasure(o.p, o.v, p_.position);
    if (z(0) < p_.min_range || z(0) > p_.max_range || z(2) < deg2rad(p_.min_elevation_deg)) continue;
    if (u_(rng_) > p_.p_detect) continue;
    RadarPlot pl;
    pl.range = z(0) + p_.sigma_range * n_(rng_);
    pl.az = wrapAngle(z(1) + sa * n_(rng_));
    pl.el = z(2) + se * n_(rng_);
    pl.rdot = z(3) + p_.sigma_rdot * n_(rng_);
    pl.is_target = o.is_target;
    pl.is_own = o.is_own;
    s.plots.push_back(pl);
  }
  // Clutter: Poisson count, uniform in range / azimuth / low elevations (birds, ground returns).
  std::poisson_distribution<int> nclut(p_.clutter_per_scan);
  const int n = nclut(rng_);
  for (int i = 0; i < n; ++i) {
    RadarPlot pl;
    pl.range = p_.min_range + u_(rng_) * (std::min(p_.max_range, 1500.0) - p_.min_range);
    pl.az = wrapAngle(2.0 * M_PI * u_(rng_));
    pl.el = deg2rad(p_.min_elevation_deg + u_(rng_) * 20.0);
    pl.rdot = 10.0 * (2.0 * u_(rng_) - 1.0);
    s.plots.push_back(pl);
  }
  return s;
}

}  // namespace follow_me
