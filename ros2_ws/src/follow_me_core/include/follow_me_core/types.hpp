// Basic types and small math helpers shared by the whole library.
// Conventions: world frame ENU (x East, y North, z Up), metres, radians.
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace follow_me {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat36 = Eigen::Matrix<double, 3, 6>;
using Mat63 = Eigen::Matrix<double, 6, 3>;

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

// Wrap an angle to [-pi, pi).
inline double wrapAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// Scale a vector down so that its norm does not exceed max_norm.
inline Vec3 saturate(const Vec3& v, double max_norm) {
  const double n = v.norm();
  return n <= max_norm ? v : Vec3(v * (max_norm / n));
}

}  // namespace follow_me
