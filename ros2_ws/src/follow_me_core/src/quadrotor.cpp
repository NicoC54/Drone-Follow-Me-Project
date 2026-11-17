#include "follow_me_core/quadrotor.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace follow_me {

namespace {
// Motor positions (body FLU) and spin direction (+1 = CCW seen from above).
struct MotorGeom {
  double x, y, spin;
};
std::array<MotorGeom, 4> motorGeometry(double arm) {
  const double a = arm / std::sqrt(2.0);
  return {{{+a, -a, +1.0}, {-a, +a, +1.0}, {+a, +a, -1.0}, {-a, -a, -1.0}}};
}
}  // namespace

Mat4 allocationMatrix(const QuadrotorParams& p) {
  Mat4 A;
  const auto g = motorGeometry(p.arm);
  for (int i = 0; i < 4; ++i) {
    A(0, i) = 1.0;                // collective thrust
    A(1, i) = g[i].y;             // roll torque  = sum(y_i f_i)
    A(2, i) = -g[i].x;            // pitch torque = -sum(x_i f_i)
    A(3, i) = -g[i].spin * p.km;  // yaw: a CCW rotor pushes the body clockwise
  }
  return A;
}

void MotorModel::step(const Vec4& w_cmd, double dt) {
  for (int i = 0; i < 4; ++i) {
    const double cmd = std::clamp(w_cmd(i), 0.0, p_.w_max);
    const double tau = cmd > w_(i) ? p_.tau_up : p_.tau_down;
    w_(i) += (cmd - w_(i)) * (1.0 - std::exp(-dt / tau));  // exact discretisation
  }
}

RotorWrench rotorWrench(const QuadrotorParams& p, const Vec4& w) {
  const Vec4 f = p.kf * w.cwiseProduct(w);
  const Vec4 u = allocationMatrix(p) * f;
  return {u(0), u.tail<3>()};
}

Quadrotor::Quadrotor(const QuadrotorParams& p, const Vec3& p0, double yaw0, std::uint64_t seed)
    : p_(p), motors_(p), rng_(seed) {
  s_.p = p0;
  s_.q = Quat(Eigen::AngleAxisd(yaw0, Vec3::UnitZ()));
  motors_.reset(Vec4::Constant(hoverMotorSpeed()));  // spawned hovering
  f_body_ = Vec3(0, 0, p_.g);
}

double Quadrotor::hoverMotorSpeed() const { return std::sqrt(p_.mass * p_.g / (4.0 * p_.kf)); }

void Quadrotor::step(const Vec4& motor_cmd, double dt) {
  const int n = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
  for (int i = 0; i < n; ++i) subStep(motor_cmd, dt / n);
}

void Quadrotor::subStep(const Vec4& motor_cmd, double dt) {
  motors_.step(motor_cmd, dt);
  const RotorWrench rw = rotorWrench(p_, motors_.speeds());
  const Mat3 R = s_.q.toRotationMatrix();

  // Ornstein-Uhlenbeck gust (acceleration), as in the point-mass model.
  const Vec3 n(normal_(rng_), normal_(rng_), normal_(rng_));
  gust_ += -gust_ * dt / p_.gust_tau + p_.gust_sigma * std::sqrt(2.0 * dt / p_.gust_tau) * n;

  // Translation (world): thrust + gravity + drag on airspeed + gust
  const Vec3 a = R * Vec3(0, 0, rw.thrust / p_.mass) + Vec3(0, 0, -p_.g) -
                 p_.drag * (s_.v - p_.wind) + gust_;
  f_body_ = R.transpose() * (a - Vec3(0, 0, -p_.g));

  // Rotation (body): Euler's equation J w' = tau - w x J w
  const Vec3 J = p_.inertia;
  const Vec3 w_dot = (rw.torque - s_.w.cross(J.cwiseProduct(s_.w))).cwiseQuotient(J);

  s_.p += s_.v * dt + 0.5 * a * dt * dt;
  s_.v += a * dt;
  const Vec3 dtheta = s_.w * dt + 0.5 * w_dot * dt * dt;
  if (dtheta.norm() > 1e-12) s_.q = s_.q * Quat(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
  s_.q.normalize();
  s_.w += w_dot * dt;
}

}  // namespace follow_me
