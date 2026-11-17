#include "follow_me_core/attitude_control.hpp"

#include <algorithm>
#include <cmath>

namespace follow_me {

MulticopterController::MulticopterController(const AttitudeControlParams& c,
                                             const QuadrotorParams& q)
    : c_(c), q_(q), A_inv_(allocationMatrix(q).inverse()), f_max_(q.kf * q.w_max * q.w_max) {}

Vec4 MulticopterController::mix(double thrust, const Vec3& torque, bool* saturated) const {
  auto fits = [&](const Vec4& f) { return f.minCoeff() >= 0.0 && f.maxCoeff() <= f_max_; };
  Vec4 u(thrust, torque.x(), torque.y(), torque.z());
  Vec4 f = A_inv_ * u;
  *saturated = false;
  if (fits(f)) return f;
  *saturated = true;
  // 1. Yaw has the lowest priority: keep the largest fraction k of the yaw torque that fits
  //    (dropping it completely would stop the vehicle from turning while it tilts hard).
  const double yaw = u(3);
  u(3) = 0.0;
  const Vec4 f_no_yaw = A_inv_ * u;
  if (fits(f_no_yaw)) {
    const Vec4 f_yaw = A_inv_.col(3) * yaw;
    double lo_k = 0.0, hi_k = 1.0;
    for (int i = 0; i < 20; ++i) {  // bisection on k (the feasible set is an interval)
      const double k = 0.5 * (lo_k + hi_k);
      (fits(f_no_yaw + k * f_yaw) ? lo_k : hi_k) = k;
    }
    return f_no_yaw + lo_k * f_yaw;
  }
  f = f_no_yaw;
  // 2. Keep roll/pitch differential thrust, shift the collective to fit (altitude yields).
  const double lo = f.minCoeff(), hi = f.maxCoeff();
  if (hi - lo <= f_max_) {
    f.array() += lo < 0.0 ? -lo : f_max_ - hi;
    return f;
  }
  // 3. Even that is impossible: scale the differential part down, centre it in the range.
  const double mean = f.mean();
  f = (f.array() - mean) * (f_max_ / (hi - lo)) + 0.5 * f_max_;
  return f.cwiseMax(0.0).cwiseMin(f_max_);
}

MulticopterController::Output MulticopterController::update(const Vec3& a_des, double yaw_des,
                                                            const Quat& q, const Vec3& w,
                                                            double dt) {
  const double m = q_.mass, g = q_.g;
  // --- 1. Thrust vector and tilt limit ---
  Vec3 F = m * (a_des + Vec3(0, 0, g));
  F.z() = std::max(F.z(), c_.min_thrust_ratio * m * g);
  const double h = F.head<2>().norm();
  const double h_max = F.z() * std::tan(deg2rad(c_.max_tilt_deg));
  if (h > h_max) F.head<2>() *= h_max / h;

  // --- 2. Desired attitude: body z along F, heading yaw_des ---
  const Vec3 b3 = F.normalized();
  const Vec3 b1c(std::cos(yaw_des), std::sin(yaw_des), 0.0);
  const Vec3 b2 = b3.cross(b1c).normalized();
  const Vec3 b1 = b2.cross(b3);
  Mat3 Rd;
  Rd << b1, b2, b3;
  Output out;
  out.q_des = Quat(Rd);

  // --- 3. Collective thrust: projection on the CURRENT body z (no thrust while tilted wrong) ---
  const Mat3 R = q.toRotationMatrix();
  out.thrust = std::clamp(F.dot(R.col(2)), 0.0, 4.0 * q_.kf * q_.w_max * q_.w_max);

  // --- 4. Attitude loop: error quaternion in body frame -> desired rates ---
  Quat qe = q.conjugate() * out.q_des;
  if (qe.w() < 0) qe.coeffs() *= -1.0;  // shortest rotation
  const Vec3 e = 2.0 * qe.vec();        // ~ rotation vector for small errors
  const double rp_max = deg2rad(c_.rate_max_rp_deg), y_max = deg2rad(c_.rate_max_yaw_deg);
  out.rate_des = Vec3(std::clamp(c_.kp_att_rp * e.x(), -rp_max, rp_max),
                      std::clamp(c_.kp_att_rp * e.y(), -rp_max, rp_max),
                      std::clamp(c_.kp_att_yaw * e.z(), -y_max, y_max));

  // --- 5. Rate loop: PI on angular acceleration, times inertia, + gyroscopic compensation ---
  const Vec3 e_w = out.rate_des - w;
  integral_ = saturate(integral_ + c_.ki_rate.cwiseProduct(e_w) * dt, c_.i_max);
  const Vec3 alpha = c_.kp_rate.cwiseProduct(e_w) + integral_;
  const Vec3 J = q_.inertia;
  out.torque = J.cwiseProduct(alpha) + w.cross(J.cwiseProduct(w));

  // --- 6. Mixer -> motor speeds ---
  const Vec4 f = mix(out.thrust, out.torque, &out.saturated);
  out.motor_cmd = (f / q_.kf).cwiseSqrt();
  return out;
}

}  // namespace follow_me
