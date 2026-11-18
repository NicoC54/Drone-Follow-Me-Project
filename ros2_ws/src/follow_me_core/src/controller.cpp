#include "follow_me_core/controller.hpp"

#include <algorithm>

namespace follow_me {

Vec3 PositionController::compute(const Vec3& p, const Vec3& v, const Vec3& p_des,
                                 const Vec3& v_des, double dt, double v_max_override) {
  const double v_max = v_max_override > 0.0 ? std::min(v_max_override, v_max_) : v_max_;
  const Vec3 v_cmd = saturate(v_des + p_.kp_pos * (p_des - p), v_max);
  const Vec3 e_v = v_cmd - v;
  integral_ = saturate(integral_ + p_.ki * e_v * dt, p_.i_max);  // clamped: anti-windup
  return saturate(p_.kv * e_v + integral_, a_max_);
}

SafetyFilter::Result SafetyFilter::filter(const Vec3& a_cmd, const Vec3& p_f, const Vec3& v_f,
                                          const Vec3& p_t, const Vec3& v_t,
                                          const Mat3& P_pos) const {
  const Vec3 rel = p_f - p_t;
  const double d = std::max(rel.norm(), 1e-6);
  const Vec3 n = rel / d;  // unit vector leader -> follower
  const double sigma = std::sqrt(std::max(n.dot(P_pos * n), 0.0));
  const double margin = d - p_.k_sigma * sigma - p_.d_min;
  const double closing = -(v_f - v_t).dot(n);  // > 0 when getting closer
  Vec3 a = a_cmd;
  bool active = false;

  // 1. Stopping-distance speed limit (active at any distance).
  const double v_safe = std::sqrt(2.0 * p_.a_brake * std::max(margin, 0.0));
  if (closing > v_safe) {
    const double towards = -a.dot(n);
    if (towards > 0) a += towards * n;                  // stop pushing towards the leader
    a += p_.k_speed * (closing - v_safe) * n;           // and brake the excess speed
    active = true;
  }

  // 2. Buffer zone: repulsion + damping.
  if (margin < p_.buffer) {
    const double s = std::clamp(1.0 - margin / p_.buffer, 0.0, 2.0);  // 0 at buffer edge, 1 at d_min
    const double towards = -a.dot(n);
    if (towards > 0) a += std::min(s, 1.0) * towards * n;
    a += (p_.k_rep * s + p_.k_damp * std::max(closing, 0.0) * std::min(s, 1.0)) * n;
    active = true;
  }
  return {active ? saturate(a, a_max_) : a_cmd, active};
}

}  // namespace follow_me
