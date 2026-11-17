#include "follow_me_core/guidance.hpp"

#include <algorithm>

namespace follow_me {

FollowGuidance::Setpoint FollowGuidance::compute(const Vec3& p_f, const Vec3& p_t, const Vec3& v_t,
                                                 double dt) {
  const Vec2 rel = (p_f - p_t).head<2>();
  const Vec2 rel_dir = rel / std::max(rel.norm(), 1e-6);
  const Vec2 vh = v_t.head<2>();
  const double speed = vh.norm();
  const double w =
      std::clamp((speed - p_.v_heading_lo) / (p_.v_heading_hi - p_.v_heading_lo), 0.0, 1.0);
  const Vec2 behind = speed > 1e-6 ? Vec2(-vh / speed) : Vec2::Zero();
  Vec2 d = w * behind + (1.0 - w) * rel_dir;
  if (d.norm() < 1e-6) d = rel_dir;

  if (!angle_) angle_ = std::atan2(rel_dir.y(), rel_dir.x());
  // First-order tracking of the desired bearing, rate limited. A pure rate limit (bang-bang)
  // would make omega jump by +/- max rate at every step on a noisy heading.
  const double wmax = deg2rad(p_.dir_rate_max_deg);
  const double omega =
      std::clamp(p_.dir_gain * wrapAngle(std::atan2(d.y(), d.x()) - *angle_), -wmax, wmax);
  angle_ = wrapAngle(*angle_ + omega * dt);

  const Vec2 u(std::cos(*angle_), std::sin(*angle_));
  Setpoint sp;
  sp.p = p_t + Vec3(p_.follow_dist * u.x(), p_.follow_dist * u.y(), p_.height_offset);
  // Leader velocity + velocity of the rotating offset point.
  sp.v = v_t + p_.follow_dist * omega * Vec3(-u.y(), u.x(), 0.0);
  return sp;
}

}  // namespace follow_me
