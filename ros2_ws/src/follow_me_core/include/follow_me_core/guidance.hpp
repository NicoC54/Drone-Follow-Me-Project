// Guidance: where should the follower be, given the estimated leader state?
//
// Follow point = leader + follow_dist behind (along its horizontal velocity) + height_offset up.
// When the leader is slow its heading is noise, so "behind" blends towards the current
// bearing leader -> follower. The bearing goes through a rate-limited first-order filter.
#pragma once

#include <optional>

#include "follow_me_core/params.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

class FollowGuidance {
 public:
  explicit FollowGuidance(const GuidanceParams& p) : p_(p) {}

  void reset() { angle_.reset(); }
  // Standoff distance used from now on (agility-scaled by the mission; default GuidanceParams).
  void setFollowDist(double d) { p_.follow_dist = d; }

  struct Setpoint {
    Vec3 p;  // desired follower position
    Vec3 v;  // feed-forward velocity
  };
  Setpoint compute(const Vec3& p_follower, const Vec3& p_target, const Vec3& v_target, double dt);

 private:
  GuidanceParams p_;
  std::optional<double> angle_;  // bearing of the follow point, seen from the leader
};

}  // namespace follow_me
