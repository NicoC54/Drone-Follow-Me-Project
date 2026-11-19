// The phase-4 TARGET as a real multirotor (6-DOF quadrotor + its own flight controller), flying
// the scenario of the kinematic Leader as a REFERENCE trajectory:
//   a_cmd = a_ref + kv (v_ref - v) + kp (p_ref - p),  yaw = reference heading.
// So it tilts to accelerate and to turn (its silhouette changes), cannot exceed what its thrust
// and tilt limit allow, and is pushed by the wind like the follower. physics = false: the old
// kinematic point that follows the scenario exactly (no tilt, no wind).
#pragma once

#include <optional>

#include "follow_me_core/follower_sim.hpp"
#include "follow_me_core/sim_models.hpp"

namespace follow_me {

class TargetDrone {
 public:
  TargetDrone(const LeaderParams& lp, const SimConfig& base, bool physics)
      : ref_(lp), v_ref_prev_(Vec3::Zero()) {
    if (physics) {
      SimConfig c = base;
      c.quadrotor = true;
      c.use_nav = false;  // the target's own navigation does not matter to us: truth
      c.seed = base.seed + 99;
      quad_.emplace(c, lp.start_pos, deg2rad(lp.start_heading_deg));
    }
  }

  void step(double t, double dt) {
    ref_.step(t, dt);
    if (!quad_) return;
    const Vec3 v_ref = ref_.velocity();
    const Vec3 a_ref = (v_ref - v_ref_prev_) / dt;
    v_ref_prev_ = v_ref;
    Vec3 a = a_ref + kv_ * (v_ref - quad_->velocity()) +
             kp_ * (ref_.position() - quad_->truePosition());
    if (a.norm() > a_max_) a *= a_max_ / a.norm();
    quad_->step(a, ref_.heading(), dt);
  }

  Vec3 position() const { return quad_ ? quad_->truePosition() : ref_.position(); }
  Vec3 velocity() const { return quad_ ? quad_->velocity() : ref_.velocity(); }
  double heading() const { return quad_ ? yawOf(quad_->trueAttitude()) : ref_.heading(); }
  Quat attitude() const {
    return quad_ ? quad_->trueAttitude() : Quat(Eigen::AngleAxisd(ref_.heading(), Vec3::UnitZ()));
  }
  double maxTiltDeg() const { return quad_ ? quad_->maxTiltDeg() : 0.0; }

 private:
  Leader ref_;
  std::optional<FollowerSim> quad_;
  Vec3 v_ref_prev_;
  double kv_ = 2.5, kp_ = 0.6, a_max_ = 6.0;
};

}  // namespace follow_me
