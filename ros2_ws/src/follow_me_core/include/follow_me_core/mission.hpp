// Mission logic of the radar-cued interception (phase 4).
//
//   WAIT ──radar track──► MIDCOURSE ──range < handover_range──► HANDOVER ──camera locked──► FOLLOW
//     ▲                      │  (fly to the predicted          │ (slow approach, gimbal   │ (vision follow at
//     │                      │   rendezvous point, PIP)        │  cued by the radar track,│  the standoff,
//     └──────── LOST ◄───────┴─── no track ────────────────────┴── wait for the lock) ────┘  radar kept)
//
// Predicted Intercept Point: the target is assumed to keep its velocity; we look for the time
// tau at which the follower, flying straight at speed V, reaches the target's FUTURE standoff
// point (behind it, not the target itself!):
//     | p_t + v_t tau + offset - p_f | = V tau
//  -> (|v_t|^2 - V^2) tau^2 + 2 r.v_t tau + |r|^2 = 0,   r = p_t + offset - p_f
// The smallest positive root gives the rendezvous; none exists if the target is faster than V
// and flying away: then pure pursuit of the standoff point.
#pragma once

#include <optional>

#include "follow_me_core/guidance.hpp"
#include "follow_me_core/types.hpp"

namespace follow_me {

struct InterceptParams {
  double approach_speed = 12.0;   // midcourse speed [m/s] (v_max of the follower = 14)
  double handover_range = 80.0;   // start the hand-over below this range [m]
  double handover_speed = 4.0;    // max closing speed while waiting for the camera lock [m/s]
  double blind_hold_range = 25.0; // never come closer than this without the camera lock [m]...
  // ...unless the target is too small to learn its size from there: approach until it spans
  // resolve_px pixels (f W / resolve_px), never closer than hold_min (Monte-Carlo: 0.45 m targets
  // were 7 px at 25 m, their size never converged, the mission waited forever).
  double resolve_px = 20.0;
  double camera_fx = 400.0;       // [px]
  double hold_min = 12.0;         // [m]
  // ...and never closer than d_min + hold_margin + k_range_sigma * range_sigma: while the range
  // comes from the radar alone (sigma 3-4 m), wait further away (Monte-Carlo, random manoeuvres:
  // hand-over violations with 5-9 m range errors).
  double d_min = 4.0, hold_margin = 3.0, k_range_sigma = 3.0;
  // Low-pass on the velocity command before the follow phase. The PIP is p_t + v_t tau with
  // tau ~ 30 s: the target velocity noise of the radar track is multiplied by the time-to-go,
  // the aim point jumped by tens of metres at every scan and the follower hit its 35 deg tilt
  // limit in cruise (Gazebo). Far away a few degrees of heading error cost nothing.
  double cmd_tau_midcourse = 1.5, cmd_tau_handover = 1.0;  // [s]
  // Feed-forward of the target acceleration while matching its motion (hand-over, follow): the
  // follower brakes WITH a stopping target instead of after it (jerks of up to 30 deg tilt).
  double a_ff_gain = 0.0;  // off by default: see FusionParams::model (measured, not smoother)
  // Smooth ride in FOLLOW: the nominal acceleration is capped (the target brakes at <= 2.5 m/s^2)
  // and its rate of change limited; the safety filter keeps its full authority after that.
  // Measured: "jerks" of up to 30 deg tilt when the target stopped or restarted came from the
  // command's aggressiveness, not from the estimate's lag.
  // Measured: capping at 3-4 m/s^2 removed the tilt peaks but the follower could not keep its
  // place behind a turning 10 m/s target (distance error 0.8 -> 2-11 m). Off by default (< 0).
  double follow_a_max = -1.0;     // [m/s^2] (3.0 = ~17 deg tilt)
  double follow_jerk_max = -1.0;  // [m/s^3]
  double lock_lost_time = 3.0;    // FOLLOW without camera for this long -> back to HANDOVER [s]
  // Bumpless transfer between phases: for transition_time after a phase change, the nominal
  // acceleration may only change at transition_jerk (the safety filter keeps its full authority).
  // Measured 2026-11-19: at HANDOVER -> FOLLOW the two guidance laws aim at different points; a
  // follower that arrived in front of the target was pulled one way, then the other: the command
  // jumped by 12 m/s^2 in ONE step (thrust axis swung 60 deg in 0.4 s in Gazebo).
  // Measured in Gazebo: it only DELAYED the problem: the position error to the follow point grew
  // during the limited second, then the follower charged at 14 m/s, came within 5.9 m, braked hard
  // and lost the target from the camera (2 runs out of 2). Off; see follow_close_speed.
  double transition_jerk = -1.0;  // [m/s^3] (< 0: off)
  double transition_time = 1.5;   // [s]
  // FOLLOW: closing speed on the follow point limited relative to the target (like the hand-over's
  // +4 m/s). Without it the correction 0.8 (p_des - p) was bounded by the absolute 14 m/s only: a
  // follower that entered FOLLOW 20 m from its follow point asked for target + 16 m/s, overshot and
  // reversed at full tilt. In steady follow the error is < 1 m: the limit never acts.
  // Measured 2026-11-19: neutral offline (300 runs x 3 levels), Gazebo: no fewer tilt reversals
  // (9 vs 8), distance error 1.10 -> 1.55 m. Off by default, kept as an option.
  double follow_close_speed = -1.0;  // [m/s] (< 0: off)
  // Bumpless transfer (2026-11-19): after a phase change, the controller's velocity command starts
  // from its previous value and joins the new law with a time constant decaying from
  // bumpless_tau to 0 over bumpless_time; meanwhile the closing speed on the setpoint is limited
  // to bumpless_close_speed (no charge towards a follow point 20 m away).
  double bumpless_tau = 0.8;          // [s] (< 0: off)
  double bumpless_time = 2.0;         // [s]
  double bumpless_close_speed = 4.0;  // [m/s]
};

enum class MissionPhase : std::uint8_t { Wait = 0, Midcourse = 1, Handover = 2, Follow = 3, Lost = 4 };
const char* toString(MissionPhase p);

struct Rendezvous {
  Vec3 p_aim;      // where we meet the target's standoff point
  Vec3 v_des;      // straight-line velocity to get there
  double t_go;     // time to go [s] (infinity: pure pursuit)
  bool feasible;   // false: target faster than us and moving away
};
Rendezvous predictedIntercept(const Vec3& p_f, const Vec3& p_t, const Vec3& v_t, const Vec3& offset,
                              double speed);

// Standoff offset seen from the target: behind it (along -v) or, if slow, where we already are.
Vec3 standoffOffset(const Vec3& p_f, const Vec3& p_t, const Vec3& v_t, const GuidanceParams& g);

class MissionManager {
 public:
  MissionManager(const InterceptParams& ip, const GuidanceParams& gp) : ip_(ip), guidance_(gp), gp_(gp) {}

  struct Input {
    double t;
    Vec3 p_f, v_f;          // follower (navigation)
    bool has_track;         // radar and/or camera track
    bool camera_locked;     // hand-over criterion: camera locked AND able to range (size learnt);
                            // locking at 80 m with the range from the radar only came too close
    Vec3 p_t, v_t;          // target estimate
    double target_size = 0.0;  // current size estimate [m] (0: unknown -> fixed blind_hold_range)
    double range_sigma = 0.0;  // estimate uncertainty along the line of sight [m]
    Vec3 a_t = Vec3::Zero();   // target acceleration estimate (IMM; ~0 when it flies steadily)
  };
  struct Output {
    MissionPhase phase;
    Vec3 p_des, v_des;      // position-control setpoint
    double v_max;           // speed limit for this phase
    bool hold;              // true: hold position (no track, or blind and too close)
    Vec3 p_aim = Vec3::Zero();  // where we are heading: PIP (midcourse/hand-over), follow point (FOLLOW)
    double t_go = 0.0;          // PIP time to go [s] (0 outside midcourse/hand-over)
    Vec3 a_ff = Vec3::Zero();   // acceleration feed-forward for the position controller
    double a_max = -1.0;        // nominal acceleration cap for this phase (< 0: none)
    double jerk_max = -1.0;     // nominal jerk limit for this phase (< 0: none)
    double v_rel_max = -1.0;    // limit of the position correction kp (p_des - p) [m/s] (< 0: none)
    double v_cmd_tau = -1.0;    // bumpless transfer: smoothing of the velocity command [s] (< 0: none)
  };
  Output update(const Input& in, double dt);
  Output setpoint(const Input& in, double dt);  // the phase's guidance (update = transitions + this)
  MissionPhase phase() const { return phase_; }

 private:
  InterceptParams ip_;
  FollowGuidance guidance_;
  GuidanceParams gp_;
  MissionPhase phase_ = MissionPhase::Wait;
  double t_lock_ok_ = -1e9;
  MissionPhase last_phase_ = MissionPhase::Wait;
  double t_phase_change_ = -1e9;
  std::optional<Vec3> hold_;
  std::optional<Vec3> v_cmd_;  // filtered velocity command (MIDCOURSE / HANDOVER)
};

}  // namespace follow_me
