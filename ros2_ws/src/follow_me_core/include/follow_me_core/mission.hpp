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
  double blind_hold_range = 25.0; // never come closer than this without the camera lock [m]
  double lock_lost_time = 3.0;    // FOLLOW without camera for this long -> back to HANDOVER [s]
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
  };
  struct Output {
    MissionPhase phase;
    Vec3 p_des, v_des;      // position-control setpoint
    double v_max;           // speed limit for this phase
    bool hold;              // true: hold position (no track, or blind and too close)
  };
  Output update(const Input& in, double dt);
  MissionPhase phase() const { return phase_; }

 private:
  InterceptParams ip_;
  FollowGuidance guidance_;
  GuidanceParams gp_;
  MissionPhase phase_ = MissionPhase::Wait;
  double t_lock_ok_ = -1e9;
  std::optional<Vec3> hold_;
};

}  // namespace follow_me
