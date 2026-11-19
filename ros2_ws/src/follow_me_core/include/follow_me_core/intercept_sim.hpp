// Offline simulation of the radar-cued interception (phase 4), no ROS:
//   ground radar (2 Hz, 0.3 s latency, clutter, own echo) -> FusionTracker (OOSM) -> mission
//   (WAIT / MIDCOURSE to the PIP / HANDOVER / FOLLOW) -> position control + safety -> quadrotor
//   flown by its own flight controller on its own navigation. Camera: bearing + apparent size of
//   an UNKNOWN-size drone (size learnt during the approach).
#pragma once

#include "follow_me_core/fusion_tracker.hpp"
#include "follow_me_core/mission.hpp"
#include "follow_me_core/offline_sim.hpp"

#include <functional>
#include <string>
#include <vector>

namespace follow_me {

struct InterceptConfig {
  SimConfig base;                 // quadrotor, navigation, control, safety, camera noise...
  GroundRadarParams radar;
  // Additional ground radars (same noise model, other sites): their ranges cross the first
  // radar's coarse angles -> trilateration. Default: none.
  std::vector<GroundRadarParams> extra_radars;
  static GroundRadarParams secondRadarSite(const GroundRadarParams& like);  // (450, -100, 3)
  // "north": (230, 420, 3), closes the triangle; "center": (230, 150, 3), UNDER the engagement
  // area: it sees the target at high elevation, so its precise range constrains the altitude.
  static GroundRadarParams thirdRadarSite(const GroundRadarParams& like, const std::string& where);
  FusionParams fusion;
  InterceptParams intercept;
  double target_size = 0.6;       // TRUE size of the target drone (unknown to the tracker)
  // Aspect: the apparent width of an X quadrotor depends on its yaw relative to the line of
  // sight (narrowest seen from behind, arms at +-45 deg; widest seen along a diagonal). The TRUE
  // apparent size is target_size * (1 + aspect_gain * (g(psi) - 1)), g = X geometry, g(0) = 1.
  // Gazebo grey quadrotor, bounding-box size: +17 % at 32 deg -> aspect_gain 0.7 (--aspect 0.7
  // reproduces the Gazebo NEES problem). The detector now measures a yaw-invariant size
  // (second moment), hence the default 0. 0 = round target.
  double aspect_gain = 0.0;
  // Leader flight plan: 0 = the scripted scenario, 1 / 2 = random manoeuvres (moderate /
  // aggressive), drawn from maneuver_seed. Altitude is kept within 25-60 m.
  int maneuver_level = 0;
  // The target is a real quadrotor flying the scenario as a reference (tilts, thrust limit, wind),
  // false: kinematic point that follows the scenario exactly (target_drone.hpp).
  bool target_physics = true;
  std::uint64_t maneuver_seed = 1;
  double camera_max_range = 300.0;
  Vec3 follower_start = Vec3(-50.0, -30.0, 20.0);
  // Optional logs for figures, GIF and the web replay (empty = no log):
  //   <out_dir>/intercept.csv (10 Hz state), <out_dir>/radar.csv (every radar plot)
  std::string out_dir;
  InterceptConfig();              // sets the intercept scenario defaults
};

struct InterceptMetrics {
  double t_track = -1, t_lock = -1, t_follow = -1;  // first radar track, camera lock, FOLLOW [s]
  double initial_range = 0;
  double min_distance = 1e9;
  double t_min_distance = 0;        // when, and in which mission phase
  int phase_min_distance = 0;
  double time_below_dmin = 0;
  double follow_dist_err_rms = 0;   // in FOLLOW, after 5 s
  double follow_est_err_rms = 0;    // target estimate error in FOLLOW
  double follow_pct = 0;            // share of the flight in FOLLOW
  double size_est = 0, size_sigma = 0;
  double follow_nees = 0;
  double handover_nees = 0;
  double target_max_tilt = 0;       // [deg] (0 for a kinematic target)
  double follow_acc_p95 = 0, follow_tilt_p95 = 0;  // smoothness in FOLLOW [m/s^2], [deg]
  double radar_phase_err_rms = 0;   // estimate error in MIDCOURSE + HANDOVER (radar-led) [m]
  double radar_phase_err_h = 0, radar_phase_err_v = 0;  // its horizontal / vertical parts [m]         // target NEES during the hand-over (radar range + camera bearing)           // target NEES (position + velocity, ideal 6) in FOLLOW
  int phase_changes = 0;
  FusionTracker::Stats fusion;
};

InterceptMetrics runInterceptSim(const InterceptConfig& cfg);
void printInterceptMetrics(const InterceptMetrics& m, double target_size);

// Monte-Carlo campaign: n randomised interceptions (wind, radar latency and clutter, target
// size, sensor noise seeds). One CSV line per run: drawn parameters + metrics.
// tweak: optional change applied to every run's configuration (experiments: --set key=value).
using ConfigTweak = std::function<void(InterceptConfig&)>;
void runInterceptMonteCarlo(int n, std::uint64_t seed0, const std::string& csv_path, int maneuver_level = 0,
                            const ConfigTweak& tweak = {});
// Replays run `index` of that campaign alone (same draws), with logs in out_dir.
InterceptMetrics replayMonteCarloRun(int index, std::uint64_t seed0, int maneuver_level, const std::string& out_dir,
                                     InterceptConfig* cfg_out = nullptr, const ConfigTweak& tweak = {});

}  // namespace follow_me
