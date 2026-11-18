// Offline simulation of the radar-cued interception (phase 4), no ROS:
//   ground radar (2 Hz, 0.3 s latency, clutter, own echo) -> FusionTracker (OOSM) -> mission
//   (WAIT / MIDCOURSE to the PIP / HANDOVER / FOLLOW) -> position control + safety -> quadrotor
//   flown by its own flight controller on its own navigation. Camera: bearing + apparent size of
//   an UNKNOWN-size drone (size learnt during the approach).
#pragma once

#include "follow_me_core/fusion_tracker.hpp"
#include "follow_me_core/mission.hpp"
#include "follow_me_core/offline_sim.hpp"

#include <string>

namespace follow_me {

struct InterceptConfig {
  SimConfig base;                 // quadrotor, navigation, control, safety, camera noise...
  GroundRadarParams radar;
  FusionParams fusion;
  InterceptParams intercept;
  double target_size = 0.6;       // TRUE size of the target drone (unknown to the tracker)
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
  double time_below_dmin = 0;
  double follow_dist_err_rms = 0;   // in FOLLOW, after 5 s
  double follow_est_err_rms = 0;    // target estimate error in FOLLOW
  double follow_pct = 0;            // share of the flight in FOLLOW
  double size_est = 0, size_sigma = 0;
  int phase_changes = 0;
  FusionTracker::Stats fusion;
};

InterceptMetrics runInterceptSim(const InterceptConfig& cfg);
void printInterceptMetrics(const InterceptMetrics& m, double target_size);

// Monte-Carlo campaign: n randomised interceptions (wind, radar latency and clutter, target
// size, sensor noise seeds). One CSV line per run: drawn parameters + metrics.
void runInterceptMonteCarlo(int n, std::uint64_t seed0, const std::string& csv_path);

}  // namespace follow_me
