// Offline closed-loop simulation (no ROS, no Gazebo): the C++ twin of follow_me/sim.py.
// Used to validate the library against the Python reference and in the unit tests.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "follow_me_core/params.hpp"

namespace follow_me {

struct SimConfig {
  double dt = 0.01;
  double duration = 90.0;
  std::uint64_t seed = 0;
  std::vector<std::pair<double, double>> occlusions;  // [t0, t1) windows without detection
  std::string csv_path;                               // optional log file
  // Detection latency: time between image capture and the detection reaching the tracker.
  double detection_latency = 0.0;  // [s]
  // true: fuse with the camera pose and time of ARRIVAL (the bug seen in Gazebo with the GUI).
  // false: fuse with the pose and time of CAPTURE (correct).
  bool naive_latency = false;
  // Follower model: true = 6-DOF quadrotor + attitude/rate controller (500 Hz inner loop),
  // false = point mass driven by the acceleration command (phase 1 model).
  bool quadrotor = true;
  // Quadrotor only: true = every GNC block uses the ESKF navigation estimate (IMU + GNSS + baro +
  // mag); false = they use the true state (perfect navigation).
  bool use_nav = true;
  // Add the camera pose uncertainty (from the navigation covariance) to the tracker's
  // measurement noise. Without it the tracker is over-confident (target NEES ~44 instead of ~6).
  bool nav_noise_in_tracker = true;
  LeaderParams leader;
  FollowerParams follower;
  QuadrotorParams quad;
  AttitudeControlParams attitude;
  NavSensorParams nav_sensors;
  NavFilterParams nav_filter;
  CameraParams camera;
  DetectorParams detector;
  GimbalParams gimbal;
  EstimatorParams estimator;
  GuidanceParams guidance;
  SafetyParams safety;
  ControlParams control;
};

struct SimMetrics {
  double est_pos_rmse = 0;    // [m]
  double est_vel_rmse = 0;    // [m/s]
  double mean_nees = 0;       // ideal: 6
  double dist_err_rms = 0;    // [m]
  double min_distance = 1e9;  // [m]
  double time_below_dmin = 0; // [s]
  double in_frame_pct = 0;
  double valid_track_pct = 0;
  double safety_active_pct = 0;
  int outliers_total = 0;
  int outliers_rejected = 0;
  double max_tilt_deg = 0;     // quadrotor only
  double mixer_sat_pct = 0;    // quadrotor only: share of inner steps with mixer saturation
  double nav_pos_rmse = 0;     // navigation (ESKF) position error [m]
  double nav_att_rmse_deg = 0; // navigation attitude error [deg]
};

SimMetrics runOfflineSim(const SimConfig& cfg);
void printMetrics(const SimMetrics& m);

}  // namespace follow_me
