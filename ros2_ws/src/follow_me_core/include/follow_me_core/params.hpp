// All tunable parameters, with the same defaults as the Python reference (follow_me/config.py).
#pragma once

#include <vector>

#include "follow_me_core/types.hpp"

namespace follow_me {

// One leg of the leader's flight plan, active from t_start until the next segment.
struct Segment {
  double t_start;        // [s]
  double speed;          // horizontal speed [m/s]
  double turn_rate_deg;  // [deg/s], positive = left (counter-clockwise)
  double climb_rate;     // [m/s]
};

std::vector<Segment> defaultScenario();

struct LeaderParams {
  Vec3 start_pos = Vec3(0.0, 0.0, 20.0);
  double start_heading_deg = 0.0;
  double max_accel = 2.5;   // [m/s^2]
  double tau_turn = 0.7;    // [s]
  double tau_climb = 1.0;   // [s]
  std::vector<Segment> scenario = defaultScenario();
};

struct FollowerParams {
  Vec3 start_pos = Vec3(-12.0, -5.0, 18.0);
  double tau_accel = 0.15;  // attitude loop lag [s]
  double a_max = 6.0;       // [m/s^2]
  double v_max = 14.0;      // [m/s]
  double drag = 0.15;       // linear drag on airspeed [1/s]
  Vec3 wind = Vec3(2.0, -1.0, 0.0);
  double gust_sigma = 0.4;  // [m/s^2]
  double gust_tau = 2.0;    // [s]
};

// Quadrotor X (PX4 numbering), body frame FLU (x forward, y left, z up).
//   motor 0: front-right, CCW     motor 2: front-left, CW
//   motor 1: back-left,   CCW     motor 3: back-right, CW
// Rotor constants from the Gazebo X3 example (MulticopterMotorModel).
struct QuadrotorParams {
  double mass = 1.5;                            // [kg]
  Vec3 inertia = Vec3(0.029, 0.029, 0.055);     // diagonal [kg m^2]
  double arm = 0.25;                            // centre to motor [m]
  double kf = 8.54858e-06;                      // thrust = kf * w^2 [N/(rad/s)^2]
  double km = 0.016;                            // yaw moment = km * thrust [m]
  double w_max = 1000.0;                        // max motor speed [rad/s] (T/W = 2.3)
  double tau_up = 0.0125;                       // motor spin-up time constant [s]
  double tau_down = 0.025;                      // motor spin-down time constant [s]
  double drag = 0.15;                           // linear drag on airspeed, per unit mass [1/s]
  Vec3 wind = Vec3(2.0, -1.0, 0.0);             // [m/s]
  double gust_sigma = 0.4;                      // [m/s^2]
  double gust_tau = 2.0;                        // [s]
  double g = 9.81;
};

// Attitude + rate control (inner loops of the flight controller).
struct AttitudeControlParams {
  double max_tilt_deg = 35.0;
  double min_thrust_ratio = 0.2;        // never ask less than 20 % of hover thrust
  double kp_att_rp = 8.0;               // attitude -> rate gains [1/s]
  double kp_att_yaw = 3.0;
  double rate_max_rp_deg = 220.0;
  double rate_max_yaw_deg = 120.0;
  Vec3 kp_rate = Vec3(20.0, 20.0, 10.0);  // rate -> angular acceleration [1/s]
  Vec3 ki_rate = Vec3(8.0, 8.0, 4.0);     // [1/s^2]
  double i_max = 2.0;                   // integral clamp [rad/s^2]
};

// Navigation sensors of the follower (simulated) and the matching noise model of the ESKF.
// Noise values are per sample at the given rate (typical consumer-grade MEMS IMU + GNSS).
struct NavSensorParams {
  double imu_rate = 250.0;               // [Hz]
  double accel_sigma = 0.1;              // white noise per sample [m/s^2]
  double gyro_sigma = 0.005;             // [rad/s]
  double accel_bias_sigma0 = 0.15;       // initial bias spread [m/s^2]
  double gyro_bias_sigma0 = 0.01;        // [rad/s]
  double accel_bias_rw = 0.003;          // bias random walk [m/s^2 / sqrt(s)]
  double gyro_bias_rw = 0.0003;          // [rad/s / sqrt(s)]
  double gnss_rate = 10.0;               // [Hz]
  double gnss_pos_sigma_h = 0.5;         // [m]
  double gnss_pos_sigma_v = 0.8;         // [m]
  double gnss_vel_sigma = 0.1;           // [m/s]
  double baro_rate = 25.0;               // [Hz]
  double baro_sigma = 0.3;               // [m]
  double mag_rate = 50.0;                // [Hz]
  double mag_yaw_sigma_deg = 2.0;        // heading from the magnetometer [deg]
};

struct NavFilterParams {
  // Initial uncertainty
  double init_pos_sigma = 1.0;           // [m]
  double init_vel_sigma = 0.5;           // [m/s]
  double init_tilt_sigma_deg = 2.0;
  double init_yaw_sigma_deg = 5.0;
  double gate_chi2_gnss = 22.46;         // chi2(6 dof) 99.9 %
  double gate_chi2_1d = 10.83;           // chi2(1 dof) 99.9 %
};

struct CameraParams {
  int width = 1280;
  int height = 720;
  double fx = 800.0;            // [px]
  double fy = 800.0;
  double target_size = 0.5;     // known leader diameter [m]
  double min_size_px = 3.0;
  double max_range = 80.0;      // [m]
  double sigma_px = 2.0;        // noise on the centre [px]
  double sigma_size_px = 1.0;   // noise on the apparent size [px]
};

// Synthetic detector used by the offline simulation (and for fault injection in ROS).
struct DetectorParams {
  double rate_hz = 30.0;
  double p_miss = 0.10;
  double p_outlier = 0.02;
};

struct GimbalParams {
  double gain = 8.0;               // [1/s]
  double rate_max_deg = 120.0;
  double pitch_min_deg = -90.0;
  double pitch_max_deg = 60.0;
  double search_rate_deg = 40.0;
};

struct EstimatorParams {
  double q_accel = 1.5;         // CV model white-acceleration density [m^2/s^3]
  double init_vel_sigma = 5.0;  // [m/s]
  double gate_chi2 = 16.27;     // chi2(3 dof) 99.9 %
  double gate_chi2_2d = 13.82;  // chi2(2 dof) 99.9 %, bearing-only updates
  int max_rejects = 10;
  double confirm_window = 0.3;  // [s]
  double coast_time = 0.5;      // [s]
  double lost_time = 3.0;       // [s]
};

struct GuidanceParams {
  double follow_dist = 8.0;
  double height_offset = 2.0;
  double v_heading_lo = 0.5;
  double v_heading_hi = 2.0;
  double dir_gain = 1.0;          // [1/s]
  double dir_rate_max_deg = 45.0;
};

struct SafetyParams {
  double d_min = 4.0;
  double buffer = 3.0;
  double k_sigma = 2.0;
  double k_rep = 6.0;
  double k_damp = 2.0;
  // Stopping-distance speed limit: never close in faster than sqrt(2 a_brake (d - d_min)),
  // i.e. the speed from which a_brake can still stop before d_min (CBF-like constraint).
  double a_brake = 3.0;    // braking deceleration assumed available [m/s^2] (half of a_max)
  double k_speed = 3.0;    // gain on the excess closing speed [1/s]
};

struct ControlParams {
  double kp_pos = 0.8;
  double kv = 2.5;
  double ki = 0.4;
  double i_max = 2.0;
};

}  // namespace follow_me
