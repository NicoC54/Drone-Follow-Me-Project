// Small helpers shared by the nodes: message conversions, parameter declaration, camera pose.
#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <string>
#include <vector>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/fusion_tracker.hpp"
#include "follow_me_core/mission.hpp"
#include "follow_me_core/params.hpp"
#include "follow_me_core/radar_model.hpp"

namespace follow_me_ros {

using follow_me::Mat3;
using follow_me::Vec3;

// ---------- state from Gazebo ----------
struct OdomState {
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();  // world frame
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  double yaw = 0.0;
  double stamp = 0.0;
  Mat3 P_pos = Mat3::Zero();  // position covariance (navigation estimate; zero for truth)
  double sigma_att = 0.0;     // attitude std, worst axis [rad]
};

inline OdomState fromOdom(const nav_msgs::msg::Odometry& m) {
  OdomState s;
  const auto& P = m.pose.pose.position;
  const auto& Q = m.pose.pose.orientation;
  const auto& V = m.twist.twist.linear;
  s.p = Vec3(P.x, P.y, P.z);
  s.q = Eigen::Quaterniond(Q.w, Q.x, Q.y, Q.z);
  s.v = s.q * Vec3(V.x, V.y, V.z);  // OdometryPublisher gives the twist in the body frame
  s.yaw = std::atan2(2.0 * (Q.w * Q.z + Q.x * Q.y), 1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));
  s.stamp = rclcpp::Time(m.header.stamp).seconds();
  const auto& C = m.pose.covariance;  // 6x6 row-major [x y z rot_x rot_y rot_z]
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) s.P_pos(i, j) = C[6 * i + j];
  s.sigma_att = std::sqrt(std::max({C[21], C[28], C[35], 0.0}));
  return s;
}

// Measured gimbal joint angles (relative to the follower body).
struct GimbalState {
  double yaw = 0.0;
  double pitch = 0.0;  // positive = camera looks up
  bool valid = false;
};

inline void updateGimbal(const sensor_msgs::msg::JointState& js, GimbalState& g) {
  for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
    if (js.name[i] == "gimbal_yaw") g.yaw = js.position[i];
    if (js.name[i] == "gimbal_pitch") g.pitch = js.position[i];
  }
  g.valid = true;
}

// Camera optical centre and world->camera rotation R_cw, using the FULL body attitude:
//   R_world_link = R_body * Rz(gimbal_yaw) * R(-y axis, gimbal_pitch)
// Gazebo camera link: x forward, y left, z up. Our camera frame: x right, y down, z forward.
// (The body is not perfectly level: the gimbal motors' reaction torque makes it roll a bit.)
inline Vec3 cameraPosition(const OdomState& o, const Vec3& offset) { return o.p + o.q * offset; }
inline Mat3 cameraRotation(const OdomState& o, const GimbalState& g) {
  const Mat3 R_wl = o.q.toRotationMatrix() *
                    Eigen::AngleAxisd(g.yaw, Vec3::UnitZ()).toRotationMatrix() *
                    Eigen::AngleAxisd(g.pitch, -Vec3::UnitY()).toRotationMatrix();
  Mat3 R_cw;
  R_cw.row(0) = -R_wl.col(1).transpose();  // right   = -left
  R_cw.row(1) = -R_wl.col(2).transpose();  // down    = -up
  R_cw.row(2) = R_wl.col(0).transpose();   // forward
  return R_cw;
}

// ---------- parameters (defaults = Gazebo camera 640x360, 77 deg) ----------
inline Vec3 declareVec3(rclcpp::Node& n, const std::string& name, const Vec3& def) {
  const auto v = n.declare_parameter<std::vector<double>>(name, {def.x(), def.y(), def.z()});
  return v.size() == 3 ? Vec3(v[0], v[1], v[2]) : def;
}

inline follow_me::CameraParams declareCamera(rclcpp::Node& n) {
  follow_me::CameraParams c;
  c.width = n.declare_parameter("camera.width", 640);
  c.height = n.declare_parameter("camera.height", 360);
  c.fx = n.declare_parameter("camera.fx", 400.0);
  c.fy = n.declare_parameter("camera.fy", 400.0);
  c.target_size = n.declare_parameter("camera.target_size", 0.5);
  c.min_size_px = n.declare_parameter("camera.min_size_px", 3.0);
  c.max_range = n.declare_parameter("camera.max_range", 120.0);
  c.sigma_px = n.declare_parameter("camera.sigma_px", 1.0);
  c.sigma_size_px = n.declare_parameter("camera.sigma_size_px", 1.0);
  return c;
}

inline follow_me::EstimatorParams declareEstimator(rclcpp::Node& n) {
  follow_me::EstimatorParams e;
  e.q_accel = n.declare_parameter("estimator.q_accel", e.q_accel);
  e.init_vel_sigma = n.declare_parameter("estimator.init_vel_sigma", e.init_vel_sigma);
  e.gate_chi2 = n.declare_parameter("estimator.gate_chi2", e.gate_chi2);
  e.max_rejects = n.declare_parameter("estimator.max_rejects", e.max_rejects);
  e.confirm_window = n.declare_parameter("estimator.confirm_window", e.confirm_window);
  e.coast_time = n.declare_parameter("estimator.coast_time", e.coast_time);
  e.lost_time = n.declare_parameter("estimator.lost_time", e.lost_time);
  return e;
}

inline follow_me::GuidanceParams declareGuidance(rclcpp::Node& n) {
  follow_me::GuidanceParams g;
  g.follow_dist = n.declare_parameter("guidance.follow_dist", g.follow_dist);
  g.height_offset = n.declare_parameter("guidance.height_offset", g.height_offset);
  g.v_heading_lo = n.declare_parameter("guidance.v_heading_lo", g.v_heading_lo);
  g.v_heading_hi = n.declare_parameter("guidance.v_heading_hi", g.v_heading_hi);
  g.dir_gain = n.declare_parameter("guidance.dir_gain", g.dir_gain);
  g.dir_rate_max_deg = n.declare_parameter("guidance.dir_rate_max_deg", g.dir_rate_max_deg);
  return g;
}

inline follow_me::SafetyParams declareSafety(rclcpp::Node& n) {
  follow_me::SafetyParams s;
  s.d_min = n.declare_parameter("safety.d_min", s.d_min);
  s.buffer = n.declare_parameter("safety.buffer", s.buffer);
  s.k_sigma = n.declare_parameter("safety.k_sigma", s.k_sigma);
  s.k_rep = n.declare_parameter("safety.k_rep", s.k_rep);
  s.k_damp = n.declare_parameter("safety.k_damp", s.k_damp);
  s.a_brake = n.declare_parameter("safety.a_brake", s.a_brake);
  s.k_speed = n.declare_parameter("safety.k_speed", s.k_speed);
  return s;
}

inline follow_me::ControlParams declareControl(rclcpp::Node& n) {
  follow_me::ControlParams c;
  c.kp_pos = n.declare_parameter("control.kp_pos", c.kp_pos);
  c.kv = n.declare_parameter("control.kv", c.kv);
  c.ki = n.declare_parameter("control.ki", c.ki);
  c.i_max = n.declare_parameter("control.i_max", c.i_max);
  return c;
}

inline follow_me::FollowerParams declareFollower(rclcpp::Node& n) {
  follow_me::FollowerParams f;
  f.tau_accel = n.declare_parameter("follower.tau_accel", f.tau_accel);
  f.a_max = n.declare_parameter("follower.a_max", f.a_max);
  f.v_max = n.declare_parameter("follower.v_max", f.v_max);
  f.drag = n.declare_parameter("follower.drag", f.drag);
  f.wind = declareVec3(n, "follower.wind", f.wind);
  f.gust_sigma = n.declare_parameter("follower.gust_sigma", f.gust_sigma);
  f.gust_tau = n.declare_parameter("follower.gust_tau", f.gust_tau);
  return f;
}

inline follow_me::GimbalParams declareGimbal(rclcpp::Node& n) {
  follow_me::GimbalParams g;
  g.gain = n.declare_parameter("gimbal.gain", g.gain);
  g.pitch_min_deg = n.declare_parameter("gimbal.pitch_min_deg", g.pitch_min_deg);
  g.pitch_max_deg = n.declare_parameter("gimbal.pitch_max_deg", g.pitch_max_deg);
  g.search_rate_deg = n.declare_parameter("gimbal.search_rate_deg", g.search_rate_deg);
  return g;
}

inline follow_me::GroundRadarParams declareRadar(rclcpp::Node& n) {
  follow_me::GroundRadarParams r;
  r.position = declareVec3(n, "radar.position", r.position);
  r.scan_rate = n.declare_parameter("radar.scan_rate", r.scan_rate);
  r.latency = n.declare_parameter("radar.latency", r.latency);
  r.max_range = n.declare_parameter("radar.max_range", r.max_range);
  r.sigma_range = n.declare_parameter("radar.sigma_range", r.sigma_range);
  r.sigma_az_deg = n.declare_parameter("radar.sigma_az_deg", r.sigma_az_deg);
  r.sigma_el_deg = n.declare_parameter("radar.sigma_el_deg", r.sigma_el_deg);
  r.sigma_rdot = n.declare_parameter("radar.sigma_rdot", r.sigma_rdot);
  r.p_detect = n.declare_parameter("radar.p_detect", r.p_detect);
  r.clutter_per_scan = n.declare_parameter("radar.clutter_per_scan", r.clutter_per_scan);
  return r;
}

inline follow_me::FusionParams declareFusion(rclcpp::Node& n) {
  follow_me::FusionParams f;
  f.q_accel = n.declare_parameter("fusion.q_accel", f.q_accel);
  f.coast_time = n.declare_parameter("fusion.coast_time", f.coast_time);
  f.lost_time = n.declare_parameter("fusion.lost_time", f.lost_time);
  f.lock_count = n.declare_parameter("fusion.lock_count", f.lock_count);
  f.size_prior = n.declare_parameter("fusion.size_prior", f.size_prior);
  f.size_prior_sigma = n.declare_parameter("fusion.size_prior_sigma", f.size_prior_sigma);
  return f;
}

inline follow_me::InterceptParams declareIntercept(rclcpp::Node& n) {
  follow_me::InterceptParams i;
  i.approach_speed = n.declare_parameter("intercept.approach_speed", i.approach_speed);
  i.handover_range = n.declare_parameter("intercept.handover_range", i.handover_range);
  i.handover_speed = n.declare_parameter("intercept.handover_speed", i.handover_speed);
  i.blind_hold_range = n.declare_parameter("intercept.blind_hold_range", i.blind_hold_range);
  i.lock_lost_time = n.declare_parameter("intercept.lock_lost_time", i.lock_lost_time);
  return i;
}

}  // namespace follow_me_ros
