// intercept_node (phase 4): companion computer of the radar-cued interception.
//   mission (WAIT / MIDCOURSE to the PIP / HANDOVER / FOLLOW / LOST, follow_me::MissionManager)
//   -> position control (speed limit per phase) -> safety filter -> flight-controller setpoint.
//   The gimbal is pointed at the target estimate (radar cueing before the camera lock).
// Inputs : /target/estimate (fusion_node), /follower/odom (navigation), /follower/joint_states.
// Outputs: /follower/fc/accel_yaw_setpoint, gimbal commands, /mission/phase, markers,
//          /mission/debug (setpoint, command before / after the safety filter: diagnosis).
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "follow_me_core/controller.hpp"
#include "follow_me_core/mission.hpp"
#include "follow_me_ros/ros_utils.hpp"

using follow_me::Mat3;
using follow_me::MissionPhase;
using follow_me::Vec3;
using follow_me_msgs::msg::TargetEstimate;
using visualization_msgs::msg::Marker;

class InterceptNode : public rclcpp::Node {
 public:
  InterceptNode()
      : Node("intercept_node"),
        fol_(follow_me_ros::declareFollower(*this)),
        gim_(follow_me_ros::declareGimbal(*this)),
        mission_(follow_me_ros::declareIntercept(*this), follow_me_ros::declareGuidance(*this)),
        controller_(follow_me_ros::declareControl(*this), fol_.a_max, fol_.v_max),
        safety_(follow_me_ros::declareSafety(*this), fol_.a_max) {
    cam_offset_ = follow_me_ros::declareVec3(*this, "camera.offset", Vec3(0, 0, -0.25));
    size_known_rel_ = declare_parameter("intercept.size_known_rel", 0.08);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom", 1,
        [this](const nav_msgs::msg::Odometry& m) { odom_ = follow_me_ros::fromOdom(m); });
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
        "/follower/joint_states", 1,
        [this](const sensor_msgs::msg::JointState& m) { follow_me_ros::updateGimbal(m, gimbal_); });
    sub_est_ = create_subscription<TargetEstimate>("/target/estimate", 1,
                                                   [this](const TargetEstimate& m) { est_ = m; });
    pub_sp_ = create_publisher<geometry_msgs::msg::Twist>("/follower/fc/accel_yaw_setpoint", 10);
    pub_yaw_ = create_publisher<std_msgs::msg::Float64>("/follower/gimbal/yaw_cmd", 10);
    pub_pitch_ = create_publisher<std_msgs::msg::Float64>("/follower/gimbal/pitch_cmd", 10);
    pub_phase_ = create_publisher<std_msgs::msg::String>("/mission/phase", 10);
    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("/follow/markers", 10);
    pub_dbg_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mission/debug", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), [this] { tick(); });
  }

 private:
  void tick() {
    const rclcpp::Time stamp = get_clock()->now();
    const double now = stamp.seconds();
    if (now <= 0.0 || !odom_ || !gimbal_.valid) return;
    const double dt = last_ > 0.0 ? now - last_ : 0.0;
    last_ = now;
    if (dt <= 0.0) return;

    const bool has_track = est_ && est_->status != TargetEstimate::SEARCH &&
                           now - rclcpp::Time(est_->header.stamp).seconds() < 0.5;
    Vec3 p_t = Vec3::Zero(), v_t = Vec3::Zero();
    Mat3 P = Mat3::Identity();
    if (est_) {
      p_t = Vec3(est_->position.x, est_->position.y, est_->position.z);
      v_t = Vec3(est_->velocity.x, est_->velocity.y, est_->velocity.z);
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P(i, j) = est_->covariance[6 * i + j];
    }
    // Hand-over complete only when the camera is locked AND can range (size learnt to 8 %):
    // until then, slow approach and never closer than blind_hold_range.
    const bool locked = has_track && est_->camera_locked && est_->target_size > 0.0 &&
                        est_->target_size_sigma / est_->target_size < size_known_rel_;

    // ---- mission ----
    const double size = has_track ? est_->target_size : 0.0;
    const Vec3 los = (p_t - odom_->p).norm() > 1e-6 ? Vec3((p_t - odom_->p).normalized()) : Vec3::UnitX();
    const double range_sigma = std::sqrt(std::max(los.dot(P * los), 0.0));
    const auto out = mission_.update({now, odom_->p, odom_->v, has_track, locked, p_t, v_t, size, range_sigma}, dt);
    if (out.phase != last_phase_) {
      RCLCPP_INFO(get_logger(), "mission %s -> %s (range %.0f m)", follow_me::toString(last_phase_),
                  follow_me::toString(out.phase), has_track ? (p_t - odom_->p).norm() : -1.0);
      last_phase_ = out.phase;
    }
    pub_phase_->publish(std_msgs::msg::String().set__data(follow_me::toString(out.phase)));

    // ---- control + safety ----
    Vec3 a_cmd = controller_.compute(odom_->p, odom_->v, out.p_des, out.v_des, dt, out.v_max, Vec3::Zero(), out.v_rel_max,
                                     out.v_cmd_tau);
    if (out.a_max > 0) a_cmd = follow_me::saturate(a_cmd, out.a_max);
    if (out.jerk_max > 0) {  // bumpless phase transitions (and optional smooth follow), before safety
      jerk_.setMax(out.jerk_max);
      a_cmd = jerk_.apply(a_cmd, dt);
    } else {
      jerk_.reset(a_cmd);
    }
    const Vec3 a_ctrl = a_cmd;
    bool safety_active = false;
    if (has_track) {
      const auto r = safety_.filter(a_cmd, odom_->p, odom_->v, p_t, v_t, P);
      a_cmd = r.a;
      safety_active = r.active;
    }
    a_cmd = follow_me::saturate(a_cmd, fol_.a_max);
    {  // diagnosis: t, phase, p_des, v_des, v_max, command before / after the safety filter, active
      std_msgs::msg::Float64MultiArray dbg;
      dbg.data = {now, double(static_cast<int>(out.phase)), out.p_des.x(), out.p_des.y(), out.p_des.z(),
                  out.v_des.x(), out.v_des.y(), out.v_des.z(), out.v_max, a_ctrl.x(), a_ctrl.y(), a_ctrl.z(),
                  a_cmd.x(), a_cmd.y(), a_cmd.z(), safety_active ? 1.0 : 0.0};
      pub_dbg_->publish(dbg);
    }

    // ---- heading + gimbal: look at the target (radar cueing before the camera lock) ----
    double yaw_sp = hold_yaw_ ? *hold_yaw_ : odom_->yaw;
    double yaw_cmd, pitch_cmd;
    if (has_track) {
      yaw_sp = std::atan2(p_t.y() - odom_->p.y(), p_t.x() - odom_->p.x());
      hold_yaw_ = yaw_sp;
      const Vec3 los_b = odom_->q.conjugate() * (p_t - follow_me_ros::cameraPosition(*odom_, cam_offset_));
      yaw_cmd = gimbal_.yaw + follow_me::wrapAngle(std::atan2(los_b.y(), los_b.x()) - gimbal_.yaw);
      pitch_cmd = std::clamp(std::atan2(los_b.z(), std::hypot(los_b.x(), los_b.y())),
                             follow_me::deg2rad(gim_.pitch_min_deg), follow_me::deg2rad(gim_.pitch_max_deg));
    } else {
      yaw_cmd = gimbal_.yaw;
      pitch_cmd = 0.0;
    }

    geometry_msgs::msg::Twist sp;
    sp.linear.x = a_cmd.x();
    sp.linear.y = a_cmd.y();
    sp.linear.z = a_cmd.z();
    sp.angular.z = yaw_sp;
    pub_sp_->publish(sp);
    pub_yaw_->publish(std_msgs::msg::Float64().set__data(yaw_cmd));
    pub_pitch_->publish(std_msgs::msg::Float64().set__data(pitch_cmd));
    if (safety_active && !was_safety_) RCLCPP_WARN(get_logger(), "Safety filter ACTIVE");
    was_safety_ = safety_active;
    if (++tick_count_ % 5 == 0) publishMarkers(stamp, out, has_track, p_t, v_t);
  }

  void publishMarkers(const rclcpp::Time& stamp, const follow_me::MissionManager::Output& out, bool has_track,
                      const Vec3& p_t, const Vec3& v_t) {
    visualization_msgs::msg::MarkerArray arr;
    Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "world";
    m.ns = "setpoint";
    m.type = Marker::SPHERE;
    m.pose.position.x = out.p_des.x();
    m.pose.position.y = out.p_des.y();
    m.pose.position.z = out.p_des.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.6;
    m.color.g = 0.9f;
    m.color.a = 1.0f;
    arr.markers.push_back(m);
    Marker txt = m;
    txt.ns = "phase";
    txt.type = Marker::TEXT_VIEW_FACING;
    txt.pose.position = geometry_msgs::msg::Point();
    txt.pose.position.x = odom_->p.x();
    txt.pose.position.y = odom_->p.y();
    txt.pose.position.z = odom_->p.z() + 2.6;  // above the monitor "FOLLOWER" label
    txt.scale.z = 0.6;
    // Same phase colours as the HUD, the figures and the web replay.
    switch (out.phase) {
      case MissionPhase::Midcourse: txt.color = follow_me_ros::rgba(0.92f, 0.41f, 0.20f, 1.0f); break;
      case MissionPhase::Handover: txt.color = follow_me_ros::rgba(0.93f, 0.63f, 0.0f, 1.0f); break;
      case MissionPhase::Follow: txt.color = follow_me_ros::rgba(0.11f, 0.69f, 0.48f, 1.0f); break;
      default: txt.color = follow_me_ros::rgba(0.66f, 0.66f, 0.64f, 1.0f); break;
    }
    txt.text = std::string("MISSION: ") + follow_me::toString(out.phase) + (has_track ? "" : " (no track)");
    arr.markers.push_back(txt);

    // Guidance geometry (MIDCOURSE / HANDOVER): predicted intercept point, the follower's straight
    // approach to it and the target's constant-velocity prediction up to the rendezvous time.
    const bool pip = has_track && (out.phase == MissionPhase::Midcourse || out.phase == MissionPhase::Handover);
    const auto orange = follow_me_ros::rgba(0.92f, 0.41f, 0.20f, 1.0f);
    Marker aim;
    aim.header = m.header;
    aim.ns = "pip";
    aim.action = pip ? Marker::ADD : Marker::DELETE;
    aim.type = Marker::CUBE;  // a diamond: cube rotated 45 deg about x and y
    aim.pose.position = follow_me_ros::toPoint(out.p_aim);
    const Eigen::Quaterniond qd = Eigen::AngleAxisd(M_PI / 4, Vec3::UnitX()) * Eigen::AngleAxisd(M_PI / 4, Vec3::UnitY());
    aim.pose.orientation.w = qd.w();
    aim.pose.orientation.x = qd.x();
    aim.pose.orientation.y = qd.y();
    aim.pose.orientation.z = qd.z();
    aim.scale.x = aim.scale.y = aim.scale.z = 2.0;
    aim.color = orange;
    arr.markers.push_back(aim);

    Marker approach;
    approach.header = m.header;
    approach.ns = "approach";
    approach.action = aim.action;
    approach.type = Marker::LINE_LIST;  // dashed line follower -> PIP
    approach.pose.orientation.w = 1.0;
    approach.scale.x = 0.25;
    approach.color = orange;
    const Vec3 d = out.p_aim - odom_->p;
    const int n_dash = std::clamp(int(d.norm() / 6.0), 1, 80);
    for (int k = 0; k < n_dash; ++k) {
      approach.points.push_back(follow_me_ros::toPoint(odom_->p + d * (double(k) / n_dash)));
      approach.points.push_back(follow_me_ros::toPoint(odom_->p + d * ((k + 0.5) / n_dash)));
    }
    arr.markers.push_back(approach);

    Marker pred;
    pred.header = m.header;
    pred.ns = "target_prediction";
    pred.action = aim.action;
    pred.type = Marker::LINE_STRIP;
    pred.pose.orientation.w = 1.0;
    pred.scale.x = 0.2;
    pred.color = follow_me_ros::rgba(0.89f, 0.29f, 0.28f, 0.8f);
    const double horizon = out.t_go > 0 ? std::min(out.t_go, 60.0) : 10.0;
    for (int k = 0; k <= 20; ++k) pred.points.push_back(follow_me_ros::toPoint(p_t + v_t * (horizon * k / 20.0)));
    arr.markers.push_back(pred);

    Marker info = txt;
    info.ns = "pip_text";
    info.action = aim.action;
    info.pose.position = follow_me_ros::toPoint(out.p_aim + Vec3(0, 0, 3.0));
    info.scale.z = out.phase == MissionPhase::Handover ? 1.0 : 2.0;
    info.color = orange;
    char buf[64];
    // The aim is never the target itself but its STANDOFF point (8 m behind it, along its
    // velocity). Mid-course: where we meet that point in the future (PIP). Hand-over: the follower
    // matches the target velocity + 4 m/s towards the CURRENT standoff point (no rendezvous: the
    // target is faster than 4 m/s), so it is labelled as such, not as a PIP.
    if (out.phase == MissionPhase::Handover)
      std::snprintf(buf, sizeof(buf), "standoff point (8 m behind target)");
    else if (out.t_go > 0.0)
      std::snprintf(buf, sizeof(buf), "PIP  t_go %.0f s", out.t_go);
    else
      std::snprintf(buf, sizeof(buf), "standoff point (pursuit)");  // no rendezvous solution
    info.text = buf;
    arr.markers.push_back(info);

    Marker tether;  // hand-over: the standoff point hangs 8 m behind the target estimate
    tether.header = m.header;
    tether.ns = "standoff_tether";
    tether.action = (pip && out.phase == MissionPhase::Handover) ? Marker::ADD : Marker::DELETE;
    tether.type = Marker::LINE_LIST;
    tether.pose.orientation.w = 1.0;
    tether.scale.x = 0.08;
    tether.color = follow_me_ros::rgba(0.92f, 0.41f, 0.20f, 0.6f);
    tether.points.push_back(follow_me_ros::toPoint(p_t));
    tether.points.push_back(follow_me_ros::toPoint(out.p_aim));
    arr.markers.push_back(tether);
    pub_mk_->publish(arr);
  }

  follow_me::FollowerParams fol_;
  follow_me::GimbalParams gim_;
  follow_me::MissionManager mission_;
  follow_me::PositionController controller_;
  follow_me::SafetyFilter safety_;
  follow_me::JerkLimiter jerk_{-1.0};
  Vec3 cam_offset_;
  double size_known_rel_;
  std::optional<follow_me_ros::OdomState> odom_;
  follow_me_ros::GimbalState gimbal_;
  std::optional<TargetEstimate> est_;
  std::optional<double> hold_yaw_;
  MissionPhase last_phase_ = MissionPhase::Wait;
  double last_ = 0.0;
  long tick_count_ = 0;
  bool was_safety_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<TargetEstimate>::SharedPtr sub_est_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_sp_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_, pub_pitch_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_phase_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_dbg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptNode>());
  rclcpp::shutdown();
  return 0;
}
