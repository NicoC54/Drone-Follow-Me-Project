// intercept_node (phase 4): companion computer of the radar-cued interception.
//   mission (WAIT / MIDCOURSE to the PIP / HANDOVER / FOLLOW / LOST, follow_me::MissionManager)
//   -> position control (speed limit per phase) -> safety filter -> flight-controller setpoint.
//   The gimbal is pointed at the target estimate (radar cueing before the camera lock).
// Inputs : /target/estimate (fusion_node), /follower/odom (navigation), /follower/joint_states.
// Outputs: /follower/fc/accel_yaw_setpoint, gimbal commands, /mission/phase, markers.
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
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
    const auto out = mission_.update({now, odom_->p, odom_->v, has_track, locked, p_t, v_t}, dt);
    if (out.phase != last_phase_) {
      RCLCPP_INFO(get_logger(), "mission %s -> %s (range %.0f m)", follow_me::toString(last_phase_),
                  follow_me::toString(out.phase), has_track ? (p_t - odom_->p).norm() : -1.0);
      last_phase_ = out.phase;
    }
    pub_phase_->publish(std_msgs::msg::String().set__data(follow_me::toString(out.phase)));

    // ---- control + safety ----
    Vec3 a_cmd = controller_.compute(odom_->p, odom_->v, out.p_des, out.v_des, dt, out.v_max);
    bool safety_active = false;
    if (has_track) {
      const auto r = safety_.filter(a_cmd, odom_->p, odom_->v, p_t, v_t, P);
      a_cmd = r.a;
      safety_active = r.active;
    }
    a_cmd = follow_me::saturate(a_cmd, fol_.a_max);

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
    if (++tick_count_ % 5 == 0) publishMarkers(stamp, out, has_track);
  }

  void publishMarkers(const rclcpp::Time& stamp, const follow_me::MissionManager::Output& out, bool has_track) {
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
    txt.pose.position.z = odom_->p.z() + 1.5;
    txt.scale.z = 0.6;  // readable in the close follow view
    txt.color.r = txt.color.g = txt.color.b = 1.0f;
    txt.text = std::string("MISSION: ") + follow_me::toString(out.phase) + (has_track ? "" : " (no track)");
    arr.markers.push_back(txt);
    pub_mk_->publish(arr);
  }

  follow_me::FollowerParams fol_;
  follow_me::GimbalParams gim_;
  follow_me::MissionManager mission_;
  follow_me::PositionController controller_;
  follow_me::SafetyFilter safety_;
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
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptNode>());
  rclcpp::shutdown();
  return 0;
}
