// follow_node: guidance + control + safety + gimbal pointing.
// Inputs : leader estimate (/target/estimate), follower odometry, gimbal joint angles.
// Outputs: acceleration + yaw setpoint for the flight controller (/follower/fc/accel_yaw_setpoint),
//          gimbal joint angle commands, markers.
// It runs on the follower's NAVIGATION estimate (/follower/odom), never on the ground truth.
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "follow_me_core/controller.hpp"
#include "follow_me_core/guidance.hpp"
#include "follow_me_ros/ros_utils.hpp"

using follow_me::Mat3;
using follow_me::Vec3;
using follow_me_msgs::msg::TargetEstimate;
using visualization_msgs::msg::Marker;

class FollowNode : public rclcpp::Node {
 public:
  FollowNode()
      : Node("follow_node"),
        fol_(follow_me_ros::declareFollower(*this)),
        gim_(follow_me_ros::declareGimbal(*this)),
        guidance_(follow_me_ros::declareGuidance(*this)),
        controller_(follow_me_ros::declareControl(*this), fol_.a_max, fol_.v_max),
        safety_prm_(follow_me_ros::declareSafety(*this)),
        safety_(safety_prm_, fol_.a_max) {
    cam_offset_ = follow_me_ros::declareVec3(*this, "camera.offset", Vec3(0, 0, -0.25));
    estimate_timeout_ = declare_parameter("estimate_timeout", 0.5);

    // Queue of 1: control must use the FRESHEST state. With a deeper queue, under CPU load the
    // controller would act on states tens of ms old (a delay in the loop -> jerky motion).
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom", 1,
        [this](const nav_msgs::msg::Odometry& m) { odom_ = follow_me_ros::fromOdom(m); have_odom_ = true; });
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
        "/follower/joint_states", 1,
        [this](const sensor_msgs::msg::JointState& m) { follow_me_ros::updateGimbal(m, gimbal_); });
    sub_est_ = create_subscription<TargetEstimate>(
        "/target/estimate", 1, [this](const TargetEstimate& m) { est_ = m; });

    pub_sp_ = create_publisher<geometry_msgs::msg::Twist>("/follower/fc/accel_yaw_setpoint", 10);
    pub_yaw_ = create_publisher<std_msgs::msg::Float64>("/follower/gimbal/yaw_cmd", 10);
    pub_pitch_ = create_publisher<std_msgs::msg::Float64>("/follower/gimbal/pitch_cmd", 10);
    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("/follow/markers", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01),
                                  [this] { tick(); });
  }

 private:
  void tick() {
    const rclcpp::Time stamp = get_clock()->now();
    const double now = stamp.seconds();
    if (now <= 0.0 || !have_odom_ || !gimbal_.valid) return;
    const double dt = last_ > 0.0 ? now - last_ : 0.0;
    last_ = now;
    if (dt <= 0.0) return;

    const bool has_est = est_ && est_->status != TargetEstimate::SEARCH &&
                         now - rclcpp::Time(est_->header.stamp).seconds() < estimate_timeout_;
    Vec3 p_des, v_des = Vec3::Zero(), a_cmd;
    bool safety_active = false;
    double yaw_cmd, pitch_cmd, yaw_sp;

    if (has_est) {
      const Vec3 p_t(est_->position.x, est_->position.y, est_->position.z);
      const Vec3 v_t(est_->velocity.x, est_->velocity.y, est_->velocity.z);
      Mat3 P_pos;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_pos(i, j) = est_->covariance[6 * i + j];

      // Guidance -> control -> safety
      const auto sp = guidance_.compute(odom_.p, p_t, v_t, dt);
      p_des = sp.p;
      v_des = sp.v;
      hold_.reset();
      a_cmd = controller_.compute(odom_.p, odom_.v, p_des, v_des, dt);
      const auto r = safety_.filter(a_cmd, odom_.p, odom_.v, p_t, v_t, P_pos);
      a_cmd = r.a;
      safety_active = r.active;
      last_target_ = LastTarget{p_t, P_pos, now};

      // Heading: the drone faces the leader, so the gimbal works near its neutral yaw.
      yaw_sp = std::atan2(p_t.y() - odom_.p.y(), p_t.x() - odom_.p.x());
      hold_yaw_.reset();

      // Gimbal: line of sight expressed in the BODY frame (the body tilts up to 35 deg), then
      // yaw joint (about body z) and pitch joint. The yaw joint is continuous: shortest way.
      const Vec3 los_b = odom_.q.conjugate() *
                         (p_t - follow_me_ros::cameraPosition(odom_, cam_offset_));
      const double yaw_j = std::atan2(los_b.y(), los_b.x());
      const double pitch_j = std::atan2(los_b.z(), std::hypot(los_b.x(), los_b.y()));
      yaw_cmd = gimbal_.yaw + follow_me::wrapAngle(yaw_j - gimbal_.yaw);
      pitch_cmd = std::clamp(pitch_j, follow_me::deg2rad(gim_.pitch_min_deg),
                             follow_me::deg2rad(gim_.pitch_max_deg));
    } else {
      // SEARCH: hold position, sweep the horizon with the gimbal.
      if (!hold_) {
        hold_ = odom_.p;
        guidance_.reset();
        controller_.reset();
        RCLCPP_WARN(get_logger(), "No target: holding position and searching");
      }
      if (!hold_yaw_) hold_yaw_ = odom_.yaw;
      yaw_sp = *hold_yaw_;
      p_des = *hold_;
      a_cmd = controller_.compute(odom_.p, odom_.v, p_des, v_des, dt);
      // The leader is still out there: keep the safety filter on its last known position for a
      // few seconds, with an uncertainty growing like a 3 m/s unknown motion.
      if (last_target_ && now - last_target_->t < safety_memory_) {
        const double age = now - last_target_->t;
        const Mat3 P = last_target_->P + std::pow(3.0 * age, 2) * Mat3::Identity();
        const auto r = safety_.filter(a_cmd, odom_.p, odom_.v, last_target_->p, Vec3::Zero(), P);
        a_cmd = r.a;
        safety_active = r.active;
      }
      yaw_cmd = gimbal_.yaw + follow_me::deg2rad(gim_.search_rate_deg) / gim_.gain;
      pitch_cmd = 0.0;
    }
    a_cmd = follow_me::saturate(a_cmd, fol_.a_max);

    geometry_msgs::msg::Twist sp_msg;  // linear = desired acceleration (world), angular.z = yaw
    sp_msg.linear.x = a_cmd.x();
    sp_msg.linear.y = a_cmd.y();
    sp_msg.linear.z = a_cmd.z();
    sp_msg.angular.z = yaw_sp;
    pub_sp_->publish(sp_msg);
    std_msgs::msg::Header hdr;
    hdr.stamp = stamp;
    hdr.frame_id = "world";
    pub_yaw_->publish(std_msgs::msg::Float64().set__data(yaw_cmd));
    pub_pitch_->publish(std_msgs::msg::Float64().set__data(pitch_cmd));

    if (safety_active && !was_safety_) RCLCPP_WARN(get_logger(), "Safety filter ACTIVE");
    was_safety_ = safety_active;
    if (++tick_count_ % 5 == 0) publishMarkers(hdr, p_des, has_est, safety_active);
  }

  void publishMarkers(const std_msgs::msg::Header& h, const Vec3& p_des, bool has_est,
                      bool safety_active) {
    visualization_msgs::msg::MarkerArray arr;
    Marker sp = sphere(h, "follow_point", 0, p_des, 0.35);
    sp.color.g = 0.9f;
    sp.color.a = 1.0f;
    arr.markers.push_back(sp);

    if (has_est) {
      // d_min sphere around the ESTIMATED leader (what the safety filter believes)
      Marker s = sphere(h, "d_min", 1,
                        Vec3(est_->position.x, est_->position.y, est_->position.z),
                        2.0 * safety_prm_.d_min);
      s.color.r = 1.0f;
      s.color.a = safety_active ? 0.35f : 0.1f;
      arr.markers.push_back(s);
    }
    pub_mk_->publish(arr);
  }

  static Marker sphere(const std_msgs::msg::Header& h, const std::string& ns, int id,
                       const Vec3& p, double diameter) {
    Marker m;
    m.header = h;
    m.ns = ns;
    m.id = id;
    m.type = Marker::SPHERE;
    m.action = Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = diameter;
    return m;
  }

  follow_me::FollowerParams fol_;
  follow_me::GimbalParams gim_;
  follow_me::FollowGuidance guidance_;
  follow_me::PositionController controller_;
  follow_me::SafetyParams safety_prm_;
  follow_me::SafetyFilter safety_;
  Vec3 cam_offset_;
  double estimate_timeout_, last_ = 0.0;
  long tick_count_ = 0;
  bool have_odom_ = false, was_safety_ = false;
  follow_me_ros::OdomState odom_;
  follow_me_ros::GimbalState gimbal_;
  std::optional<TargetEstimate> est_;
  std::optional<Vec3> hold_;
  std::optional<double> hold_yaw_;
  struct LastTarget {
    Vec3 p;
    Mat3 P;
    double t;
  };
  std::optional<LastTarget> last_target_;
  double safety_memory_ = 3.0;  // [s] keep protecting against the last known leader position
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<TargetEstimate>::SharedPtr sub_est_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_sp_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_, pub_pitch_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowNode>());
  rclcpp::shutdown();
  return 0;
}
