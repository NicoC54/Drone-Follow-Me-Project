// leader_node: flies the leader along the scripted scenario (same as the offline sim).
//  physics = false: BODY-frame twist to Gazebo's VelocityControl (kinematic model).
//  physics = true : the leader is a quadrotor with the follower's flight-controller plugin; this
//    node is its PILOT: the scenario is a reference trajectory, tracked with
//    a = a_ref + kv (v_ref - v) + kp (p_ref - p), yaw = reference heading (world ENU accel + yaw
//    on /leader/fc/accel_yaw_setpoint), from the leader's ground truth (the target's own
//    navigation is not part of our problem). Same law as follow_me::TargetDrone offline.
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "follow_me_ros/ros_utils.hpp"

#include "follow_me_core/intercept_sim.hpp"
#include "follow_me_core/sim_models.hpp"

class LeaderNode : public rclcpp::Node {
 public:
  // scenario "follow" (phases 1-3, default) or "intercept" (phase 4: unknown drone seen by the
  // ground radar ~480 m away; the Gazebo model must start with the same pose and heading).
  static follow_me::LeaderParams scenario(const std::string& name) {
    return name == "intercept" ? follow_me::InterceptConfig().base.leader : follow_me::LeaderParams{};
  }

  LeaderNode()
      : Node("leader_node"),
        start_delay_(declare_parameter("start_delay", 5.0)),  // time for the follower to lock on
        duration_(declare_parameter("duration", 90.0)),       // then brake to a hover
        leader_(withFinalStop(scenario(declare_parameter("scenario", std::string("follow"))), duration_)) {
    const double rate = declare_parameter("rate_hz", 50.0);
    physics_ = declare_parameter("physics", false);
    pub_ = create_publisher<geometry_msgs::msg::Twist>(physics_ ? "/leader/fc/accel_yaw_setpoint" : "/leader/cmd_vel", 10);
    if (physics_)
      sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "/leader/odom_truth", 5, [this](const nav_msgs::msg::Odometry& m) { odom_ = follow_me_ros::fromOdom(m); });
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
                                  [this] { tick(); });
  }

 private:
  void tick() {
    const double now = get_clock()->now().seconds();
    if (now <= 0.0) return;  // /clock not received yet
    if (!t0_) {
      t0_ = now + start_delay_;
      last_ = now;
      RCLCPP_INFO(get_logger(), "Scenario starts in %.1f s", start_delay_);
    }
    const double dt = now - last_;
    last_ = now;
    const double t = now - *t0_;
    geometry_msgs::msg::Twist cmd;
    if (t >= 0.0 && dt > 0.0) leader_.step(t, dt);
    if (physics_) {
      if (!odom_) return;
      const follow_me::Vec3 v_ref = leader_.velocity();
      const follow_me::Vec3 a_ref = dt > 0.0 ? follow_me::Vec3((v_ref - v_ref_prev_) / dt) : follow_me::Vec3::Zero();
      v_ref_prev_ = v_ref;
      follow_me::Vec3 a = a_ref + 2.5 * (v_ref - odom_->v) + 0.6 * (leader_.position() - odom_->p);
      if (a.norm() > 6.0) a *= 6.0 / a.norm();
      cmd.linear.x = a.x();
      cmd.linear.y = a.y();
      cmd.linear.z = a.z();
      cmd.angular.z = leader_.heading();
    } else if (t >= 0.0) {
      cmd.linear.x = leader_.speed();
      cmd.linear.z = leader_.climbRate();
      cmd.angular.z = leader_.turnRate();
    }
    if (t > duration_ && !done_) {
      RCLCPP_INFO(get_logger(), "Scenario finished, leader brakes to a hover");
      done_ = true;
    }
    pub_->publish(cmd);
  }

  // The end of the flight plan is a STOP segment, flown with the leader's acceleration limit.
  // (Publishing a zero twist stopped the Gazebo model instantly from 5 m/s: an impossible
  // deceleration that the follower could not match, 4.3 m minimum distance at t = 123 s.)
  static follow_me::LeaderParams withFinalStop(follow_me::LeaderParams p, double duration) {
    p.scenario.push_back({duration, 0.0, 0.0, 0.0});
    return p;
  }

  double start_delay_, duration_, last_ = 0.0;
  follow_me::Leader leader_;
  bool physics_ = false;
  std::optional<follow_me_ros::OdomState> odom_;
  follow_me::Vec3 v_ref_prev_ = follow_me::Vec3::Zero();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::optional<double> t0_;
  bool done_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderNode>());
  rclcpp::shutdown();
  return 0;
}
