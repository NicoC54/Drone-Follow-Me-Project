// leader_node: flies the leader along the scripted scenario (same as the offline sim).
// Publishes a BODY-frame twist to Gazebo's VelocityControl: forward speed, climb rate, turn rate.
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "follow_me_core/sim_models.hpp"

class LeaderNode : public rclcpp::Node {
 public:
  LeaderNode() : Node("leader_node"), leader_(follow_me::LeaderParams{}) {
    start_delay_ = declare_parameter("start_delay", 5.0);  // time for the follower to lock on
    duration_ = declare_parameter("duration", 90.0);       // then hover
    const double rate = declare_parameter("rate_hz", 50.0);
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/leader/cmd_vel", 10);
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
    if (t >= 0.0 && t <= duration_) {
      if (dt > 0.0) leader_.step(t, dt);
      cmd.linear.x = leader_.speed();
      cmd.linear.z = leader_.climbRate();
      cmd.angular.z = leader_.turnRate();
    } else if (t > duration_ && !done_) {
      RCLCPP_INFO(get_logger(), "Scenario finished, leader hovers");
      done_ = true;
    }
    pub_->publish(cmd);
  }

  follow_me::Leader leader_;
  double start_delay_, duration_, last_ = 0.0;
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
