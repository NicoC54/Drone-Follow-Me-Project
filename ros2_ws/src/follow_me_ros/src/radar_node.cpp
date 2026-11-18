// radar_node: simulated GROUND surveillance radar (Gazebo has no radar sensor).
//
// Like any sensor simulator it starts from the ground truth of the objects in the scene
// (/leader/odom_truth, /follower/odom_truth), then applies the radar model of follow_me_core:
// detection probability, polar noise, Poisson clutter. Every echo is published UNLABELLED
// (target, the follower itself, clutter): telling them apart is the tracker's job.
// Scans are taken at 2 Hz and delivered after the datalink latency (0.3 s), stamped with the
// time they were TAKEN, so the tracker receives them out of sequence with the camera.
#include <deque>
#include <follow_me_msgs/msg/radar_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "follow_me_core/radar_model.hpp"
#include "follow_me_ros/ros_utils.hpp"

using follow_me::Vec3;

class RadarNode : public rclcpp::Node {
 public:
  RadarNode() : Node("radar_node"), prm_(follow_me_ros::declareRadar(*this)), sim_(prm_, declare_parameter("seed", 3)) {
    sub_l_ = create_subscription<nav_msgs::msg::Odometry>(
        "/leader/odom_truth", 5, [this](const nav_msgs::msg::Odometry& m) { leader_ = follow_me_ros::fromOdom(m); });
    sub_f_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom_truth", 5, [this](const nav_msgs::msg::Odometry& m) { follower_ = follow_me_ros::fromOdom(m); });
    pub_ = create_publisher<follow_me_msgs::msg::RadarScan>("/radar/scan", 10);
    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("/radar/markers", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Ground radar at (%.0f, %.0f, %.0f), %.1f Hz, latency %.2f s",
                prm_.position.x(), prm_.position.y(), prm_.position.z(), prm_.scan_rate, prm_.latency);
  }

 private:
  void tick() {
    const rclcpp::Time now = get_clock()->now();
    const double t = now.seconds();
    if (t <= 0.0 || !leader_ || !follower_) return;
    if (next_scan_ < 0) next_scan_ = t;
    if (t >= next_scan_) {  // take a scan
      next_scan_ += 1.0 / prm_.scan_rate;
      const auto scan = sim_.scan(t, {{leader_->p, leader_->v, true, false},
                                      {follower_->p, follower_->v, false, true}});
      follow_me_msgs::msg::RadarScan msg;
      msg.header.stamp = now;
      msg.header.frame_id = "world";
      msg.site.x = prm_.position.x();
      msg.site.y = prm_.position.y();
      msg.site.z = prm_.position.z();
      for (const auto& pl : scan.plots) {
        follow_me_msgs::msg::RadarPlot p;
        p.range = pl.range;
        p.azimuth = pl.az;
        p.elevation = pl.el;
        p.range_rate = pl.rdot;
        msg.plots.push_back(p);
      }
      link_.push_back(msg);
    }
    // Datalink: deliver the scans whose latency has elapsed.
    while (!link_.empty() && t - rclcpp::Time(link_.front().header.stamp).seconds() >= prm_.latency) {
      pub_->publish(link_.front());
      publishMarkers(link_.front());
      link_.pop_front();
    }
  }

  void publishMarkers(const follow_me_msgs::msg::RadarScan& s) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header = s.header;
    m.ns = "radar_plots";
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 2.0;
    m.color.r = 0.2f;
    m.color.g = 0.9f;
    m.color.b = 1.0f;
    m.color.a = 0.8f;
    m.lifetime = rclcpp::Duration::from_seconds(1.0);
    for (const auto& p : s.plots) {
      const auto c = follow_me::radarToCartesian(
          follow_me::RadarPlot{p.range, p.azimuth, p.elevation, p.range_rate}, prm_);
      geometry_msgs::msg::Point q;
      q.x = c.p.x();
      q.y = c.p.y();
      q.z = c.p.z();
      m.points.push_back(q);
    }
    arr.markers.push_back(m);
    visualization_msgs::msg::Marker site = m;
    site.ns = "radar_site";
    site.type = visualization_msgs::msg::Marker::CYLINDER;
    site.points.clear();
    site.pose.position.x = prm_.position.x();
    site.pose.position.y = prm_.position.y();
    site.pose.position.z = prm_.position.z() / 2;
    site.scale.x = site.scale.y = 3.0;
    site.scale.z = prm_.position.z();
    site.lifetime = rclcpp::Duration::from_seconds(0);
    arr.markers.push_back(site);
    pub_mk_->publish(arr);
  }

  follow_me::GroundRadarParams prm_;
  follow_me::GroundRadarSim sim_;
  std::optional<follow_me_ros::OdomState> leader_, follower_;
  double next_scan_ = -1.0;
  std::deque<follow_me_msgs::msg::RadarScan> link_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_l_, sub_f_;
  rclcpp::Publisher<follow_me_msgs::msg::RadarScan>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarNode>());
  rclcpp::shutdown();
  return 0;
}
