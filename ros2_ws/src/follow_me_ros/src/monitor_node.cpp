// monitor_node: the only node allowed to read the ground truth (/leader/odom_truth).
// Computes the same metrics as the offline sim, publishes them for rqt_plot, draws the
// true trajectories in RViz, and prints a summary on shutdown (Ctrl+C).
#include <deque>
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/float32_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_ros/ros_utils.hpp"

using follow_me::Vec3;
using follow_me_msgs::msg::TargetEstimate;
using visualization_msgs::msg::Marker;

class MonitorNode : public rclcpp::Node {
 public:
  MonitorNode() : Node("monitor_node"), cam_(follow_me_ros::declareCamera(*this)) {
    cam_offset_ = follow_me_ros::declareVec3(*this, "camera.offset", Vec3(0, 0, -0.25));
    const auto g = follow_me_ros::declareGuidance(*this);
    d_ref_ = std::hypot(g.follow_dist, g.height_offset);
    d_min_ = follow_me_ros::declareSafety(*this).d_min;
    settle_time_ = declare_parameter("settle_time", 10.0);
    trail_len_ = declare_parameter("trail_seconds", 20.0) * 10;

    sub_l_ = create_subscription<nav_msgs::msg::Odometry>(
        "/leader/odom_truth", 1, [this](const nav_msgs::msg::Odometry& m) { leader_ = follow_me_ros::fromOdom(m); });
    // Truth of the follower (for distances, in-frame check) and its navigation estimate
    // (to measure the navigation error).
    sub_f_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom_truth", 1, [this](const nav_msgs::msg::Odometry& m) { follower_ = follow_me_ros::fromOdom(m); });
    sub_nav_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom", 1, [this](const nav_msgs::msg::Odometry& m) { nav_ = follow_me_ros::fromOdom(m); });
    sub_fc_ = create_subscription<ros_gz_interfaces::msg::Float32Array>(
        "/follower/fc/status", 10, [this](const ros_gz_interfaces::msg::Float32Array& m) {
          if (m.data.size() < 3) return;
          max_tilt_ = std::max(max_tilt_, static_cast<double>(m.data[0]));
          ++n_fc_;
          n_sat_ += m.data[2] > 0.5f;
        });
    pub_nav_err_ = create_publisher<std_msgs::msg::Float64>("/monitor/navigation_error", 10);
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
        "/follower/joint_states", 1,
        [this](const sensor_msgs::msg::JointState& m) { follow_me_ros::updateGimbal(m, gimbal_); });
    // Phase 4: mission phase, to time the interception and measure the follow quality only in FOLLOW.
    sub_phase_ = create_subscription<std_msgs::msg::String>("/mission/phase", 10, [this](const std_msgs::msg::String& m) {
      if (m.data != phase_) {
        if (t_start_) phase_log_ += "  " + m.data + "@" + std::to_string(static_cast<int>(get_clock()->now().seconds() - *t_start_)) + "s";
        if (m.data == "FOLLOW") t_follow_start_ = get_clock()->now().seconds();
        phase_ = m.data;
      }
    });
    sub_est_ = create_subscription<TargetEstimate>("/target/estimate", 10,
                                                   [this](const TargetEstimate& m) { est_ = m; });
    pub_dist_ = create_publisher<std_msgs::msg::Float64>("/monitor/distance", 10);
    pub_err_ = create_publisher<std_msgs::msg::Float64>("/monitor/estimation_error", 10);
    pub_nees_ = create_publisher<std_msgs::msg::Float64>("/monitor/nees", 10);
    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("/monitor/markers", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1),
                                  [this] { tick(); });
  }

  // Printed from the shutdown callback (Ctrl+C), on stderr so it is not lost in a buffer.
  void printSummary() const {
    if (!phase_log_.empty()) std::fprintf(stderr, "\n  mission phases:%s\n", phase_log_.c_str());
    const double n = std::max(n_steady_, 1);
    std::fprintf(stderr, "\n===== follow-me summary (%.0f s monitored) =====\n", n_total_ * 0.1);
    std::fprintf(stderr, "  estimation position RMSE [m]     %8.2f\n", std::sqrt(s_ep2_ / n));
    std::fprintf(stderr, "  estimation velocity RMSE [m/s]   %8.2f\n", std::sqrt(s_ev2_ / n));
    std::fprintf(stderr, "  mean NEES (ideal 6)              %8.2f\n", s_nees_ / n);
    std::fprintf(stderr, "  distance error RMS [m]           %8.2f\n", std::sqrt(s_derr2_ / n));
    std::fprintf(stderr, "  min distance [m]                 %8.2f\n", min_dist_);
    std::fprintf(stderr, "  time below d_min [s]             %8.2f\n", t_below_);
    std::fprintf(stderr, "  time leader in frame [%%]         %8.2f\n", 100.0 * n_frame_ / std::max(n_total_, 1));
    std::fprintf(stderr, "  time with valid track [%%]        %8.2f\n", 100.0 * n_valid_ / std::max(n_total_, 1));
    std::fprintf(stderr, "  navigation position RMSE [m]     %8.2f\n", std::sqrt(s_nav2_ / std::max(n_nav_, 1)));
    std::fprintf(stderr, "  navigation attitude RMSE [deg]   %8.2f\n",
                 follow_me::rad2deg(std::sqrt(s_att2_ / std::max(n_nav_, 1))));
    std::fprintf(stderr, "  max tilt [deg]                   %8.1f\n", max_tilt_);
    std::fprintf(stderr, "  mixer saturated [%%]              %8.2f\n", 100.0 * n_sat_ / std::max(n_fc_, 1));
    std::fflush(stderr);
  }

 private:
  void tick() {
    if (!leader_ || !follower_ || !gimbal_.valid) return;
    const double now = get_clock()->now().seconds();
    if (!t_start_) t_start_ = now;
    const double t = now - *t_start_;
    ++n_total_;

    const double dist = (follower_->p - leader_->p).norm();
    if (nav_) {  // navigation error (estimate vs truth)
      const double e = (nav_->p - follower_->p).norm();
      Eigen::Quaterniond qe = nav_->q.conjugate() * follower_->q;
      if (qe.w() < 0) qe.coeffs() *= -1.0;
      s_nav2_ += e * e;
      s_att2_ += (2.0 * qe.vec()).squaredNorm();
      ++n_nav_;
      pub_nav_err_->publish(std_msgs::msg::Float64().set__data(e));
    }
    min_dist_ = std::min(min_dist_, dist);
    if (dist < d_min_) t_below_ += 0.1;
    pub_dist_->publish(std_msgs::msg::Float64().set__data(dist));

    const Vec3 cam_pos = follow_me_ros::cameraPosition(*follower_, cam_offset_);
    const Vec3 rc = follow_me::PinholeCamera::toCamera(
        leader_->p, cam_pos, follow_me_ros::cameraRotation(*follower_, gimbal_));
    if (rc.z() > 0 && cam_.visible(cam_.project(rc), rc)) ++n_frame_;

    const bool valid = est_ && est_->status != TargetEstimate::SEARCH;
    if (valid) {
      ++n_valid_;
      follow_me::Vec6 e;
      e << est_->position.x - leader_->p.x(), est_->position.y - leader_->p.y(),
          est_->position.z - leader_->p.z(), est_->velocity.x - leader_->v.x(),
          est_->velocity.y - leader_->v.y(), est_->velocity.z - leader_->v.z();
      follow_me::Mat6 P;
      for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) P(i, j) = est_->covariance[6 * i + j];
      const double nees = e.dot(P.ldlt().solve(e));
      pub_err_->publish(std_msgs::msg::Float64().set__data(e.head<3>().norm()));
      pub_nees_->publish(std_msgs::msg::Float64().set__data(nees));
      // Intercept mode: follow quality only counts in FOLLOW (after settling); otherwise after settle_time.
      const bool steady = phase_.empty() ? t >= settle_time_
                                         : (phase_ == "FOLLOW" && t_follow_start_ > 0 &&
                                            get_clock()->now().seconds() - t_follow_start_ > 5.0);
      if (steady) {
        ++n_steady_;
        s_ep2_ += e.head<3>().squaredNorm();
        s_ev2_ += e.tail<3>().squaredNorm();
        s_nees_ += nees;
        s_derr2_ += (dist - d_ref_) * (dist - d_ref_);
      }
    }
    if (n_total_ % 50 == 0) {  // every 5 s
      const char* st = !est_ ? "-" : est_->status == 1 ? "TRACK" : est_->status == 2 ? "COAST" : "SEARCH";
      const double err = valid ? Vec3(est_->position.x - leader_->p.x(), est_->position.y - leader_->p.y(),
                                      est_->position.z - leader_->p.z()).norm()
                               : NAN;
      RCLCPP_INFO(get_logger(), "t=%5.1fs  distance=%5.2f m  est.error=%5.2f m  track=%s", t, dist, err, st);
    }

    trail_l_.push_back(leader_->p);
    trail_f_.push_back(follower_->p);
    while (static_cast<int>(trail_l_.size()) > trail_len_) trail_l_.pop_front();
    while (static_cast<int>(trail_f_.size()) > trail_len_) trail_f_.pop_front();
    publishMarkers(dist);
  }

  void publishMarkers(double dist) {
    std_msgs::msg::Header h;
    h.stamp = get_clock()->now();
    h.frame_id = "world";
    visualization_msgs::msg::MarkerArray arr;
    auto line = [&](const std::string& ns, const std::deque<Vec3>& pts, float r, float g, float b) {
      Marker m;
      m.header = h;
      m.ns = ns;
      m.type = Marker::LINE_STRIP;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.1;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
      for (const auto& p : pts) {
        geometry_msgs::msg::Point q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        m.points.push_back(q);
      }
      return m;
    };
    arr.markers.push_back(line("leader_truth_trail", trail_l_, 0.9f, 0.1f, 0.1f));
    arr.markers.push_back(line("follower_trail", trail_f_, 0.1f, 0.3f, 0.9f));
    // Link between the drones: green near the setpoint, red close to d_min.
    const double k = std::clamp((dist - d_min_) / (d_ref_ - d_min_), 0.0, 1.0);
    arr.markers.push_back(line("link", {leader_->p, follower_->p}, 1.0f - k, k, 0.1f));
    // Drones drawn IN their TF frame (bridged from Gazebo), frame-locked: they move with every TF
    // update, tilt and turn with the drone. A light X-frame silhouette at the real Gazebo size
    // (two arms + four translucent rotor discs) instead of an opaque box hiding the TF axes.
    auto drone = [&](const std::string& ns, const std::string& frame, double rotor_xy, float r, float g,
                     float b) {
      Marker base;
      base.header.frame_id = frame;
      base.header.stamp = rclcpp::Time(0);  // latest available transform
      base.frame_locked = true;
      base.ns = ns;
      base.pose.orientation.w = 1.0;
      base.color.r = r; base.color.g = g; base.color.b = b;

      Marker arms = base;  // the X frame
      arms.id = 0;
      arms.type = Marker::LINE_LIST;
      arms.scale.x = 0.04;
      arms.color.a = 0.9f;
      for (const double sx : {1.0, -1.0}) {
        geometry_msgs::msg::Point a, c;
        a.x = sx * rotor_xy; a.y = rotor_xy;
        c.x = -sx * rotor_xy; c.y = -rotor_xy;
        arms.points.push_back(a);
        arms.points.push_back(c);
      }
      arr.markers.push_back(arms);

      int id = 1;  // the four rotor discs
      for (const double sx : {1.0, -1.0})
        for (const double sy : {1.0, -1.0}) {
          Marker rotor = base;
          rotor.id = id++;
          rotor.type = Marker::CYLINDER;
          rotor.pose.position.x = sx * rotor_xy;
          rotor.pose.position.y = sy * rotor_xy;
          rotor.pose.position.z = 0.03;
          rotor.scale.x = rotor.scale.y = 0.28;
          rotor.scale.z = 0.01;
          rotor.color.a = 0.45f;
          arr.markers.push_back(rotor);
        }
    };
    drone("leader_truth", "leader", 0.42, 0.9f, 0.1f, 0.1f);           // = leader model in the SDF
    drone("follower", "follower_truth", 0.37, 0.1f, 0.3f, 0.9f);       // = follower model in the SDF
    pub_mk_->publish(arr);
  }

  follow_me::PinholeCamera cam_;
  Vec3 cam_offset_;
  double d_ref_, d_min_, settle_time_;
  int trail_len_;
  std::optional<follow_me_ros::OdomState> leader_, follower_, nav_;
  follow_me_ros::GimbalState gimbal_;
  std::optional<TargetEstimate> est_;
  std::optional<double> t_start_;
  std::string phase_, phase_log_;
  double t_follow_start_ = -1.0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_phase_;
  std::deque<Vec3> trail_l_, trail_f_;
  int n_total_ = 0, n_valid_ = 0, n_frame_ = 0, n_steady_ = 0;
  double s_ep2_ = 0, s_ev2_ = 0, s_nees_ = 0, s_derr2_ = 0, min_dist_ = 1e9, t_below_ = 0;
  double s_nav2_ = 0, s_att2_ = 0, max_tilt_ = 0;
  int n_nav_ = 0, n_fc_ = 0, n_sat_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_l_, sub_f_, sub_nav_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Float32Array>::SharedPtr sub_fc_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<TargetEstimate>::SharedPtr sub_est_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dist_, pub_err_, pub_nees_, pub_nav_err_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MonitorNode>();
  rclcpp::on_shutdown([node] { node->printSummary(); });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
