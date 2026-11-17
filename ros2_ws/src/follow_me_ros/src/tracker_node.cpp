// tracker_node: EKF + track management on the leader.
// Inputs : detections, follower odometry (camera position), gimbal joint angles (camera rotation).
// Output : /target/estimate at 100 Hz (predicted to the current time) + RViz markers.
//
// Detections are fused AT THEIR CAPTURE TIME (image stamp). The camera pose at that time is
// rebuilt from TWO stamped histories, each interpolated at the image stamp:
//   - follower odometry (position, attitude), 100 Hz
//   - gimbal joint angles, ~500 Hz
// Pairing "latest odometry" with "latest gimbal angles" broke under CPU load (GUI): queued
// messages made them tens of ms apart, i.e. >10 px of error while the gimbal turns (see docs).
#include <Eigen/Eigenvalues>
#include <follow_me_msgs/msg/detection.hpp>
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "follow_me_core/time_series.hpp"
#include "follow_me_core/timed_tracker.hpp"
#include "follow_me_ros/ros_utils.hpp"

using follow_me::Mat3;
using follow_me::TrackStatus;
using follow_me::Vec3;
using visualization_msgs::msg::Marker;

class TrackerNode : public rclcpp::Node {
 public:
  TrackerNode()
      : Node("tracker_node"),
        cam_(follow_me_ros::declareCamera(*this)),
        tracker_(follow_me_ros::declareEstimator(*this), cam_) {
    cam_offset_ = follow_me_ros::declareVec3(*this, "camera.offset", Vec3(0, 0, -0.25));
    latency_warn_ = declare_parameter("latency_warn", 0.2);  // [s]
    // Deep queues: under load, messages wait instead of being dropped; the histories are
    // stamped, so a late message is still placed at the right time.
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom", 200, [this](const nav_msgs::msg::Odometry& m) {
          const auto o = follow_me_ros::fromOdom(m);
          odom_hist_.add(o.stamp, o);
        });
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
        "/follower/joint_states", 1000, [this](const sensor_msgs::msg::JointState& m) {
          follow_me_ros::GimbalState g;
          follow_me_ros::updateGimbal(m, g);
          gimbal_hist_.add(rclcpp::Time(m.header.stamp).seconds(), g);
        });
    sub_det_ = create_subscription<follow_me_msgs::msg::Detection>(
        "/follower/detection", 10, [this](const follow_me_msgs::msg::Detection& m) { onDetection(m); });
    pub_est_ = create_publisher<follow_me_msgs::msg::TargetEstimate>("/target/estimate", 10);
    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/markers", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01),
                                  [this] { tick(); });
  }

  ~TrackerNode() override {
    std::fprintf(stderr,
                 "[tracker_node] detections used %d / received %d, injected outliers rejected %d / %d, "
                 "no camera pose %d, out of order %d, truncated (bearing only) %d, mean latency %.0f ms\n",
                 n_used_, n_det_, n_out_rej_, n_out_, n_no_pose_, tracker_.dropped(), n_bearing_only_,
                 n_det_ ? 1000.0 * sum_latency_ / n_det_ : 0.0);
  }

 private:
  // Camera pose when the image was taken: each stream interpolated at the image stamp.
  struct CameraPose {
    Vec3 pos;
    Mat3 R_cw;
    Mat3 P_pos;        // navigation position covariance at that time
    double sigma_att;  // navigation attitude std [rad]
  };
  std::optional<CameraPose> cameraPoseAt(double t) const {
    const auto odom = odom_hist_.at(t, [](const follow_me_ros::OdomState& a,
                                          const follow_me_ros::OdomState& b, double w) {
      follow_me_ros::OdomState o = a;
      o.p = (1 - w) * a.p + w * b.p;
      o.q = a.q.slerp(w, b.q);
      o.yaw = std::atan2(2.0 * (o.q.w() * o.q.z() + o.q.x() * o.q.y()),
                         1.0 - 2.0 * (o.q.y() * o.q.y() + o.q.z() * o.q.z()));
      return o;
    });
    const auto gimbal = gimbal_hist_.at(t, [](const follow_me_ros::GimbalState& a,
                                              const follow_me_ros::GimbalState& b, double w) {
      follow_me_ros::GimbalState g = a;
      g.yaw = a.yaw + w * (b.yaw - a.yaw);  // joint angle is continuous (not wrapped)
      g.pitch = a.pitch + w * (b.pitch - a.pitch);
      return g;
    });
    if (!odom || !gimbal) return std::nullopt;
    return CameraPose{follow_me_ros::cameraPosition(*odom, cam_offset_),
                      follow_me_ros::cameraRotation(*odom, *gimbal), odom->P_pos, odom->sigma_att};
  }

  void onDetection(const follow_me_msgs::msg::Detection& m) {
    if (!m.detected) return;
    const double t_capture = rclcpp::Time(m.header.stamp).seconds();
    const double latency = get_clock()->now().seconds() - t_capture;
    ++n_det_;
    sum_latency_ += latency;
    if (latency > latency_warn_)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "detection latency %.0f ms (fused at capture time anyway)", 1000 * latency);

    const auto pose = cameraPoseAt(t_capture);  // where was the camera when the image was taken?
    if (!pose) {
      ++n_no_pose_;
      return;
    }
    const auto before = tracker_.status();
    // The camera pose comes from the follower's NAVIGATION: its uncertainty is added to the
    // pixel noise, otherwise the tracker is over-confident (NEES ~44 instead of ~6 offline).
    const Mat3 R_extra =
        tracker_.cameraPoseNoise(pose->pos, pose->R_cw, pose->P_pos, pose->sigma_att);
    const bool used =
        tracker_.onMeasurement(Vec3(m.u, m.v, m.size), t_capture, pose->pos, pose->R_cw, R_extra,
                               m.size_valid);
    n_bearing_only_ += !m.size_valid;
    n_used_ += used;
    if (m.injected_outlier) {
      ++n_out_;
      n_out_rej_ += !used;
    }
    if (before != tracker_.status())
      RCLCPP_INFO(get_logger(), "track %s -> %s", follow_me::toString(before),
                  follow_me::toString(tracker_.status()));
  }

  void tick() {
    const rclcpp::Time stamp = get_clock()->now();
    const double now = stamp.seconds();
    if (now <= 0.0) return;
    const auto before = tracker_.status();
    tracker_.updateStatus(now);
    if (before != tracker_.status())
      RCLCPP_WARN(get_logger(), "track %s -> %s", follow_me::toString(before),
                  follow_me::toString(tracker_.status()));

    // The filter lives at the time of the last fused image: predict it to "now" for control.
    const follow_me::TargetEkf ekf = tracker_.predictedAt(now);
    follow_me_msgs::msg::TargetEstimate est;
    est.header.stamp = stamp;
    est.header.frame_id = "world";
    est.status = static_cast<uint8_t>(tracker_.status());
    est.position.x = ekf.x()(0);
    est.position.y = ekf.x()(1);
    est.position.z = ekf.x()(2);
    est.velocity.x = ekf.x()(3);
    est.velocity.y = ekf.x()(4);
    est.velocity.z = ekf.x()(5);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) est.covariance[6 * i + j] = ekf.P()(i, j);
    est.last_nis = tracker_.tracker().lastNis();
    pub_est_->publish(est);

    if (++tick_count_ % 5 == 0) publishMarkers(est.header, ekf);  // 20 Hz is enough for RViz
  }

  void publishMarkers(const std_msgs::msg::Header& h, const follow_me::TargetEkf& ekf) {
    visualization_msgs::msg::MarkerArray arr;
    if (!tracker_.hasEstimate()) {
      Marker del;
      del.header = h;
      del.action = Marker::DELETEALL;
      arr.markers.push_back(del);
      pub_mk_->publish(arr);
      return;
    }
    const Vec3 p = ekf.position(), v = ekf.velocity();
    const bool coast = tracker_.status() == TrackStatus::Coast;

    // 3-sigma covariance ellipsoid: axes = eigenvectors, diameters = 2 * 3 * sqrt(eigenvalues).
    Eigen::SelfAdjointEigenSolver<Mat3> es(ekf.positionCov());
    Mat3 axes = es.eigenvectors();
    if (axes.determinant() < 0) axes.col(0) *= -1.0;
    const Eigen::Quaterniond q(axes);
    Marker ell = base(h, "estimate_cov", 0, Marker::SPHERE, p);
    ell.pose.orientation.w = q.w();
    ell.pose.orientation.x = q.x();
    ell.pose.orientation.y = q.y();
    ell.pose.orientation.z = q.z();
    const Vec3 ev = es.eigenvalues().cwiseMax(1e-6).cwiseSqrt() * 6.0;
    ell.scale.x = ev.x();
    ell.scale.y = ev.y();
    ell.scale.z = ev.z();
    ell.color.r = 1.0f;
    ell.color.g = coast ? 0.85f : 0.55f;
    ell.color.a = 0.35f;
    arr.markers.push_back(ell);

    Marker c = base(h, "estimate", 1, Marker::SPHERE, p);
    c.scale.x = c.scale.y = c.scale.z = 0.3;
    c.color.r = 1.0f;
    c.color.g = 0.55f;
    c.color.a = 1.0f;
    arr.markers.push_back(c);

    Marker arrow = base(h, "estimate_vel", 2, Marker::ARROW, Vec3::Zero());
    geometry_msgs::msg::Point a, b;
    a.x = p.x(); a.y = p.y(); a.z = p.z();
    b.x = p.x() + v.x(); b.y = p.y() + v.y(); b.z = p.z() + v.z();  // 1 s ahead
    arrow.points = {a, b};
    arrow.scale.x = 0.08;
    arrow.scale.y = 0.2;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.55f;
    arrow.color.a = 1.0f;
    arr.markers.push_back(arrow);

    Marker txt = base(h, "status", 3, Marker::TEXT_VIEW_FACING, p + Vec3(0, 0, 1.5));
    txt.text = follow_me::toString(tracker_.status());
    txt.scale.z = 0.8;
    txt.color.r = txt.color.g = txt.color.b = 1.0f;
    txt.color.a = 1.0f;
    arr.markers.push_back(txt);
    pub_mk_->publish(arr);
  }

  static Marker base(const std_msgs::msg::Header& h, const std::string& ns, int id, int type,
                     const Vec3& p) {
    Marker m;
    m.header = h;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    return m;
  }

  follow_me::PinholeCamera cam_;
  follow_me::TimedTracker tracker_;
  follow_me::TimeSeries<follow_me_ros::OdomState> odom_hist_{2.0};
  follow_me::TimeSeries<follow_me_ros::GimbalState> gimbal_hist_{2.0};
  Vec3 cam_offset_;
  double latency_warn_;
  long tick_count_ = 0;
  int n_det_ = 0, n_used_ = 0, n_out_ = 0, n_out_rej_ = 0, n_no_pose_ = 0, n_bearing_only_ = 0;
  double sum_latency_ = 0.0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<follow_me_msgs::msg::Detection>::SharedPtr sub_det_;
  rclcpp::Publisher<follow_me_msgs::msg::TargetEstimate>::SharedPtr pub_est_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<TrackerNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
