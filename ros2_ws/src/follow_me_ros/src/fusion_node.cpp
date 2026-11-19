// fusion_node (phase 4): radar + camera target tracker (follow_me::FusionTracker).
//
// Inputs : /radar/scan (ground radar, 0.3 s late, unlabelled echoes incl. our own),
//          /follower/detection (camera: bearing + apparent size of an unknown drone),
//          /follower/odom (NAVIGATION estimate + covariance), /follower/joint_states (gimbal).
// Outputs: /target/estimate (100 Hz, predicted to now, camera lock flag, learnt size),
//          /follower/detector/roi (where the detector must look in the next image: "cueing"),
//          /tracker/markers.
// Every measurement is fused at its CAPTURE time; late ones are handled by an exact replay.
#include <follow_me_msgs/msg/detection.hpp>
#include <follow_me_msgs/msg/radar_scan.hpp>
#include <follow_me_msgs/msg/roi.hpp>
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "follow_me_core/fusion_tracker.hpp"
#include "follow_me_core/time_series.hpp"
#include "follow_me_ros/ros_utils.hpp"

using follow_me::Mat3;
using follow_me::TrackStatus;
using follow_me::Vec3;
using visualization_msgs::msg::Marker;

class FusionNode : public rclcpp::Node {
 public:
  FusionNode()
      : Node("fusion_node"),
        cam_prm_(follow_me_ros::declareCamera(*this)),
        radar_prm_(follow_me_ros::declareRadar(*this)),
        tracker_(follow_me_ros::declareFusion(*this), cam_prm_, radar_prm_) {
    cam_offset_ = follow_me_ros::declareVec3(*this, "camera.offset", Vec3(0, 0, -0.25));
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
        "/follower/detection", 20, [this](const follow_me_msgs::msg::Detection& m) { onDetection(m); });
    sub_radar_ = create_subscription<follow_me_msgs::msg::RadarScan>(
        "/radar/scan", 10, [this](const follow_me_msgs::msg::RadarScan& m) { onRadar(m); });
    pub_est_ = create_publisher<follow_me_msgs::msg::TargetEstimate>("/target/estimate", 10);
    pub_roi_ = create_publisher<follow_me_msgs::msg::Roi>("/follower/detector/roi", 5);
    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/markers", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), [this] { tick(); });
  }

  ~FusionNode() override {
    const auto s = tracker_.stats();
    std::fprintf(stderr,
                 "[fusion_node] radar: target %d, own echo %d, clutter %d | camera: used %d, rejected %d | "
                 "OOSM replays %d | size %.3f +/- %.3f m\n",
                 s.radar_used, s.radar_own, s.radar_clutter, s.cam_used, s.cam_rejected, s.replays,
                 tracker_.sizeEstimate(), tracker_.sizeSigma());
  }

 private:
  struct CameraPose {
    Vec3 pos;
    Mat3 R_cw;
    follow_me_ros::OdomState odom;
    follow_me_ros::GimbalState gimbal;
  };
  // Camera pose at time t: odometry and gimbal histories, each interpolated at t.
  std::optional<CameraPose> cameraPoseAt(double t) const {
    const auto odom = odom_hist_.at(t, [](const follow_me_ros::OdomState& a, const follow_me_ros::OdomState& b, double w) {
      follow_me_ros::OdomState o = a;
      o.p = (1 - w) * a.p + w * b.p;
      o.q = a.q.slerp(w, b.q);
      return o;
    });
    const auto gimbal = gimbal_hist_.at(t, [](const follow_me_ros::GimbalState& a, const follow_me_ros::GimbalState& b, double w) {
      follow_me_ros::GimbalState g = a;
      g.yaw = a.yaw + w * (b.yaw - a.yaw);
      g.pitch = a.pitch + w * (b.pitch - a.pitch);
      return g;
    });
    if (!odom || !gimbal) return std::nullopt;
    return CameraPose{follow_me_ros::cameraPosition(*odom, cam_offset_),
                      follow_me_ros::cameraRotation(*odom, *gimbal), *odom, *gimbal};
  }

  void onDetection(const follow_me_msgs::msg::Detection& m) {
    if (!m.detected) return;
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const auto pose = cameraPoseAt(t);
    if (!pose) return;
    follow_me::FusionTracker::CameraObs obs;
    obs.z = Vec3(m.u, m.v, m.size);
    obs.cam_pos = pose->pos;
    obs.R_cw = pose->R_cw;
    obs.R_extra = tracker_.cameraPoseNoise(pose->pos, pose->R_cw, pose->odom.P_pos, pose->odom.sigma_att);
    obs.size_valid = m.size_valid;
    const bool was_locked = tracker_.cameraLocked(t);
    tracker_.onCamera(t, obs);
    if (!was_locked && tracker_.cameraLocked(t))
      RCLCPP_INFO(get_logger(), "camera LOCKED on the target (size %.2f +/- %.2f m)",
                  tracker_.sizeEstimate(), tracker_.sizeSigma());
  }

  void onRadar(const follow_me_msgs::msg::RadarScan& m) {
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const auto odom = odom_hist_.at(t, [](const follow_me_ros::OdomState& a, const follow_me_ros::OdomState& b, double w) {
      follow_me_ros::OdomState o = a;
      o.p = (1 - w) * a.p + w * b.p;
      return o;
    });
    if (!odom) return;  // scan older than our navigation history
    follow_me::FusionTracker::RadarObs obs;
    obs.own_pos = odom->p;  // where WE were at the scan time: to recognise our own echo
    obs.own_P = odom->P_pos + 0.01 * Mat3::Identity();
    obs.site = Vec3(m.site.x, m.site.y, m.site.z);  // several radars: the site of THIS scan
    for (const auto& p : m.plots) obs.plots.push_back({p.range, p.azimuth, p.elevation, p.range_rate});
    const auto before = tracker_.status();
    tracker_.onRadar(t, obs);
    if (before != tracker_.status())
      RCLCPP_INFO(get_logger(), "track %s -> %s (radar)", follow_me::toString(before),
                  follow_me::toString(tracker_.status()));
  }

  void tick() {
    const rclcpp::Time stamp = get_clock()->now();
    const double now = stamp.seconds();
    if (now <= 0.0) return;
    const auto before = tracker_.status();
    tracker_.updateStatus(now);
    if (before != tracker_.status())
      RCLCPP_WARN(get_logger(), "track %s -> %s", follow_me::toString(before), follow_me::toString(tracker_.status()));

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
    est.camera_locked = tracker_.cameraLocked(now);
    est.target_size = tracker_.sizeEstimate();
    est.target_size_sigma = tracker_.sizeSigma();
    pub_est_->publish(est);

    publishRoi(stamp, ekf);
    if (++tick_count_ % 5 == 0) publishMarkers(est.header, ekf, est.camera_locked);
  }

  // Cueing: project the predicted target into the current image; the window covers the
  // 3-sigma position uncertainty plus the target's apparent size.
  void publishRoi(const rclcpp::Time& stamp, const follow_me::TargetEkf& ekf) {
    follow_me_msgs::msg::Roi roi;
    roi.header.stamp = stamp;
    roi.valid = false;
    const auto pose = cameraPoseAt(stamp.seconds() - 0.01);
    if (tracker_.hasEstimate() && pose) {
      follow_me::CameraParams cp = cam_prm_;
      cp.target_size = tracker_.sizeEstimate();
      const follow_me::PinholeCamera cam(cp);
      const Vec3 rc = follow_me::PinholeCamera::toCamera(ekf.position(), pose->pos, pose->R_cw);
      if (rc.z() > 1.0) {
        const Vec3 z = cam.project(rc);
        const Mat3 H = cam.jacobianPos(rc, pose->R_cw);
        const Mat3 S = H * ekf.positionCov() * H.transpose();
        const double sigma_px = std::sqrt(std::max(S(0, 0), S(1, 1)));
        roi.valid = true;
        roi.u = z(0);
        roi.v = z(1);
        roi.half_size = std::clamp(3.0 * sigma_px + 1.5 * z(2) + 15.0, 25.0, 200.0);
      }
    }
    pub_roi_->publish(roi);
  }

  void publishMarkers(const std_msgs::msg::Header& h, const follow_me::TargetEkf& ekf, bool locked) {
    visualization_msgs::msg::MarkerArray arr;
    if (!tracker_.hasEstimate()) {
      Marker del;
      del.header = h;
      del.action = Marker::DELETEALL;
      arr.markers.push_back(del);
      pub_mk_->publish(arr);
      return;
    }
    const Vec3 p = ekf.position();
    Marker ell = follow_me_ros::ellipsoidMarker(p, ekf.positionCov(), 3.0);
    ell.header = h;
    ell.ns = "estimate_cov";
    ell.color.r = locked ? 0.2f : 1.0f;  // green when the camera is locked, orange on radar only
    ell.color.g = locked ? 1.0f : 0.55f;
    ell.color.a = 0.35f;
    arr.markers.push_back(ell);
    Marker txt = ell;
    txt.ns = "status";
    txt.type = Marker::TEXT_VIEW_FACING;
    txt.pose.position.z += 2.3;
    txt.pose.orientation = geometry_msgs::msg::Quaternion();
    txt.scale.x = txt.scale.y = 0;
    txt.scale.z = 0.65;
    txt.color = ell.color;  // green when the camera is locked, orange on radar only
    txt.color.a = 1.0f;
    char buf[96];
    if (tracker_.sizeSigma() < 0.25)
      std::snprintf(buf, sizeof(buf), "ESTIMATE %s\nsize %.2f+/-%.2f m", locked ? "radar+camera" : "radar-only",
                    tracker_.sizeEstimate(), 2.0 * tracker_.sizeSigma());
    else
      std::snprintf(buf, sizeof(buf), "ESTIMATE %s\nsize unknown", locked ? "radar+camera" : "radar-only");
    txt.text = buf;
    arr.markers.push_back(txt);
    if (const auto pose = cameraPoseAt(rclcpp::Time(h.stamp).seconds() - 0.01))
      arr.markers.push_back(frustumMarker(h, *pose, (p - pose->pos).norm(), locked));
    pub_mk_->publish(arr);
  }

  // Camera field of view: a pyramid from the camera centre to the image corners, cut at the
  // target range (what the detector can see; the ROI is the small window inside it).
  // Camera field of view: a pyramid from the camera centre to the image corners, cut at the
  // target range. Drawn in the follower's TF frame (frame-locked): it moves with the drone at the
  // display rate. (Drawn in the world frame at 20 Hz from the navigation pose, it lagged the
  // smoothly moving drone by up to 0.6 m at 12 m/s, then jumped: it looked like it was shaking.)
  // Only the gimbal angles are taken from here; the body attitude comes from the TF.
  Marker frustumMarker(const std_msgs::msg::Header& h, const CameraPose& pose, double range, bool locked) const {
    Marker m;
    m.header.frame_id = "follower_truth";
    m.header.stamp = rclcpp::Time(0);  // latest transform
    m.frame_locked = true;
    m.ns = "camera_fov";
    m.type = Marker::LINE_LIST;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;
    m.color = locked ? follow_me_ros::rgba(0.11f, 0.69f, 0.48f, 0.5f) : follow_me_ros::rgba(0.93f, 0.63f, 0.0f, 0.5f);
    (void)h;
    const double d = std::clamp(range, 3.0, 60.0);
    // Body -> camera link: yaw about z, then pitch about -y (positive = looks up), as in cameraRotation.
    const Mat3 R_bl = (Eigen::AngleAxisd(pose.gimbal.yaw, Vec3::UnitZ()) *
                       Eigen::AngleAxisd(pose.gimbal.pitch, -Vec3::UnitY())).toRotationMatrix();
    Vec3 c[4];
    const double us[4] = {0, double(cam_prm_.width), double(cam_prm_.width), 0};
    const double vs[4] = {0, 0, double(cam_prm_.height), double(cam_prm_.height)};
    for (int k = 0; k < 4; ++k) {
      const double x = (us[k] - 0.5 * cam_prm_.width) / cam_prm_.fx, y = (vs[k] - 0.5 * cam_prm_.height) / cam_prm_.fy;
      const Vec3 dir_link(1.0, -x, -y);  // camera (right, down, forward) -> link (forward, left, up)
      c[k] = cam_offset_ + d * (R_bl * dir_link);
    }
    for (int k = 0; k < 4; ++k) {
      m.points.push_back(follow_me_ros::toPoint(cam_offset_));
      m.points.push_back(follow_me_ros::toPoint(c[k]));
      m.points.push_back(follow_me_ros::toPoint(c[k]));
      m.points.push_back(follow_me_ros::toPoint(c[(k + 1) % 4]));
    }
    return m;
  }

  follow_me::CameraParams cam_prm_;
  follow_me::GroundRadarParams radar_prm_;
  follow_me::FusionTracker tracker_;
  follow_me::TimeSeries<follow_me_ros::OdomState> odom_hist_{4.0};
  follow_me::TimeSeries<follow_me_ros::GimbalState> gimbal_hist_{4.0};
  Vec3 cam_offset_;
  long tick_count_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<follow_me_msgs::msg::Detection>::SharedPtr sub_det_;
  rclcpp::Subscription<follow_me_msgs::msg::RadarScan>::SharedPtr sub_radar_;
  rclcpp::Publisher<follow_me_msgs::msg::TargetEstimate>::SharedPtr pub_est_;
  rclcpp::Publisher<follow_me_msgs::msg::Roi>::SharedPtr pub_roi_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
