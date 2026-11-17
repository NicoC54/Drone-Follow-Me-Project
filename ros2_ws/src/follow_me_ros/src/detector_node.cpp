// detector_node: finds the leader's red beacon in the REAL rendered camera image (OpenCV).
// Measurement z = [u, v, size] = centre and diameter of the enclosing circle of red pixels.
// Optional fault injection (missed / false detections) to exercise the tracker's robustness.
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <deque>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "follow_me_msgs/msg/detection.hpp"

class DetectorNode : public rclcpp::Node {
 public:
  DetectorNode() : Node("detector_node") {
    p_miss_ = declare_parameter("inject.p_miss", 0.0);
    p_outlier_ = declare_parameter("inject.p_outlier", 0.0);
    // Simulated processing pipeline latency: detections are released `latency_ms` (sim time)
    // after their image, without lowering the detection rate.
    latency_ms_ = declare_parameter("inject.latency_ms", 0);
    rng_.seed(declare_parameter("seed", 1));
    min_pixels_ = declare_parameter("min_pixels", 3);
    sat_min_ = declare_parameter("hsv.sat_min", 120);
    val_min_ = declare_parameter("hsv.val_min", 60);
    hue_max_ = declare_parameter("hsv.hue_max", 10);
    size_bias_px_ = declare_parameter("size_bias_px", 0.0);  // detector calibration [px]
    fill_min_ = declare_parameter("fill_min", 0.75);  // red pixels / circle area below this: truncated

    pub_det_ = create_publisher<follow_me_msgs::msg::Detection>("/follower/detection", 10);
    pub_dbg_ = create_publisher<sensor_msgs::msg::Image>("/follower/detector/debug_image", 5);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        // Queue of ONE image: if processing lags, old frames are dropped instead of piling up
        // (a backlog adds latency without adding information).
        "/follower/camera/image", rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { onImage(m); });
    release_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.005),
                                          [this] { releaseDelayed(); });
  }

 private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImageConstPtr cv;
    try {
      cv = cv_bridge::toCvShare(msg, "rgb8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }
    const cv::Mat& rgb = cv->image;

    // 1. Colour segmentation in HSV (red wraps around hue 0).
    cv::Mat hsv, m1, m2, mask;
    cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, cv::Scalar(0, sat_min_, val_min_), cv::Scalar(hue_max_, 255, 255), m1);
    cv::inRange(hsv, cv::Scalar(180 - hue_max_, sat_min_, val_min_), cv::Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);

    follow_me_msgs::msg::Detection det;
    det.header = msg->header;
    cv::Point2f c;
    float r = 0.f;
    // 2. Enclosing circle of all red pixels: robust to the thin arms crossing the beacon.
    if (static_cast<int>(pts.size()) >= min_pixels_) {
      cv::minEnclosingCircle(pts, c, r);
      det.detected = true;
      // OpenCV puts pixel centres on integers; the pinhole model (cx = width/2) uses continuous
      // coordinates where pixel i spans [i, i+1): shift by half a pixel.
      det.u = c.x + 0.5;
      det.v = c.y + 0.5;
      // Calibrated against the ground truth (see docs): the raw diameter is biased by
      // anti-aliased edge pixels; size_bias_px removes it.
      det.size = 2.0 * r - size_bias_px_;
      // A truncated beacon (behind the follower's own airframe, cut by the image border...)
      // gives a too SMALL enclosing circle -> range overestimated. Detect it: a full disc fills
      // its enclosing circle, a cut one does not.
      const double fill = pts.size() / (M_PI * std::max(r, 0.5f) * std::max(r, 0.5f));
      const bool on_border = c.x - r < 1 || c.y - r < 1 || c.x + r > rgb.cols - 2 || c.y + r > rgb.rows - 2;
      det.size_valid = !on_border && (r < 3.0f || fill >= fill_min_);
    }
    // 3. Fault injection.
    if (det.detected && uni_(rng_) < p_miss_) det.detected = false;
    if (uni_(rng_) < p_outlier_) {
      det.detected = true;
      det.injected_outlier = true;
      det.size_valid = true;
      det.u = uni_(rng_) * rgb.cols;
      det.v = uni_(rng_) * rgb.rows;
      det.size = 3.0 + uni_(rng_) * 40.0;
    }
    if (latency_ms_ > 0) {
      delayed_.push_back(det);  // released by the timer
    } else {
      pub_det_->publish(det);
    }

    // 4. Debug image: what the detector saw.
    if (pub_dbg_->get_subscription_count() > 0 || pub_dbg_->get_intra_process_subscription_count() > 0) {
      cv::Mat dbg = rgb.clone();
      if (det.detected) {
        const cv::Scalar col = det.injected_outlier ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0);
        cv::circle(dbg, {static_cast<int>(det.u), static_cast<int>(det.v)},
                   std::max(4, static_cast<int>(det.size / 2 + 4)), col, 1);
        char txt[96];
        std::snprintf(txt, sizeof(txt), "%s u=%.0f v=%.0f s=%.1fpx%s",
                      det.injected_outlier ? "OUTLIER" : "det", det.u, det.v, det.size,
                      det.size_valid ? "" : " TRUNCATED (bearing only)");
        cv::putText(dbg, txt, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
      } else {
        cv::putText(dbg, "no detection", {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 0}, 1);
      }
      cv::drawMarker(dbg, {rgb.cols / 2, rgb.rows / 2}, {255, 255, 255}, cv::MARKER_CROSS, 12, 1);
      pub_dbg_->publish(*cv_bridge::CvImage(msg->header, "rgb8", dbg).toImageMsg());
    }
  }

  void releaseDelayed() {
    const rclcpp::Time now = get_clock()->now();
    while (!delayed_.empty() &&
           now - rclcpp::Time(delayed_.front().header.stamp) >=
               rclcpp::Duration::from_seconds(latency_ms_ / 1000.0)) {
      pub_det_->publish(delayed_.front());
      delayed_.pop_front();
    }
  }

  std::deque<follow_me_msgs::msg::Detection> delayed_;
  rclcpp::TimerBase::SharedPtr release_timer_;
  double p_miss_, p_outlier_, size_bias_px_, fill_min_;
  int min_pixels_, sat_min_, val_min_, hue_max_, latency_ms_;
  std::mt19937_64 rng_;
  std::uniform_real_distribution<double> uni_{0.0, 1.0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<follow_me_msgs::msg::Detection>::SharedPtr pub_det_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dbg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
