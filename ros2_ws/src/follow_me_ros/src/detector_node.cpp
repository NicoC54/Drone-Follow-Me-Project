// detector_node: finds the leader in the REAL rendered camera image (OpenCV). Two modes:
//  - "color"      (phases 2-3): red beacon of known size; z = [u, v, size] = centre and
//                 diameter of the enclosing circle of red pixels.
//  - "silhouette" (phase 4): unknown drone, no beacon. The tracker sends a region of interest
//                 (/follower/detector/roi, "cueing" by the radar track); inside it, the drone is
//                 a DARK blob against the bright sky. z = [u, v, size] = bounding-box centre and
//                 largest side (same convention as the size W learnt by the tracker).
// Optional fault injection (missed / false detections, latency) to exercise the tracker.
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <deque>
#include <optional>
#include <string>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "follow_me_msgs/msg/detection.hpp"
#include "follow_me_msgs/msg/roi.hpp"

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
    mode_ = declare_parameter("mode", std::string("color"));
    dark_ratio_ = declare_parameter("silhouette.dark_ratio", 0.65);  // darker than 65 % of the sky
    roi_timeout_ = declare_parameter("silhouette.roi_timeout", 0.5);
    size_mode_ = declare_parameter("silhouette.size_mode", std::string("bbox"));  // bbox | moment
    // Raw detection text on the debug image (u, v, size). Off by default: hud_node shows it better.
    debug_text_ = declare_parameter("debug_text", false);
    sub_roi_ = create_subscription<follow_me_msgs::msg::Roi>(
        "/follower/detector/roi", 5, [this](const follow_me_msgs::msg::Roi& r) { roi_ = r; });

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
    follow_me_msgs::msg::Detection det;
    det.header = msg->header;
    cv::Rect roi_rect;
    if (mode_ == "silhouette") {
      roi_rect = detectSilhouette(rgb, msg->header.stamp, det);
    } else {
      detectColor(rgb, det);
    }
    finish(rgb, msg, det, roi_rect);
  }

  // ---------------- phase 2-3: red beacon ----------------
  void detectColor(const cv::Mat& rgb, follow_me_msgs::msg::Detection& det) {
    // 1. Colour segmentation in HSV (red wraps around hue 0).
    cv::Mat hsv, m1, m2, mask;
    cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, cv::Scalar(0, sat_min_, val_min_), cv::Scalar(hue_max_, 255, 255), m1);
    cv::inRange(hsv, cv::Scalar(180 - hue_max_, sat_min_, val_min_), cv::Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);
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
  }

  // ---------------- phase 4: dark silhouette against the sky, inside the cued window ----------
  cv::Rect detectSilhouette(const cv::Mat& rgb, const builtin_interfaces::msg::Time& stamp,
                            follow_me_msgs::msg::Detection& det) {
    if (!roi_ || !roi_->valid ||
        (rclcpp::Time(stamp) - rclcpp::Time(roi_->header.stamp)).seconds() > roi_timeout_)
      return {};  // no cue: we do not search the whole image (no colour to rely on)
    const int h = static_cast<int>(roi_->half_size);
    const cv::Rect r = cv::Rect(static_cast<int>(roi_->u) - h, static_cast<int>(roi_->v) - h, 2 * h, 2 * h) &
                       cv::Rect(0, 0, rgb.cols, rgb.rows);
    if (r.width < 5 || r.height < 5) return {};
    cv::Mat gray;
    cv::cvtColor(rgb(r), gray, cv::COLOR_RGB2GRAY);
    // Sky brightness = 75th percentile of the window (robust to the drone itself).
    int hist[256] = {0};
    for (int y = 0; y < gray.rows; ++y)
      for (int x = 0; x < gray.cols; ++x) ++hist[gray.at<uint8_t>(y, x)];
    int acc = 0, bg = 255;
    for (int i = 0; i < 256; ++i)
      if ((acc += hist[i]) >= 0.75 * gray.total()) { bg = i; break; }
    cv::Mat mask = gray < dark_ratio_ * bg;
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    // Keep the blob closest to the cue centre that does not touch the window border (a blob
    // touching it is the ground / horizon, or a truncated target)...
    auto box = [&](int i) {
      return cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
                      stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
    };
    auto usable = [&](int i) {
      const cv::Rect b = box(i);
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      return area >= 1 && area <= 0.5 * gray.total() && b.x > 0 && b.y > 0 &&
             b.x + b.width < gray.cols && b.y + b.height < gray.rows;
    };
    int best = -1;
    double best_d = 1e9;
    for (int i = 1; i < n; ++i) {
      if (!usable(i) || stats.at<int>(i, cv::CC_STAT_AREA) < 2) continue;
      const cv::Rect b = box(i);
      const double d = std::hypot(b.x + b.width / 2.0 - (roi_->u - r.x), b.y + b.height / 2.0 - (roi_->v - r.y));
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    if (best < 0) return r;
    // ...then grow it with the nearby blobs: an unknown drone is NOT one blob (body and rotors
    // separate as soon as the thin arms fall below one pixel). Measuring only the central blob
    // made the apparent size depend on the range (Gazebo phase 4).
    cv::Rect merged = box(best);
    std::vector<bool> used(n, false);
    used[best] = true;
    for (bool grew = true; grew;) {
      grew = false;
      const int gap = std::max(3, static_cast<int>(0.6 * std::max(merged.width, merged.height)));
      const cv::Rect zone(merged.x - gap, merged.y - gap, merged.width + 2 * gap, merged.height + 2 * gap);
      for (int i = 1; i < n; ++i) {
        if (used[i] || !usable(i) || (box(i) & zone).area() == 0) continue;
        merged |= box(i);
        used[i] = true;
        grew = true;
      }
    }
    const int x = merged.x, y = merged.y, w = merged.width, hh = merged.height;
    det.detected = true;
    det.u = r.x + x + w / 2.0;  // pixel i spans [i, i+1): the box centre is already continuous
    det.v = r.y + y + hh / 2.0;
    det.size = std::max(w, hh);
    if (size_mode_ == "moment") {
      // Yaw-invariant size: for 4 rotors on a circle of radius r, 90 deg apart, the sum of their
      // squared horizontal offsets is 2 r^2 whatever the yaw, whereas the bounding-box width of an
      // X shape varies by ~30 % (narrow from behind, wide along a diagonal). Gazebo phase 4: the
      // apparent size followed the target's relative yaw (+17 % at 32 deg), a SLOW error that
      // biased the range by 1-2 m. Size = 4 x horizontal standard deviation of the blob pixels.
      double sw = 0, sx = 0, sxx = 0, sy = 0, syy = 0;
      for (int yy = y; yy < y + hh; ++yy)
        for (int xx = x; xx < x + w; ++xx) {
          const int lab = labels.at<int>(yy, xx);
          if (lab > 0 && used[lab]) {
            sw += 1;
            sx += xx;
            sxx += double(xx) * xx;
            sy += yy;
            syy += double(yy) * yy;
          }
        }
      if (sw > 0) {
        det.size = 4.0 * std::sqrt(std::max(sxx / sw - (sx / sw) * (sx / sw), 0.0) + 1.0 / 12.0);
        det.thickness = 4.0 * std::sqrt(std::max(syy / sw - (sy / sw) * (sy / sw), 0.0) + 1.0 / 12.0);
      }
    }
    const bool on_image_border = r.x + x <= 1 || r.y + y <= 1 || r.x + x + w >= rgb.cols - 1 ||
                                 r.y + y + hh >= rgb.rows - 1;
    det.size_valid = !on_image_border;
    return r;
  }

  void finish(const cv::Mat& rgb, const sensor_msgs::msg::Image::ConstSharedPtr& msg,
              follow_me_msgs::msg::Detection& det, const cv::Rect& roi_rect) {
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
      const int text_y = static_cast<int>(50 * rgb.cols / 640.0);  // below the HUD top band (hud_node)
      if (roi_rect.area() > 0) cv::rectangle(dbg, roi_rect, {255, 220, 0}, 1);  // cued window
      if (det.detected) {
        const cv::Scalar col = det.injected_outlier ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0);
        cv::circle(dbg, {static_cast<int>(det.u), static_cast<int>(det.v)},
                   std::max(4, static_cast<int>(det.size / 2 + 4)), col, 1);
        char txt[96];
        std::snprintf(txt, sizeof(txt), "%s u=%.0f v=%.0f s=%.1fpx%s",
                      det.injected_outlier ? "OUTLIER" : "det", det.u, det.v, det.size,
                      det.size_valid ? "" : " TRUNCATED (bearing only)");
        if (debug_text_) cv::putText(dbg, txt, {8, text_y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
      } else {
        if (debug_text_) cv::putText(dbg, "no detection", {8, text_y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 0}, 1);
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
  double p_miss_, p_outlier_, size_bias_px_, fill_min_, dark_ratio_, roi_timeout_;
  std::string mode_, size_mode_;
  bool debug_text_ = false;
  std::optional<follow_me_msgs::msg::Roi> roi_;
  rclcpp::Subscription<follow_me_msgs::msg::Roi>::SharedPtr sub_roi_;
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
