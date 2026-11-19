// hud_node: head-up display over the detector debug image, for demos and debugging.
//
// Inputs : /follower/detector/debug_image (camera image + cued window + detection),
//          /target/estimate (track status, camera lock, learnt size), /mission/phase (phase 4),
//          /follower/odom (navigation: range to the estimate).
// Output : /follower/hud_image (published at the camera rate).
// Pure visualisation: no other node depends on it, so it never changes the closed loop.
#include <cv_bridge/cv_bridge.hpp>
#include <follow_me_msgs/msg/target_estimate.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using follow_me_msgs::msg::TargetEstimate;

namespace {
// RGB colours (the image is rgb8). Same phase palette as the figures and the web replay.
const cv::Scalar kInk(240, 240, 236), kMuted(170, 169, 164), kGreen(27, 175, 122), kOrange(235, 104, 52),
    kAmber(237, 161, 0), kGrey(169, 168, 163), kRed(227, 73, 72);

cv::Scalar phaseColor(const std::string& p) {
  if (p == "MIDCOURSE") return kOrange;
  if (p == "HANDOVER") return kAmber;
  if (p == "FOLLOW") return kGreen;
  return kGrey;
}

void text(cv::Mat& img, const std::string& s, cv::Point at, double scale, const cv::Scalar& col, int thick = 1) {
  cv::putText(img, s, at, cv::FONT_HERSHEY_SIMPLEX, scale, col, thick, cv::LINE_AA);
}

// Darkened band so the text stays readable over the sky and the ground.
void band(cv::Mat& img, const cv::Rect& r, double alpha = 0.55) {
  const cv::Rect rr = r & cv::Rect(0, 0, img.cols, img.rows);
  if (rr.area() <= 0) return;
  cv::Mat roi = img(rr);
  roi.convertTo(roi, -1, 1.0 - alpha, 0);
}

void cornerBrackets(cv::Mat& img, cv::Point c, int half, int len, const cv::Scalar& col) {
  for (int sx : {-1, 1})
    for (int sy : {-1, 1}) {
      const cv::Point corner(c.x + sx * half, c.y + sy * half);
      cv::line(img, corner, {corner.x - sx * len, corner.y}, col, 2, cv::LINE_AA);
      cv::line(img, corner, {corner.x, corner.y - sy * len}, col, 2, cv::LINE_AA);
    }
}
}  // namespace

class HudNode : public rclcpp::Node {
 public:
  HudNode() : Node("hud_node") {
    d_min_ = declare_parameter("hud.d_min", 4.0);
    standoff_ = declare_parameter("hud.standoff", 8.25);
    upscale_ = declare_parameter("hud.upscale", 2);
    sub_est_ = create_subscription<TargetEstimate>("/target/estimate", 1, [this](const TargetEstimate& m) { est_ = m; });
    sub_phase_ = create_subscription<std_msgs::msg::String>(
        "/mission/phase", 1, [this](const std_msgs::msg::String& m) { phase_ = m.data; });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/follower/odom", 1, [this](const nav_msgs::msg::Odometry& m) { odom_ = m; });
    pub_ = create_publisher<sensor_msgs::msg::Image>("/follower/hud_image", 5);
    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        "/follower/detector/debug_image", rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { onImage(m); });
  }

 private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (pub_->get_subscription_count() == 0) return;
    cv_bridge::CvImagePtr cv;
    try {
      cv = cv_bridge::toCvCopy(msg, "rgb8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }
    // Drawn at 2x resolution: the HUD is often shown small (RViz image panel ~330 px wide), where
    // text drawn at 640 px became unreadable; a 1280 px image downscales into crisp text.
    if (upscale_ > 1) cv::resize(cv->image, cv->image, cv::Size(), upscale_, upscale_, cv::INTER_LINEAR);
    cv::Mat& img = cv->image;
    const double k = img.cols / 640.0;  // scale the layout with the resolution
    const int W = img.cols, H = img.rows;

    const bool fresh = est_ && (rclcpp::Time(msg->header.stamp) - rclcpp::Time(est_->header.stamp)).seconds() < 0.5;
    const bool tracked = fresh && est_->status != TargetEstimate::SEARCH;
    const bool locked = tracked && est_->camera_locked;
    double range = -1.0;
    if (tracked && odom_) {
      const auto& p = odom_->pose.pose.position;
      range = std::hypot(est_->position.x - p.x, est_->position.y - p.y, est_->position.z - p.z);
    }

    // ---- top band, two lines in large type (readable in a small panel) ----
    //   PHASE                          SOURCE
    //   range 8.2 m                    size 1.48 +/- 0.09 m
    const int top = static_cast<int>(52 * k);
    band(img, {0, 0, W, top}, 0.6);
    const int bold = std::max(2, int(1.6 * k + 0.5)), thin = std::max(1, int(1.1 * k + 0.5));
    const std::string phase = phase_.empty() ? (tracked ? "FOLLOW-ME" : "SEARCH") : phase_;
    const cv::Scalar pc = phase_.empty() ? (tracked ? kGreen : kGrey) : phaseColor(phase_);
    cv::rectangle(img, cv::Rect(int(8 * k), int(7 * k), int(16 * k), int(16 * k)), pc, cv::FILLED);
    text(img, phase, {int(30 * k), int(22 * k)}, 0.68 * k, pc, bold);
    // Phase 4 (mission running): radar alone, then radar + camera. Phases 2-3: camera only.
    const std::string src = !tracked ? "NO TRACK"
                            : phase_.empty() ? "CAMERA"
                            : (locked ? "RADAR + CAMERA" : "RADAR ONLY");
    const cv::Scalar src_col = (locked || (tracked && phase_.empty())) ? kGreen : (tracked ? kAmber : kMuted);
    int base = 0;
    const cv::Size ts = cv::getTextSize(src, cv::FONT_HERSHEY_SIMPLEX, 0.6 * k, bold, &base);
    text(img, src, {W - ts.width - int(10 * k), int(22 * k)}, 0.6 * k, src_col, bold);
    char buf[96];
    std::snprintf(buf, sizeof(buf), range >= 0 ? "range %.1f m" : "range --", range);
    text(img, buf, {int(10 * k), int(45 * k)}, 0.58 * k, kInk, thin);
    if (fresh && est_->target_size > 0 && est_->target_size_sigma < 0.25)
      std::snprintf(buf, sizeof(buf), "size %.2f +/- %.2f m", est_->target_size, 2 * est_->target_size_sigma);
    else if (phase_.empty())
      std::snprintf(buf, sizeof(buf), "beacon size known");
    else
      std::snprintf(buf, sizeof(buf), "size unknown");
    const cv::Size ss = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.58 * k, thin, &base);
    text(img, buf, {W - ss.width - int(10 * k), int(45 * k)}, 0.58 * k, kInk, thin);

    // ---- boresight reticle (gimbal axis) ----
    const cv::Point c(W / 2, H / 2);
    cornerBrackets(img, c, int(26 * k), int(9 * k), locked ? kGreen : kInk);
    cv::line(img, {c.x - int(5 * k), c.y}, {c.x + int(5 * k), c.y}, kInk, 1, cv::LINE_AA);
    cv::line(img, {c.x, c.y - int(5 * k)}, {c.x, c.y + int(5 * k)}, kInk, 1, cv::LINE_AA);
    if (locked) text(img, "LOCK", {c.x + int(32 * k), c.y - int(30 * k)}, 0.6 * k, kGreen, std::max(2, int(1.6 * k + 0.5)));

    // ---- bottom: range ladder on a log scale, with the safety minimum and the standoff ----
    const int bh = static_cast<int>(30 * k), y0 = H - bh;
    band(img, {0, y0, W, bh});
    const int x0 = int(70 * k), x1 = W - int(16 * k), ym = y0 + bh / 2;
    auto xr = [&](double r) {
      const double u = std::clamp((std::log10(std::max(r, 1.0)) - 0.0) / 3.0, 0.0, 1.0);  // 1 m .. 1 km
      return int(x0 + u * (x1 - x0));
    };
    text(img, "range", {int(8 * k), ym + int(5 * k)}, 0.5 * k, kMuted, std::max(1, int(k + 0.5)));
    cv::line(img, {x0, ym}, {x1, ym}, kMuted, 1, cv::LINE_AA);
    for (double r : {1.0, 10.0, 100.0, 1000.0}) {
      cv::line(img, {xr(r), ym - int(3 * k)}, {xr(r), ym + int(3 * k)}, kMuted, 1);
      std::snprintf(buf, sizeof(buf), r < 1000 ? "%.0f" : "1k", r);
      text(img, buf, {xr(r) - int(5 * k), ym + int(13 * k)}, 0.4 * k, kMuted, std::max(1, int(k + 0.5)));
    }
    cv::rectangle(img, cv::Point(x0, ym - int(4 * k)), cv::Point(xr(d_min_), ym + int(4 * k)), kRed, cv::FILLED);
    cv::line(img, {xr(standoff_), ym - int(7 * k)}, {xr(standoff_), ym + int(7 * k)}, kGreen, 2);
    if (range >= 0) {
      const int xx = xr(range);
      const cv::Point tri[3] = {{xx, ym - int(2 * k)}, {xx - int(5 * k), ym - int(10 * k)}, {xx + int(5 * k), ym - int(10 * k)}};
      cv::fillConvexPoly(img, tri, 3, range < d_min_ ? kRed : kInk, cv::LINE_AA);
    }
    pub_->publish(*cv->toImageMsg());
  }

  double d_min_, standoff_;
  int upscale_;
  std::optional<TargetEstimate> est_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::string phase_;
  rclcpp::Subscription<TargetEstimate>::SharedPtr sub_est_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_phase_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HudNode>());
  rclcpp::shutdown();
  return 0;
}
