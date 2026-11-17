// Stamped history of one signal, queried at an arbitrary time by interpolation.
//
// Each sensor stream (odometry, gimbal angles, ...) gets its OWN history, stamped with its OWN
// timestamps. To know the camera pose when an image was taken, every stream is interpolated at
// the image stamp. Pairing "the latest odometry" with "the latest gimbal angles" is wrong as soon
// as the messages are queued (CPU load): the two "latest" can be tens of ms apart.
#pragma once

#include <deque>
#include <optional>
#include <utility>

namespace follow_me {

template <typename T>
class TimeSeries {
 public:
  explicit TimeSeries(double horizon = 2.0, double max_after = 0.05)
      : horizon_(horizon), max_after_(max_after) {}

  // Samples must be added in increasing time; older or duplicate stamps are ignored.
  void add(double t, const T& v) {
    if (!buf_.empty() && t <= buf_.back().first) return;
    buf_.emplace_back(t, v);
    while (buf_.front().first < t - horizon_) buf_.pop_front();
  }

  // Value at t. lerp(a, b, w) blends two samples (w in [0, 1]).
  // Empty if t is before the history or more than max_after after its last sample.
  template <typename Lerp>
  std::optional<T> at(double t, Lerp lerp) const {
    if (buf_.empty() || t < buf_.front().first) return std::nullopt;
    if (t >= buf_.back().first) {
      if (t - buf_.back().first > max_after_) return std::nullopt;
      return buf_.back().second;
    }
    size_t hi = 1;
    while (buf_[hi].first <= t) ++hi;
    const auto& [ta, a] = buf_[hi - 1];
    const auto& [tb, b] = buf_[hi];
    return lerp(a, b, (t - ta) / (tb - ta));
  }

  bool empty() const { return buf_.empty(); }
  size_t size() const { return buf_.size(); }

 private:
  double horizon_, max_after_;
  std::deque<std::pair<double, T>> buf_;
};

}  // namespace follow_me
