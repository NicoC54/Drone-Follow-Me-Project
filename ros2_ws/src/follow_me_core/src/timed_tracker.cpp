#include "follow_me_core/timed_tracker.hpp"

#include <cmath>

namespace follow_me {

bool TimedTracker::onMeasurement(const Vec3& z, double t_capture, const Vec3& cam_pos,
                                 const Mat3& R_cw, const Mat3& R_extra, bool size_valid) {
  if (t_capture < t_filter_) {  // out of order: older than what is already fused
    ++dropped_;
    return false;
  }
  if (std::isfinite(t_filter_)) tracker_.predict(t_capture - t_filter_);
  t_filter_ = t_capture;
  return tracker_.onMeasurement(z, t_capture, cam_pos, R_cw, R_extra, size_valid);
}

TargetEkf TimedTracker::predictedAt(double t) const {
  TargetEkf ekf = tracker_.ekf();
  if (std::isfinite(t_filter_) && t > t_filter_) ekf.predict(t - t_filter_);
  return ekf;
}

}  // namespace follow_me
