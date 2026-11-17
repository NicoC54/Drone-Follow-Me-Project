// Fusion of DELAYED measurements.
//
// A camera detection describes the scene at the image capture time t_c, but reaches the tracker
// later (rendering, transport, image processing). Fusing it with the camera pose of "now" is wrong:
// if the follower moved 2 m in between, the measurement looks inconsistent and is rejected.
//
// Two ingredients:
//  - TimeSeries (time_series.hpp): stamped history of each sensor stream, so we can ask
//    "where was the camera at t_c?".
//  - TimedTracker: the EKF lives at the time of the last fused measurement; its output is
//    predicted forward to the current time on request. Measurements arrive in capture order
//    (single camera), so no rollback/replay is needed.
#pragma once

#include <limits>
#include <optional>

#include "follow_me_core/track_manager.hpp"

namespace follow_me {

class TimedTracker {
 public:
  TimedTracker(const EstimatorParams& p, const PinholeCamera& cam) : tracker_(p, cam) {}

  // z was captured at t_capture, with the camera at (cam_pos, R_cw) at that time.
  // Returns true if used. Measurements older than the last fused one are dropped.
  bool onMeasurement(const Vec3& z, double t_capture, const Vec3& cam_pos, const Mat3& R_cw,
                     const Mat3& R_extra = Mat3::Zero(), bool size_valid = true);
  void updateStatus(double now) { tracker_.updateStatus(now); }

  // Filter state predicted to time t (>= filter time): what guidance and control should use.
  TargetEkf predictedAt(double t) const;

  const TargetTracker& tracker() const { return tracker_; }
  Mat3 cameraPoseNoise(const Vec3& cam_pos, const Mat3& R_cw, const Mat3& P_cam_pos,
                       double sigma_att) const {
    return tracker_.cameraPoseNoise(tracker_.ekf().position(), cam_pos, R_cw, P_cam_pos, sigma_att);
  }
  bool hasEstimate() const { return tracker_.hasEstimate(); }
  TrackStatus status() const { return tracker_.status(); }
  double filterTime() const { return t_filter_; }
  int dropped() const { return dropped_; }

 private:
  TargetTracker tracker_;
  double t_filter_ = -std::numeric_limits<double>::infinity();
  int dropped_ = 0;
};

}  // namespace follow_me
