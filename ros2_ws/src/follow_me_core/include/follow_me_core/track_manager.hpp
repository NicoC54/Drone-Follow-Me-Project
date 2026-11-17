// Track management around the EKF: confirmed initialisation, chi2 gating, track status.
#pragma once

#include <cstdint>
#include <limits>
#include <optional>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/ekf.hpp"
#include "follow_me_core/params.hpp"

namespace follow_me {

enum class TrackStatus : std::uint8_t {
  Search = 0,  // no valid estimate: hover and sweep the gimbal
  Track = 1,   // a measurement was accepted recently
  Coast = 2,   // no recent measurement: follow the prediction
};

const char* toString(TrackStatus s);

class TargetTracker {
 public:
  TargetTracker(const EstimatorParams& p, const PinholeCamera& cam);

  void predict(double dt);
  // Returns true if the measurement was used (update or track start).
  // R_extra: extra pixel covariance from the camera pose uncertainty (default: pose perfect).
  // size_valid = false: the apparent size is unreliable (truncated beacon) -> only the bearing
  // (u, v) is fused, and no track is started from it.
  bool onMeasurement(const Vec3& z, double t, const Vec3& cam_pos, const Mat3& R_cw,
                     const Mat3& R_extra = Mat3::Zero(), bool size_valid = true);

  // Pixel covariance induced by an uncertain camera pose (from the follower's navigation):
  //   position error (cov P_cam_pos) moves the target like the opposite position error,
  //   attitude error (std sigma_att, rad) shifts the image by ~ f * sigma_att pixels.
  Mat3 cameraPoseNoise(const Vec3& p_target, const Vec3& cam_pos, const Mat3& R_cw,
                       const Mat3& P_cam_pos, double sigma_att) const;
  void updateStatus(double t);

  bool hasEstimate() const { return status_ != TrackStatus::Search; }
  TrackStatus status() const { return status_; }
  const TargetEkf& ekf() const { return ekf_; }
  double lastNis() const { return last_nis_; }

 private:
  // A track starts only when a 2nd detection agrees with the 1st one (chi2 test).
  bool tryInit(const Vec3& z, double t, const Vec3& cam_pos, const Mat3& R_cw);
  bool updateBearingOnly(const TargetEkf::Innovation& in, const Mat3& R, double t);

  struct Candidate {
    double t;
    Vec3 p;
    Mat3 P;
  };

  EstimatorParams p_;
  PinholeCamera cam_;
  TargetEkf ekf_;
  TrackStatus status_ = TrackStatus::Search;
  double t_last_ = -std::numeric_limits<double>::infinity();
  int n_rejects_ = 0;
  double last_nis_ = std::numeric_limits<double>::quiet_NaN();
  std::optional<Candidate> candidate_;
};

}  // namespace follow_me
