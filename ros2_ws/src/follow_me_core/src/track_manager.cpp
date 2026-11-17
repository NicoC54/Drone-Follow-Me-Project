#include "follow_me_core/track_manager.hpp"

namespace follow_me {

const char* toString(TrackStatus s) {
  switch (s) {
    case TrackStatus::Search: return "SEARCH";
    case TrackStatus::Track: return "TRACK";
    case TrackStatus::Coast: return "COAST";
  }
  return "?";
}

TargetTracker::TargetTracker(const EstimatorParams& p, const PinholeCamera& cam)
    : p_(p), cam_(cam), ekf_(p.q_accel) {}

void TargetTracker::predict(double dt) {
  if (hasEstimate()) ekf_.predict(dt);
}

bool TargetTracker::tryInit(const Vec3& z, double t, const Vec3& cam_pos, const Mat3& R_cw) {
  const auto bp = cam_.backproject(z, cam_pos, R_cw);
  const std::optional<Candidate> prev = candidate_;
  candidate_ = Candidate{t, bp.p, bp.P};
  if (!prev || t - prev->t > p_.confirm_window) return false;

  const double dt = t - prev->t;
  const double sv = p_.init_vel_sigma * dt;  // the leader may have moved in between
  const Mat3 S = bp.P + prev->P + sv * sv * Mat3::Identity();
  const Vec3 d = bp.p - prev->p;
  if (d.dot(S.ldlt().solve(d)) > p_.gate_chi2) return false;

  Vec6 x;
  x << bp.p, Vec3::Zero();
  Mat6 P = Mat6::Zero();
  P.topLeftCorner<3, 3>() = bp.P;
  P.bottomRightCorner<3, 3>() = p_.init_vel_sigma * p_.init_vel_sigma * Mat3::Identity();
  ekf_.reset(x, P);
  status_ = TrackStatus::Track;
  t_last_ = t;
  n_rejects_ = 0;
  candidate_.reset();
  return true;
}

Mat3 TargetTracker::cameraPoseNoise(const Vec3& p_target, const Vec3& cam_pos, const Mat3& R_cw,
                                    const Mat3& P_cam_pos, double sigma_att) const {
  const Vec3 rc = PinholeCamera::toCamera(p_target, cam_pos, R_cw);
  if (rc.z() <= 0.1) return Mat3::Zero();
  const Mat3 H = cam_.jacobianPos(rc, R_cw);
  const auto& c = cam_.params();
  const Mat3 att = Vec3(std::pow(c.fx * sigma_att, 2), std::pow(c.fy * sigma_att, 2), 0.0).asDiagonal();
  return H * P_cam_pos * H.transpose() + att;
}

bool TargetTracker::onMeasurement(const Vec3& z, double t, const Vec3& cam_pos, const Mat3& R_cw,
                                  const Mat3& R_extra, bool size_valid) {
  if (!hasEstimate()) return size_valid && tryInit(z, t, cam_pos, R_cw);

  if (const auto in = ekf_.innovation(z, cam_, cam_pos, R_cw, R_extra)) {
    if (!size_valid) return updateBearingOnly(*in, cam_.noiseCov() + R_extra, t);
    last_nis_ = in->y.dot(in->S.ldlt().solve(in->y));  // normalised innovation squared
    if (last_nis_ <= p_.gate_chi2) {
      ekf_.update(*in, cam_.noiseCov() + R_extra);
      status_ = TrackStatus::Track;
      t_last_ = t;
      n_rejects_ = 0;
      return true;
    }
  }
  // Rejected. Many rejections in a row: the filter is wrong, not the sensor -> drop the track.
  if (++n_rejects_ >= p_.max_rejects) {
    status_ = TrackStatus::Search;
    candidate_.reset();
    return tryInit(z, t, cam_pos, R_cw);
  }
  return false;
}

bool TargetTracker::updateBearingOnly(const TargetEkf::Innovation& in, const Mat3& R, double t) {
  // Keep the first two rows (u, v): the size row is dropped.
  TargetEkf::Innovation in2;
  in2.y = Vec3(in.y(0), in.y(1), 0.0);
  in2.H = in.H;
  in2.H.row(2).setZero();
  Mat3 R2 = R;
  R2(2, 2) = 1e12;  // "infinite" noise on the size: no information
  R2.row(2).head<2>().setZero();
  R2.col(2).head<2>().setZero();
  in2.S = in2.H * ekf_.P() * in2.H.transpose() + R2;
  const Eigen::Vector2d y = in2.y.head<2>();
  const Eigen::Matrix2d S = in2.S.topLeftCorner<2, 2>();
  last_nis_ = y.dot(S.ldlt().solve(y));
  if (last_nis_ > p_.gate_chi2_2d) return false;
  ekf_.update(in2, R2);
  status_ = TrackStatus::Track;
  t_last_ = t;
  n_rejects_ = 0;
  return true;
}

void TargetTracker::updateStatus(double t) {
  if (!hasEstimate()) return;
  const double age = t - t_last_;
  if (age > p_.lost_time) {
    status_ = TrackStatus::Search;
  } else if (age > p_.coast_time) {
    status_ = TrackStatus::Coast;
  }
}

}  // namespace follow_me
