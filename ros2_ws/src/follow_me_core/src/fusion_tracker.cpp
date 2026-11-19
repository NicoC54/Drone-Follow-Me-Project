#include "follow_me_core/fusion_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace follow_me {

FusionTracker::FusionTracker(const FusionParams& p, const CameraParams& cam,
                             const GroundRadarParams& radar)
    : p_(p), cam_prm_(cam), radar_prm_(radar), R_radar_(radarNoiseCov(radar)) {
  c_.ekf = TargetEkf(p.q_accel);
  c_.imm = makeImm();
  c_.W = p.size_prior;
  c_.W_var = p.size_prior_sigma * p.size_prior_sigma;
  base_ = c_;
}

TargetImm FusionTracker::makeImm() const {
  using M = TargetEkfCA::Model;
  const std::vector<double> soj{p_.sojourn_calm, p_.sojourn_manoeuvre};
  const std::vector<double> mu0{1.0 - p_.mu0_manoeuvre, p_.mu0_manoeuvre};
  if (p_.model == "cv") return TargetImm({TargetEkfCA(M::CV, p_.q_accel)}, {1e9}, {1.0});
  if (p_.model == "singer") return TargetImm({TargetEkfCA(M::Singer, p_.q_jerk, p_.singer_tau)}, {1e9}, {1.0});
  if (p_.model == "imm_cv")
    return TargetImm({TargetEkfCA(M::CV, p_.q_calm), TargetEkfCA(M::CV, p_.q_cv_manoeuvre)}, soj, mu0);
  return TargetImm({TargetEkfCA(M::CV, p_.q_calm), TargetEkfCA(M::Singer, p_.q_jerk, p_.singer_tau)}, soj, mu0);
}

Vec3 FusionTracker::accelerationAt(double t) const {
  if (!hasEstimate()) return Vec3::Zero();
  TargetImm imm = c_.imm;
  if (std::isfinite(c_.t_filter) && t > c_.t_filter) imm.predict(t - c_.t_filter);
  return imm.combined9().acceleration();
}

void FusionTracker::onCamera(double t, const CameraObs& obs) { add({t, obs}); }
void FusionTracker::onRadar(double t, const RadarObs& obs) { add({t, obs}); }

// ---------------- out-of-sequence handling ----------------
void FusionTracker::add(Entry e) {
  if (history_.empty() || e.t >= history_.back().t) {
    process(c_, e);
    history_.push_back(std::move(e));
    snapshots_.push_back(c_);
  } else {
    // Late measurement: restore the state just before its time stamp, then re-fuse in order.
    const auto it = std::upper_bound(history_.begin(), history_.end(), e.t,
                                     [](double t, const Entry& x) { return t < x.t; });
    const size_t k = static_cast<size_t>(it - history_.begin());
    Core c = (k == 0) ? base_ : snapshots_[k - 1];
    if (std::isfinite(c.t_filter) && e.t < c.t_filter) return;  // older than the buffer: drop
    history_.insert(history_.begin() + static_cast<long>(k), std::move(e));
    snapshots_.erase(snapshots_.begin() + static_cast<long>(k), snapshots_.end());
    for (size_t i = k; i < history_.size(); ++i) {
      process(c, history_[i]);
      snapshots_.push_back(c);
    }
    ++replays_;
    replayed_ += static_cast<int>(history_.size() - k);
    c_ = c;
  }
  // Keep a few seconds of history.
  const double newest = history_.back().t;
  while (history_.size() > 1 && history_.front().t < newest - p_.history) {
    base_ = snapshots_.front();
    history_.pop_front();
    snapshots_.pop_front();
  }
}

void FusionTracker::process(Core& c, const Entry& e) const {
  if (c.status != TrackStatus::Search && std::isfinite(c.t_filter) && e.t > c.t_filter) {
    c.imm.predict(e.t - c.t_filter);
    c.ekf = c.imm.combined();
  }
  if (!std::isfinite(c.t_filter) || e.t > c.t_filter) c.t_filter = e.t;
  if (e.t > c.t_W) {  // the apparent size of a drone changes slowly with the aspect angle
    c.W_var += p_.size_process * p_.size_process * (e.t - c.t_W);
    c.t_W = e.t;
  }
  if (const auto* cam = std::get_if<CameraObs>(&e.obs)) processCamera(c, e.t, *cam);
  if (const auto* rad = std::get_if<RadarObs>(&e.obs)) processRadar(c, e.t, *rad);
}

// ---------------- camera: bearing, plus range once the size is known ----------------
void FusionTracker::processCamera(Core& c, double t, const CameraObs& o) const {
  const bool resolved = o.size_valid && o.z(2) >= p_.size_min_px;
  if (resolved) c.last_size = Core::LastSize{t, o.z(2), o.cam_pos, o.R_cw};
  if (c.status == TrackStatus::Search || c.tentative) return;  // the radar starts the tracks

  CameraParams cp = cam_prm_;
  cp.target_size = c.W;  // current size estimate
  const PinholeCamera cam(cp);
  const bool size_known = std::sqrt(c.W_var) / c.W < p_.size_sigma_rel_ok;
  const bool use_size = resolved && size_known;

  Mat3 R_extra = o.R_extra;
  const Vec3 rc = PinholeCamera::toCamera(c.ekf.position(), o.cam_pos, o.R_cw);
  if (rc.z() <= 0.1) return;
  // Detectability: if even a large drone (size + 2 sigma) would be under detectable_px at the
  // tracked depth, whatever the detector found is NOT the target (Gazebo phase 4 with scenery: 2 px
  // trees on the horizon at 480 m, where the drone is 0.4 px, pulled the radar track away).
  if (cp.fx * (c.W + 2.0 * std::sqrt(c.W_var)) / rc.z() < p_.detectable_px) return;
  // Size plausibility (before the size is learnt the camera is used for bearing only, so nothing
  // else checks it): at the depth given by the track, the blob must not be larger than any drone.
  // Gazebo phase 4 with scenery: a 35 px group of trees on the horizon at 480 m (= a 40 m object)
  // was accepted as the target and dragged the track away.
  if (o.size_valid && o.z(2) > 0.0) {
    const Vec3 fwd = o.R_cw.row(2).transpose();  // optical axis in the world frame
    const double sz = std::sqrt(std::max(fwd.dot(c.ekf.positionCov() * fwd), 0.0));
    const double z_near = std::max(rc.z() - 3.0 * sz, 0.1);
    if (o.z(2) * z_near / cp.fx > p_.size_max) {
      c.cam_consec = 0;
      ++c.stats.cam_rejected;
      return;
    }
  }
  if (use_size) {  // the size prediction f*W/z inherits the uncertainty of W, plus the aspect
    const double ds = cp.fx * std::sqrt(c.W_var) / rc.z();
    const double dt_cam = std::max(t - c.t_last_size_cam, 1e-3);
    const double corr = std::max(1.0, 2.0 * p_.size_aspect_tau / dt_cam);
    const double da = p_.size_rel_noise * o.z(2);
    R_extra(2, 2) += ds * ds + da * da * corr;
  }
  const auto in = c.ekf.innovation(o.z, cam, o.cam_pos, o.R_cw, R_extra);
  if (!in) return;

  bool ok;
  if (use_size) {
    const double nis = in->y.dot(in->S.ldlt().solve(in->y));
    ok = nis <= p_.gate_cam_3d;
    if (ok) {
      const Mat3 R = cam.noiseCov() + R_extra;
      c.imm.update([&](TargetEkfCA& m) {
        const auto im = m.innovation(o.z, cam, o.cam_pos, o.R_cw, R_extra);
        if (!im) return std::numeric_limits<double>::quiet_NaN();
        const double ll = innovationLogLikelihood<3>(im->y, im->S);
        m.update(*im, R);
        return ll;
      });
      c.ekf = c.imm.combined();
      c.t_last_size_cam = t;
    }
  } else {  // bearing only: rows u, v
    const Eigen::Vector2d y = in->y.head<2>();
    const Eigen::Matrix<double, 2, 6> H = in->H.topRows<2>();
    const Eigen::Matrix2d R = (cam.noiseCov() + R_extra).topLeftCorner<2, 2>();
    const Eigen::Matrix2d S = H * c.ekf.P() * H.transpose() + R;
    ok = y.dot(S.ldlt().solve(y)) <= p_.gate_cam_2d;  // gate on the combined estimate
    if (ok) {
      c.imm.update([&](TargetEkfCA& m) {
        const auto im = m.innovation(o.z, cam, o.cam_pos, o.R_cw, R_extra);
        if (!im) return std::numeric_limits<double>::quiet_NaN();
        const Eigen::Vector2d ym = im->y.head<2>();
        const Eigen::Matrix<double, 2, 9> Hm = im->H.topRows<2>();
        const double ll = innovationLogLikelihood<2>(ym, Eigen::Matrix2d(Hm * m.P() * Hm.transpose() + R));
        if (p_.bearing_keeps_range)
          m.updateBearingKeepRange(ym, Hm, R, (m.position() - o.cam_pos).normalized());
        else
          m.updateGeneric<2>(ym, Hm, R, std::numeric_limits<double>::infinity());
        return ll;
      });
      c.ekf = c.imm.combined();
    }
  }
  if (ok) {
    c.status = TrackStatus::Track;
    c.t_last_update = std::max(c.t_last_update, t);
    c.t_last_cam = std::max(c.t_last_cam, t);
    ++c.cam_consec;
    ++c.stats.cam_used;
  } else {
    // A rejected camera detection (wrong blob, other object) must not kill a good radar track:
    // it only breaks the camera lock.
    c.cam_consec = 0;
    ++c.stats.cam_rejected;
  }
}

// ---------------- radar: own-ship / target / clutter association ----------------
void FusionTracker::processRadar(Core& c, double t, const RadarObs& o) const {
  GroundRadarParams rp = radar_prm_;  // the radar that took this scan
  if (o.site) rp.position = *o.site;
  const Vec3& site = rp.position;
  const bool has_est = c.status != TrackStatus::Search;  // includes a tentative track
  Vec4 z_pred = Vec4::Zero();
  Mat46 H = Mat46::Zero();
  Mat4 S = Mat4::Identity();
  if (has_est) {
    z_pred = radarMeasure(c.ekf.position(), c.ekf.velocity(), site);
    H = radarJacobian(c.ekf.position(), c.ekf.velocity(), site);
    S = H * c.ekf.P() * H.transpose() + R_radar_;
  }

  int best = -1;
  double best_nis = p_.gate_radar;
  Vec4 best_y = Vec4::Zero();
  std::vector<PolarToCartesian> others;
  for (size_t i = 0; i < o.plots.size(); ++i) {
    const auto& pl = o.plots[i];
    const auto cart = radarToCartesian(pl, rp);
    const Vec3 d_own = cart.p - o.own_pos;
    const double nis_own = d_own.dot((cart.P + o.own_P).ldlt().solve(d_own));
    double nis_t = std::numeric_limits<double>::infinity();
    Vec4 y = Vec4::Zero();
    if (has_est) {
      y = pl.z() - z_pred;
      y(1) = wrapAngle(y(1));
      nis_t = y.dot(S.ldlt().solve(y));
    }
    // Nearest neighbour between the two known objects: the plot goes to the closer one.
    if (nis_t <= best_nis && nis_t < nis_own) {
      best = static_cast<int>(i);
      best_nis = nis_t;
      best_y = y;
    } else if (nis_own <= p_.gate_own) {
      ++c.stats.radar_own;
    } else {
      others.push_back(cart);
    }
  }

  if (has_est) {
    const bool associated = best >= 0;  // best_nis <= gate_radar on the combined estimate
    if (associated) {
      const Vec4 z = o.plots[static_cast<size_t>(best)].z();
      c.imm.update([&](TargetEkfCA& m) {
        const Vec4 zp = radarMeasure(m.position(), m.velocity(), site);
        Eigen::Matrix<double, 4, 9> Hm = Eigen::Matrix<double, 4, 9>::Zero();  // acceleration: no direct effect
        Hm.leftCols<6>() = radarJacobian(m.position(), m.velocity(), site);
        Vec4 y = z - zp;
        y(1) = wrapAngle(y(1));
        const double ll = innovationLogLikelihood<4>(y, Mat4(Hm * m.P() * Hm.transpose() + R_radar_));
        m.updateGeneric<4>(y, Hm, R_radar_, std::numeric_limits<double>::infinity());
        return ll;
      });
      c.ekf = c.imm.combined();
    }
    if (c.tentative) {  // M-of-N confirmation: a miss kills a tentative track
      if (!associated) {
        c.status = TrackStatus::Search;
        c.tentative = false;
        c.candidates = others;
        c.t_candidates = t;
        c.stats.radar_clutter += static_cast<int>(others.size());
        return;
      }
      if (++c.hits >= p_.confirm_hits) c.tentative = false;
    }
    if (associated) {
      c.status = TrackStatus::Track;
      c.t_last_update = std::max(c.t_last_update, t);
      c.t_last_radar = std::max(c.t_last_radar, t);
      ++c.stats.radar_used;
      // Learn the target size from the camera apparent size s and the DEPTH of the radar plot in
      // the camera frame (independent sensors, same instant): W = s * z / f.
      if (c.last_size && std::abs(t - c.last_size->t) < 0.1) {
        const auto cart = radarToCartesian(o.plots[static_cast<size_t>(best)], rp);
        const Vec3 axis = c.last_size->R_cw.row(2).transpose();  // optical axis in the world
        const double z = axis.dot(cart.p - c.last_size->cam_pos);
        const double s = c.last_size->s, f = cam_prm_.fx;
        if (z > 1.0) {
          const double W_m = s * z / f;
          const double var_z = axis.dot(cart.P * axis);  // radar uncertainty along the camera axis
          const double var_m = std::pow(z / f * cam_prm_.sigma_size_px, 2) + std::pow(s / f, 2) * var_z +
                               std::pow(p_.size_rel_noise * W_m, 2) +  // aspect-dependent silhouette
                               std::pow(z / f * p_.size_res_px, 2);    // edge erosion of small blobs
          const double K = c.W_var / (c.W_var + var_m);
          c.W += K * (W_m - c.W);
          c.W_var *= (1.0 - K);
        }
      }
    }
    c.stats.radar_clutter += static_cast<int>(others.size());
    return;
  }

  // No track: start one from two compatible plots on consecutive scans.
  c.stats.radar_clutter += static_cast<int>(others.size());
  if (c.t_candidates > 0.0 && t - c.t_candidates <= 2.5 / radar_prm_.scan_rate) {
    const double dt = t - c.t_candidates;
    for (const auto& cur : others) {
      for (const auto& prev : c.candidates) {
        const Vec3 d = cur.p - prev.p;
        const double sv = p_.init_speed_max * dt / 3.0;  // 3 sigma = max plausible speed
        const Mat3 Sd = cur.P + prev.P + sv * sv * Mat3::Identity();
        if (d.dot(Sd.ldlt().solve(d)) <= p_.gate_init) {
          startTrack(c, t, cur.p, d / dt, cur.P, (cur.P + prev.P) / (dt * dt));
          return;
        }
      }
    }
  }
  c.candidates = others;
  c.t_candidates = t;
}

void FusionTracker::startTrack(Core& c, double t, const Vec3& p, const Vec3& v, const Mat3& Pp,
                               const Mat3& Pv) const {
  Vec6 x;
  x << p, v;
  Mat6 P = Mat6::Zero();
  P.topLeftCorner<3, 3>() = Pp;
  P.bottomRightCorner<3, 3>() = Pv + Mat3::Identity();  // + 1 (m/s)^2: acceleration in between
  c.imm = makeImm();
  c.imm.reset(x, P);
  c.ekf = c.imm.combined();
  c.status = TrackStatus::Track;
  c.t_filter = t;
  c.t_last_update = t;
  c.t_last_radar = t;
  c.cam_consec = 0;
  c.candidates.clear();
  c.tentative = true;  // confirmed after confirm_hits radar plots in a row
  c.hits = 2;          // the two plots that started it
}

void FusionTracker::updateStatus(double now) {
  if (c_.status == TrackStatus::Search) return;
  const double age = now - c_.t_last_update;
  if (age > p_.lost_time || (c_.tentative && age > p_.coast_time)) {
    c_.status = TrackStatus::Search;
    c_.tentative = false;
    c_.cam_consec = 0;
    c_.candidates.clear();
    c_.t_candidates = -1.0;
  } else if (age > p_.coast_time) {
    c_.status = TrackStatus::Coast;
  }
}

Mat3 FusionTracker::cameraPoseNoise(const Vec3& cam_pos, const Mat3& R_cw, const Mat3& P_cam_pos,
                                    double sigma_att) const {
  CameraParams cp = cam_prm_;
  cp.target_size = c_.W;
  const PinholeCamera cam(cp);
  const Vec3 rc = PinholeCamera::toCamera(c_.ekf.position(), cam_pos, R_cw);
  if (!hasEstimate() || rc.z() <= 0.1) return Mat3::Zero();
  const Mat3 H = cam.jacobianPos(rc, R_cw);
  const Mat3 att = Vec3(std::pow(cp.fx * sigma_att, 2), std::pow(cp.fy * sigma_att, 2), 0.0).asDiagonal();
  return H * P_cam_pos * H.transpose() + att;
}

TargetEkf FusionTracker::predictedAt(double t) const {
  if (!(hasEstimate() && std::isfinite(c_.t_filter) && t > c_.t_filter)) return c_.ekf;
  TargetImm imm = c_.imm;
  imm.predict(t - c_.t_filter);
  return imm.combined();
}

}  // namespace follow_me
