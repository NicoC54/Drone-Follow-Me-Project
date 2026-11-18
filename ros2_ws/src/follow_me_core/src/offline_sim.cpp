#include "follow_me_core/offline_sim.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>

#include "follow_me_core/attitude_control.hpp"
#include "follow_me_core/controller.hpp"
#include "follow_me_core/guidance.hpp"
#include "follow_me_core/follower_sim.hpp"
#include "follow_me_core/sim_models.hpp"
#include "follow_me_core/timed_tracker.hpp"

#include <deque>

namespace follow_me {

SimMetrics runOfflineSim(const SimConfig& cfg) {
  const double dt = cfg.dt;
  Leader leader(cfg.leader);
  FollowerSim follower(cfg);
  PinholeCamera cam(cfg.camera);
  GimbalModel gimbal(cfg.gimbal);
  SimulatedDetector detector(cam, cfg.detector, cfg.seed + 1);
  TimedTracker tracker(cfg.estimator, cam);
  struct Pending {
    double t_capture, t_arrival;
    Vec3 z, cam_pos;
    Mat3 R_cw, P_cam;
    double sigma_att;
    bool outlier;
  };
  std::deque<Pending> in_flight;  // detections travelling from the camera to the tracker
  FollowGuidance guidance(cfg.guidance);
  PositionController controller(cfg.control, cfg.follower.a_max, cfg.follower.v_max);
  SafetyFilter safety(cfg.safety, cfg.follower.a_max);

  std::ofstream csv;
  if (!cfg.csv_path.empty()) {
    csv.open(cfg.csv_path);
    csv << "t,status,lx,ly,lz,fx,fy,fz,ex,ey,ez,evx,evy,evz,dist,safety\n";
  }

  const int n_steps = static_cast<int>(std::lround(cfg.duration / dt));
  const double cam_period = 1.0 / cfg.detector.rate_hz;
  const double settle_time = 5.0;
  const double d_ref = std::hypot(cfg.guidance.follow_dist, cfg.guidance.height_offset);
  double next_cam_t = 0.0;
  bool holding = false;
  Vec3 hold_pos = Vec3::Zero();
  double hold_yaw = 0.0;

  SimMetrics m;
  int metrics_outliers_total = 0, metrics_outliers_rejected = 0;
  double s_ep2 = 0, s_ev2 = 0, s_nees = 0, s_derr2 = 0;
  int n_steady = 0, n_in_frame = 0, n_valid = 0, n_safety = 0;

  for (int k = 0; k < n_steps; ++k) {
    const double t = k * dt;
    const Vec3 cam_true = follower.truePosition();  // where the camera really is (detector)
    const Vec3 cam_nav = follower.position();        // where the GNC believes it is (tracker)

    // ---- 1. Estimation ----
    // Capture: the detector sees the scene NOW, the result arrives `detection_latency` later.
    if (t >= next_cam_t - 1e-9) {
      next_cam_t += cam_period;
      const Mat3 R_cw = gimbal.R_cw();
      auto out = detector.measure(leader.position(), cam_true, R_cw);
      for (const auto& [t0, t1] : cfg.occlusions)
        if (t >= t0 && t < t1) out = {std::nullopt, MeasKind::Missed};
      if (out.z)
        in_flight.push_back({t, t + cfg.detection_latency, *out.z, cam_nav,
                             follower.believedCameraRotation(R_cw), follower.navPosCov(),
                             follower.navAttSigma(), out.kind == MeasKind::Outlier});
    }
    // Arrival: fuse the detections whose latency has elapsed.
    while (!in_flight.empty() && in_flight.front().t_arrival <= t + 1e-9) {
      const Pending d = in_flight.front();
      in_flight.pop_front();
      const bool accepted =
          cfg.naive_latency
              ? tracker.onMeasurement(d.z, t, cam_nav, follower.believedCameraRotation(gimbal.R_cw()))  // pose of arrival: wrong
              : tracker.onMeasurement(d.z, d.t_capture, d.cam_pos, d.R_cw,  // pose of capture
                                      cfg.nav_noise_in_tracker
                                          ? tracker.cameraPoseNoise(d.cam_pos, d.R_cw, d.P_cam, d.sigma_att)
                                          : Mat3::Zero());
      if (d.outlier) {
        ++metrics_outliers_total;
        if (!accepted) ++metrics_outliers_rejected;
      }
    }
    tracker.updateStatus(t);
    const TargetEkf est = tracker.predictedAt(t);  // estimate brought to the current time

    // ---- 2. Guidance + gimbal ----
    Vec3 p_des, v_des = Vec3::Zero();
    const bool has_est = tracker.hasEstimate();
    const Vec3 p_t = est.position(), v_t = est.velocity();
    if (has_est) {
      const auto sp = guidance.compute(follower.position(), p_t, v_t, dt);
      p_des = sp.p;
      v_des = sp.v;
      holding = false;
      const auto [yaw, pitch] = lookAt(follower.position(), p_t);
      gimbal.point(yaw, pitch, dt);
    } else {
      if (!holding) {
        hold_pos = follower.position();
        hold_yaw = follower.yaw();
        holding = true;
        guidance.reset();
      }
      p_des = hold_pos;
      gimbal.sweep(dt);
    }

    // ---- 3. Control + safety ----
    Vec3 a_cmd = controller.compute(follower.position(), follower.velocity(), p_des, v_des, dt);
    bool safety_active = false;
    if (has_est) {
      const auto r = safety.filter(a_cmd, follower.position(), follower.velocity(), p_t, v_t,
                                   est.positionCov());
      a_cmd = r.a;
      safety_active = r.active;
    }
    a_cmd = saturate(a_cmd, cfg.follower.a_max);

    // ---- metrics (state at time t) ----
    const Vec3 rel = follower.truePosition() - leader.position();
    const double dist = rel.norm();
    m.min_distance = std::min(m.min_distance, dist);
    if (dist < cfg.safety.d_min) m.time_below_dmin += dt;
    const Vec3 rc = PinholeCamera::toCamera(leader.position(), cam_true, gimbal.R_cw());
    if (rc.z() > 0 && cam.visible(cam.project(rc), rc)) ++n_in_frame;
    if (has_est) ++n_valid;
    if (safety_active) ++n_safety;
    Vec6 err;
    err << p_t - leader.position(), v_t - leader.velocity();
    if (has_est && t >= settle_time) {
      ++n_steady;
      s_ep2 += err.head<3>().squaredNorm();
      s_ev2 += err.tail<3>().squaredNorm();
      s_nees += err.dot(est.P().ldlt().solve(err));
      s_derr2 += (dist - d_ref) * (dist - d_ref);
    }
    if (csv.is_open()) {
      const auto& L = leader.position();
      const auto& F = follower.position();
      csv << t << ',' << static_cast<int>(tracker.status()) << ',' << L.x() << ',' << L.y() << ','
          << L.z() << ',' << F.x() << ',' << F.y() << ',' << F.z();
      for (int i = 0; i < 6; ++i) csv << ',' << (has_est ? err(i) : NAN);
      csv << ',' << dist << ',' << safety_active << '\n';
    }

    // ---- 4. Propagate the truth (quadrotor: inner attitude loops at 500 Hz) ----
    // Heading: face the leader, so the gimbal stays near its neutral yaw.
    const double yaw_des = has_est ? std::atan2(p_t.y() - follower.position().y(),
                                                p_t.x() - follower.position().x())
                                   : hold_yaw;
    follower.step(a_cmd, yaw_des, dt);
    leader.step(t, dt);
  }

  m.max_tilt_deg = follower.maxTiltDeg();
  m.mixer_sat_pct = follower.saturationPct();
  m.nav_pos_rmse = follower.navPosRmse();
  m.nav_att_rmse_deg = follower.navAttRmseDeg();
  m.outliers_total = metrics_outliers_total;
  m.outliers_rejected = metrics_outliers_rejected;
  const double n = std::max(n_steady, 1);
  m.est_pos_rmse = std::sqrt(s_ep2 / n);
  m.est_vel_rmse = std::sqrt(s_ev2 / n);
  m.mean_nees = s_nees / n;
  m.dist_err_rms = std::sqrt(s_derr2 / n);
  m.in_frame_pct = 100.0 * n_in_frame / n_steps;
  m.valid_track_pct = 100.0 * n_valid / n_steps;
  m.safety_active_pct = 100.0 * n_safety / n_steps;
  return m;
}

void printMetrics(const SimMetrics& m) {
  std::printf("  estimation position RMSE [m]     %8.2f\n", m.est_pos_rmse);
  std::printf("  estimation velocity RMSE [m/s]   %8.2f\n", m.est_vel_rmse);
  std::printf("  mean NEES (ideal 6)              %8.2f\n", m.mean_nees);
  std::printf("  distance error RMS [m]           %8.2f\n", m.dist_err_rms);
  std::printf("  min distance [m]                 %8.2f\n", m.min_distance);
  std::printf("  time below d_min [s]             %8.2f\n", m.time_below_dmin);
  std::printf("  time leader in frame [%%]         %8.2f\n", m.in_frame_pct);
  std::printf("  time with valid track [%%]        %8.2f\n", m.valid_track_pct);
  std::printf("  safety filter active [%%]         %8.2f\n", m.safety_active_pct);
  std::printf("  outliers rejected                %5d/%d\n", m.outliers_rejected, m.outliers_total);
  std::printf("  max tilt [deg]                   %8.1f\n", m.max_tilt_deg);
  std::printf("  mixer saturated [%%]              %8.2f\n", m.mixer_sat_pct);
  std::printf("  navigation position RMSE [m]     %8.2f\n", m.nav_pos_rmse);
  std::printf("  navigation attitude RMSE [deg]   %8.2f\n", m.nav_att_rmse_deg);
}

}  // namespace follow_me
