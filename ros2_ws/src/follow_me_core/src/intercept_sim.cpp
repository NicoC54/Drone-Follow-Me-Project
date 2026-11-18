#include "follow_me_core/intercept_sim.hpp"

#include <cstdio>
#include <deque>
#include <random>

#include "follow_me_core/controller.hpp"
#include "follow_me_core/follower_sim.hpp"

namespace follow_me {

InterceptConfig::InterceptConfig() {
  // An unknown drone appears ~450 m away and flies a manoeuvring path.
  base.leader.start_pos = Vec3(330.0, 260.0, 40.0);
  base.leader.start_heading_deg = 200.0;
  base.leader.scenario = {
      {0.0, 8.0, 0.0, 0.0},     {20.0, 8.0, 15.0, 0.0},   {28.0, 9.0, 0.0, 0.5},
      {40.0, 9.0, -20.0, 0.0},  {48.0, 6.0, 0.0, 0.0},    {60.0, 0.0, 0.0, 0.0},
      {68.0, 7.0, 30.0, -0.5},  {76.0, 10.0, 0.0, 0.0},   {90.0, 5.0, 0.0, 0.0},
  };
  base.duration = 120.0;
  // Gazebo-like camera; noise of the real detector.
  base.camera.width = 640;
  base.camera.height = 360;
  base.camera.fx = base.camera.fy = 400.0;
  base.camera.sigma_px = 1.0;
  base.camera.sigma_size_px = 0.5;
  base.camera.max_range = 300.0;
  // Follow BELOW the target: its silhouette stands out against the sky (no colour beacon).
  base.guidance.height_offset = -2.0;
}

InterceptMetrics runInterceptSim(const InterceptConfig& cfg) {
  const SimConfig& b = cfg.base;
  const double dt = b.dt;
  Leader leader(b.leader);
  FollowerSim follower(b, cfg.follower_start);
  GroundRadarSim radar(cfg.radar, b.seed + 21);
  FusionTracker tracker(cfg.fusion, b.camera, cfg.radar);
  MissionManager mission(cfg.intercept, b.guidance);
  PositionController controller(b.control, b.follower.a_max, b.follower.v_max);
  SafetyFilter safety(b.safety, b.follower.a_max);
  GimbalModel gimbal(b.gimbal);
  CameraParams true_cam = b.camera;
  true_cam.target_size = cfg.target_size;  // the world uses the TRUE size
  const PinholeCamera cam(true_cam);
  std::mt19937_64 rng(b.seed + 5);
  std::normal_distribution<double> n(0.0, 1.0);

  struct RadarMsg { double t_arrival, t_capture; FusionTracker::RadarObs obs; };
  struct CamMsg { double t_arrival, t_capture; FusionTracker::CameraObs obs; };
  std::deque<RadarMsg> radar_link;  // datalink: 0.3 s
  std::deque<CamMsg> cam_link;      // image processing: 20 ms

  InterceptMetrics m;
  m.initial_range = (follower.truePosition() - leader.position()).norm();
  const int n_steps = static_cast<int>(std::lround(b.duration / dt));
  const double settle = 5.0, d_ref = std::hypot(b.guidance.follow_dist, b.guidance.height_offset);
  double next_scan = 0.0, next_frame = 0.0, hold_yaw = 0.0, t_follow_start = -1.0;
  double s_derr2 = 0, s_eerr2 = 0;
  int n_follow_stat = 0, n_follow = 0;
  MissionPhase last_phase = MissionPhase::Wait;

  for (int k = 0; k < n_steps; ++k) {
    const double t = k * dt;
    // ---- sensors: radar scan (truth of both drones) and camera frame ----
    if (t >= next_scan - 1e-9) {
      next_scan += 1.0 / cfg.radar.scan_rate;
      RadarMsg msg{t + cfg.radar.latency, t, {}};
      msg.obs.plots = radar.scan(t, {{leader.position(), leader.velocity(), true, false},
                                     {follower.truePosition(), follower.velocity(), false, true}}).plots;
      msg.obs.own_pos = follower.position();  // what the follower believes (navigation)
      msg.obs.own_P = follower.navPosCov() + 0.01 * Mat3::Identity();
      radar_link.push_back(std::move(msg));
    }
    if (t >= next_frame - 1e-9) {
      next_frame += 1.0 / 30.0;
      const Mat3 R_true = gimbal.R_cw();
      const Vec3 rc = PinholeCamera::toCamera(leader.position(), follower.truePosition(), R_true);
      if (rc.z() > 0.1 && rc.z() < cfg.camera_max_range) {
        Vec3 z = cam.project(rc);
        if (z(0) >= 0 && z(0) < b.camera.width && z(1) >= 0 && z(1) < b.camera.height && z(2) >= 2.0) {
          z += Vec3(b.camera.sigma_px * n(rng), b.camera.sigma_px * n(rng), b.camera.sigma_size_px * n(rng));
          CamMsg msg{t + 0.02, t, {}};
          msg.obs.z = z;
          msg.obs.cam_pos = follower.position();
          msg.obs.R_cw = follower.believedCameraRotation(R_true);
          msg.obs.R_extra = tracker.cameraPoseNoise(msg.obs.cam_pos, msg.obs.R_cw, follower.navPosCov(),
                                                    follower.navAttSigma());
          cam_link.push_back(std::move(msg));
        }
      }
    }
    // ---- arrivals (radar plots are late: out-of-sequence fusion) ----
    while (!cam_link.empty() && cam_link.front().t_arrival <= t + 1e-9) {
      tracker.onCamera(cam_link.front().t_capture, cam_link.front().obs);
      cam_link.pop_front();
    }
    while (!radar_link.empty() && radar_link.front().t_arrival <= t + 1e-9) {
      tracker.onRadar(radar_link.front().t_capture, radar_link.front().obs);
      radar_link.pop_front();
    }
    tracker.updateStatus(t);
    const TargetEkf est = tracker.predictedAt(t);
    const bool has_track = tracker.hasEstimate();
    const bool locked = tracker.cameraLocked(t) && tracker.sizeKnown();  // camera can range
    if (has_track && m.t_track < 0) m.t_track = t;
    if (locked && m.t_lock < 0) m.t_lock = t;

    // ---- mission + control ----
    const auto out = mission.update({t, follower.position(), follower.velocity(), has_track, locked,
                                     est.position(), est.velocity()}, dt);
    if (out.phase != last_phase) {
      ++m.phase_changes;
      last_phase = out.phase;
    }
    if (out.phase == MissionPhase::Follow && m.t_follow < 0) m.t_follow = t;
    Vec3 a_cmd = controller.compute(follower.position(), follower.velocity(), out.p_des, out.v_des,
                                    dt, out.v_max);
    if (has_track) {
      a_cmd = safety.filter(a_cmd, follower.position(), follower.velocity(), est.position(),
                            est.velocity(), est.positionCov()).a;
      const auto [yaw_g, pitch_g] = lookAt(follower.truePosition(), est.position());
      gimbal.point(yaw_g, pitch_g, dt);  // cueing: the radar track points the camera
    } else {
      gimbal.sweep(dt);
    }
    a_cmd = saturate(a_cmd, b.follower.a_max);
    double yaw_des = hold_yaw;
    if (has_track) {
      yaw_des = std::atan2(est.position().y() - follower.position().y(),
                           est.position().x() - follower.position().x());
      hold_yaw = yaw_des;
    }

    // ---- metrics (truth) ----
    const double dist = (follower.truePosition() - leader.position()).norm();
    m.min_distance = std::min(m.min_distance, dist);
    if (dist < b.safety.d_min) m.time_below_dmin += dt;
    if (out.phase == MissionPhase::Follow) {
      ++n_follow;
      if (t_follow_start < 0) t_follow_start = t;
      if (t - t_follow_start > settle) {
        ++n_follow_stat;
        s_derr2 += (dist - d_ref) * (dist - d_ref);
        s_eerr2 += (est.position() - leader.position()).squaredNorm();
      }
    } else {
      t_follow_start = -1.0;
    }

    follower.step(a_cmd, yaw_des, dt);
    leader.step(t, dt);
  }

  const double ns = std::max(n_follow_stat, 1);
  m.follow_dist_err_rms = std::sqrt(s_derr2 / ns);
  m.follow_est_err_rms = std::sqrt(s_eerr2 / ns);
  m.follow_pct = 100.0 * n_follow / n_steps;
  m.size_est = tracker.sizeEstimate();
  m.size_sigma = tracker.sizeSigma();
  m.fusion = tracker.stats();
  return m;
}

void printInterceptMetrics(const InterceptMetrics& m, double target_size) {
  std::printf("  initial range [m]                %8.1f\n", m.initial_range);
  std::printf("  first radar track at [s]         %8.1f\n", m.t_track);
  std::printf("  camera lock at [s]               %8.1f\n", m.t_lock);
  std::printf("  FOLLOW reached at [s]            %8.1f\n", m.t_follow);
  std::printf("  time in FOLLOW [%%]               %8.1f\n", m.follow_pct);
  std::printf("  distance error RMS in FOLLOW [m] %8.2f\n", m.follow_dist_err_rms);
  std::printf("  target estimate RMSE in FOLLOW   %8.2f\n", m.follow_est_err_rms);
  std::printf("  min distance [m]                 %8.2f\n", m.min_distance);
  std::printf("  time below d_min [s]             %8.2f\n", m.time_below_dmin);
  std::printf("  size learnt [m]                  %5.3f +/- %.3f (true %.2f)\n", m.size_est,
              m.size_sigma, target_size);
  std::printf("  radar plots: target %d, own echo %d, clutter %d\n", m.fusion.radar_used,
              m.fusion.radar_own, m.fusion.radar_clutter);
  std::printf("  camera updates %d (rejected %d), OOSM replays %d\n", m.fusion.cam_used,
              m.fusion.cam_rejected, m.fusion.replays);
  std::printf("  mission phase changes            %8d\n", m.phase_changes);
}

}  // namespace follow_me
