#include "follow_me_core/intercept_sim.hpp"
#include "follow_me_core/target_drone.hpp"

#include <cstdio>
#include <deque>
#include <algorithm>
#include <fstream>
#include <vector>
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

// Apparent width of an X quadrotor seen at relative yaw psi, divided by the width seen from
// behind: rotor-tip circle of radius a = 0.594 m plus rotor radius b = 0.14 m (Gazebo model).
double aspectFactor(double psi, double gain) {
  constexpr double a = 0.594, b = 0.14;
  const double g = (a * std::max(std::abs(std::sin(M_PI / 4 + psi)), std::abs(std::cos(M_PI / 4 + psi))) + b) /
                   (a * std::sqrt(0.5) + b);
  return 1.0 + gain * (g - 1.0);
}

double rangeSigma(const TargetEkf& est, const Vec3& p_f) {
  const Vec3 d = est.position() - p_f;
  const Vec3 n = d.norm() > 1e-6 ? Vec3(d.normalized()) : Vec3::UnitX();
  return std::sqrt(std::max(n.dot(est.positionCov() * n), 0.0));
}

GroundRadarParams InterceptConfig::secondRadarSite(const GroundRadarParams& like) {
  // Placed so that its lines of sight cross the first radar's at ~90 deg over the engagement
  // area (first radar at (-60, -40), engagement around (150-400, 40-260)).
  GroundRadarParams r = like;
  r.position = Vec3(450.0, -100.0, 3.0);
  return r;
}

GroundRadarParams InterceptConfig::thirdRadarSite(const GroundRadarParams& like, const std::string& where) {
  GroundRadarParams r = like;
  r.position = where == "center" ? Vec3(230.0, 150.0, 3.0) : Vec3(230.0, 420.0, 3.0);
  return r;
}

InterceptMetrics runInterceptSim(const InterceptConfig& cfg) {
  const SimConfig& b = cfg.base;
  const double dt = b.dt;
  LeaderParams lp = b.leader;
  if (cfg.maneuver_level > 0) {
    const ManeuverParams mp = ManeuverParams::level(cfg.maneuver_level);
    lp.scenario = randomScenario(cfg.maneuver_seed, b.duration, mp);
    lp.max_accel = mp.max_accel;
  }
  TargetDrone leader(lp, b, cfg.target_physics);
  FollowerSim follower(b, cfg.follower_start);
  // Ground radars: the main one + optional extra sites (trilateration). Each has its own scan
  // phase, latency and noise draws; every scan carries the site of the radar that took it.
  std::vector<GroundRadarParams> radar_prm{cfg.radar};
  radar_prm.insert(radar_prm.end(), cfg.extra_radars.begin(), cfg.extra_radars.end());
  std::vector<GroundRadarSim> radars;
  std::vector<double> next_scan;
  for (size_t r = 0; r < radar_prm.size(); ++r) {
    radars.emplace_back(radar_prm[r], b.seed + 21 + 101 * r);
    next_scan.push_back(r / (radar_prm[r].scan_rate * radar_prm.size()));  // interleaved scans
  }
  FusionTracker tracker(cfg.fusion, b.camera, cfg.radar);
  MissionManager mission(cfg.intercept, b.guidance);
  PositionController controller(b.control, b.follower.a_max, b.follower.v_max);
  SafetyFilter safety(b.safety, b.follower.a_max);
  JerkLimiter jerk(-1.0);
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
  double next_frame = 0.0, hold_yaw = 0.0, t_follow_start = -1.0;
  double s_derr2 = 0, s_eerr2 = 0, s_nees = 0, s_hnees = 0;
  std::vector<double> follow_acc, follow_tilt;
  double s_rerr2 = 0, s_rerr2_h = 0;
  int n_radar_phase = 0;
  int n_hand = 0;
  int n_follow_stat = 0, n_follow = 0;
  MissionPhase last_phase = MissionPhase::Wait;
  double last_detection_t = -1.0;

  // ---- optional logs (figures, GIF, web replay) ----
  std::ofstream log, radar_log;
  if (!cfg.out_dir.empty()) {
    log.open(cfg.out_dir + "/intercept.csv");
    radar_log.open(cfg.out_dir + "/radar.csv");
    log << "t,phase,lx,ly,lz,fx,fy,fz,qw,qx,qy,qz,nx,ny,nz,ex,ey,ez,evx,evy,evz,"
           "pxx,pxy,pxz,pyy,pyz,pzz,track,locked,size,size_sigma,dist,ax,ay,az,t_go,"
           "gimbal_yaw,gimbal_pitch,camera_sees,lqw,lqx,lqy,lqz\n";
    radar_log << "t,x,y,z,kind,radar\n";  // kind: 0 clutter, 1 target, 2 own echo (truth label)
  }

  for (int k = 0; k < n_steps; ++k) {
    const double t = k * dt;
    // ---- sensors: radar scan (truth of both drones) and camera frame ----
    for (size_t r = 0; r < radars.size(); ++r) {
      if (t < next_scan[r] - 1e-9) continue;
      next_scan[r] += 1.0 / radar_prm[r].scan_rate;
      RadarMsg msg{t + radar_prm[r].latency, t, {}};
      msg.obs.plots = radars[r].scan(t, {{leader.position(), leader.velocity(), true, false},
                                         {follower.truePosition(), follower.velocity(), false, true}}).plots;
      if (radar_log.is_open())
        for (const auto& pl : msg.obs.plots) {
          const auto c = radarToCartesian(pl, radar_prm[r]);
          radar_log << t << ',' << c.p.x() << ',' << c.p.y() << ',' << c.p.z() << ','
                    << (pl.is_target ? 1 : pl.is_own ? 2 : 0) << ',' << r << '\n';
        }
      msg.obs.own_pos = follower.position();  // what the follower believes (navigation)
      msg.obs.own_P = follower.navPosCov() + 0.01 * Mat3::Identity();
      msg.obs.site = radar_prm[r].position;
      // the datalink delivers in ARRIVAL order (radars with different latencies interleave)
      const auto it = std::upper_bound(radar_link.begin(), radar_link.end(), msg.t_arrival,
                                       [](double ta, const RadarMsg& m2) { return ta < m2.t_arrival; });
      radar_link.insert(it, std::move(msg));
    }
    if (t >= next_frame - 1e-9) {
      next_frame += 1.0 / 30.0;
      const Mat3 R_true = gimbal.R_cw();
      const Vec3 rc = PinholeCamera::toCamera(leader.position(), follower.truePosition(), R_true);
      if (rc.z() > 0.1 && rc.z() < cfg.camera_max_range) {
        Vec3 z = cam.project(rc);
        if (z(0) >= 0 && z(0) < b.camera.width && z(1) >= 0 && z(1) < b.camera.height && z(2) >= 2.0) {
          z(2) *= aspectFactor(leader.heading() - std::atan2(leader.position().y() - follower.truePosition().y(),
                                                             leader.position().x() - follower.truePosition().x()),
                               cfg.aspect_gain);
          z += Vec3(b.camera.sigma_px * n(rng), b.camera.sigma_px * n(rng), b.camera.sigma_size_px * n(rng));
          CamMsg msg{t + 0.02, t, {}};
          msg.obs.z = z;
          msg.obs.cam_pos = follower.position();
          msg.obs.R_cw = follower.believedCameraRotation(R_true);
          msg.obs.R_extra = tracker.cameraPoseNoise(msg.obs.cam_pos, msg.obs.R_cw, follower.navPosCov(),
                                                    follower.navAttSigma());
          cam_link.push_back(std::move(msg));
          last_detection_t = t;
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
                                     est.position(), est.velocity(), tracker.sizeEstimate(),
                                     rangeSigma(est, follower.position()), tracker.accelerationAt(t)}, dt);
    if (out.phase != last_phase) {
      ++m.phase_changes;
      last_phase = out.phase;
    }
    if (out.phase == MissionPhase::Follow && m.t_follow < 0) m.t_follow = t;
    Vec3 a_cmd = controller.compute(follower.position(), follower.velocity(), out.p_des, out.v_des,
                                    dt, out.v_max, out.a_ff);
    if (out.a_max > 0) a_cmd = saturate(a_cmd, out.a_max);
    if (out.jerk_max > 0) {
      jerk.setMax(out.jerk_max);
      a_cmd = jerk.apply(a_cmd, dt);
    } else {
      jerk.reset(a_cmd);  // seamless entry into FOLLOW
    }
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
    if (dist < m.min_distance) {
      m.min_distance = dist;
      m.t_min_distance = t;
      m.phase_min_distance = static_cast<int>(out.phase);
    }
    if (dist < b.safety.d_min) m.time_below_dmin += dt;
    if ((out.phase == MissionPhase::Midcourse || out.phase == MissionPhase::Handover) && has_track) {
      const Vec3 e = est.position() - leader.position();
      s_rerr2 += e.squaredNorm();
      s_rerr2_h += e.head<2>().squaredNorm();
      ++n_radar_phase;
    }
    if (out.phase == MissionPhase::Handover && has_track) {
      Vec6 e;
      e << est.position() - leader.position(), est.velocity() - leader.velocity();
      s_hnees += e.dot(est.P().ldlt().solve(e));
      ++n_hand;
    }
    if (out.phase == MissionPhase::Follow) {
      ++n_follow;
      if (t_follow_start < 0) t_follow_start = t;
      if (t - t_follow_start > settle) {
        ++n_follow_stat;
        s_derr2 += (dist - d_ref) * (dist - d_ref);
        s_eerr2 += (est.position() - leader.position()).squaredNorm();
        Vec6 e;
        e << est.position() - leader.position(), est.velocity() - leader.velocity();
        s_nees += e.dot(est.P().ldlt().solve(e));
        follow_acc.push_back(a_cmd.norm());  // smoothness ("jerks"): commanded acceleration and tilt
        follow_tilt.push_back(
            rad2deg(std::acos(std::clamp(follower.trueAttitude().toRotationMatrix()(2, 2), -1.0, 1.0))));
      }
    } else {
      t_follow_start = -1.0;
    }

    if (log.is_open() && k % 10 == 0) {  // 10 Hz
      const Vec3& L = leader.position();
      const Vec3 F = follower.truePosition(), N = follower.position();
      const Quat q = follower.trueAttitude();
      const Mat3 P = est.positionCov();
      log << t << ',' << static_cast<int>(out.phase) << ',' << L.x() << ',' << L.y() << ',' << L.z() << ','
          << F.x() << ',' << F.y() << ',' << F.z() << ',' << q.w() << ',' << q.x() << ',' << q.y() << ','
          << q.z() << ',' << N.x() << ',' << N.y() << ',' << N.z() << ',' << est.position().x() << ','
          << est.position().y() << ',' << est.position().z() << ',' << est.velocity().x() << ','
          << est.velocity().y() << ',' << est.velocity().z() << ',' << P(0, 0) << ',' << P(0, 1) << ','
          << P(0, 2) << ',' << P(1, 1) << ',' << P(1, 2) << ',' << P(2, 2) << ',' << has_track << ','
          << locked << ',' << tracker.sizeEstimate() << ',' << tracker.sizeSigma() << ',' << dist << ','
          << out.p_aim.x() << ',' << out.p_aim.y() << ',' << out.p_aim.z() << ',' << out.t_go << ','
          << gimbal.yaw() << ',' << gimbal.pitch() << ',' << (t - last_detection_t < 0.1) << ','
          << leader.attitude().w() << ',' << leader.attitude().x() << ',' << leader.attitude().y() << ','
          << leader.attitude().z() << '\n';
    }
    follower.step(a_cmd, yaw_des, dt);
    leader.step(t, dt);
  }

  const double ns = std::max(n_follow_stat, 1);
  m.follow_dist_err_rms = std::sqrt(s_derr2 / ns);
  m.follow_est_err_rms = std::sqrt(s_eerr2 / ns);
  m.follow_nees = s_nees / ns;
  m.target_max_tilt = leader.maxTiltDeg();
  m.handover_nees = s_hnees / std::max(n_hand, 1);
  auto p95 = [](std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(0.95 * (v.size() - 1)), v.end());
    return v[static_cast<size_t>(0.95 * (v.size() - 1))];
  };
  m.radar_phase_err_rms = std::sqrt(s_rerr2 / std::max(n_radar_phase, 1));
  m.radar_phase_err_h = std::sqrt(s_rerr2_h / std::max(n_radar_phase, 1));
  m.radar_phase_err_v = std::sqrt(std::max(s_rerr2 - s_rerr2_h, 0.0) / std::max(n_radar_phase, 1));
  m.follow_acc_p95 = p95(follow_acc);
  m.follow_tilt_p95 = p95(follow_tilt);
  m.follow_pct = 100.0 * n_follow / n_steps;
  m.size_est = tracker.sizeEstimate();
  m.size_sigma = tracker.sizeSigma();
  m.fusion = tracker.stats();
  return m;
}

namespace {
// The draws of run i of a campaign (wind, radar, target, manoeuvres): one RNG stream per run so
// that any run can be replayed alone.
InterceptConfig monteCarloConfig(int i, std::uint64_t seed0, int maneuver_level, double* wind_out,
                                 const ConfigTweak& tweak = {}) {
  std::mt19937_64 rng(seed0 * 7919 + static_cast<std::uint64_t>(i));
  std::uniform_real_distribution<double> u(0.0, 1.0);
  InterceptConfig cfg;
  cfg.base.seed = seed0 + 1000 + static_cast<std::uint64_t>(i);
  const double wind = 6.0 * u(rng), heading = 2.0 * M_PI * u(rng);
  cfg.base.quad.wind = Vec3(wind * std::cos(heading), wind * std::sin(heading), 0.0);
  cfg.radar.latency = 0.1 + 0.5 * u(rng);
  cfg.radar.clutter_per_scan = 4.0 * u(rng);
  cfg.target_size = 0.4 + 0.8 * u(rng);
  cfg.maneuver_level = maneuver_level;
  cfg.maneuver_seed = cfg.base.seed + 7;
  if (tweak) tweak(cfg);
  if (wind_out) *wind_out = wind;
  return cfg;
}
}  // namespace

void runInterceptMonteCarlo(int n, std::uint64_t seed0, const std::string& csv_path, int maneuver_level,
                            const ConfigTweak& tweak) {
  std::ofstream csv(csv_path);
  csv << "run,seed,wind,radar_latency,clutter,target_size,initial_range,t_track,t_lock,t_follow,"
         "min_distance,time_below_dmin,follow_dist_err_rms,follow_est_err_rms,follow_pct,size_est,follow_nees,"
         "t_min_distance,phase_min_distance,handover_nees,follow_acc_p95,follow_tilt_p95,radar_phase_err_rms,radar_phase_err_h,radar_phase_err_v\n";
  for (int i = 0; i < n; ++i) {
    double wind = 0;
    const InterceptConfig cfg = monteCarloConfig(i, seed0, maneuver_level, &wind, tweak);
    const auto m = runInterceptSim(cfg);
    csv << i << ',' << cfg.base.seed << ',' << wind << ',' << cfg.radar.latency << ','
        << cfg.radar.clutter_per_scan << ',' << cfg.target_size << ',' << m.initial_range << ','
        << m.t_track << ',' << m.t_lock << ',' << m.t_follow << ',' << m.min_distance << ','
        << m.time_below_dmin << ',' << m.follow_dist_err_rms << ',' << m.follow_est_err_rms << ','
        << m.follow_pct << ',' << m.size_est << ',' << m.follow_nees << ',' << m.t_min_distance << ','
        << m.phase_min_distance << ',' << m.handover_nees << ',' << m.follow_acc_p95 << ','
        << m.follow_tilt_p95 << ',' << m.radar_phase_err_rms << ',' << m.radar_phase_err_h << ','
        << m.radar_phase_err_v << '\n';
    if ((i + 1) % 25 == 0) std::printf("  %d / %d runs\n", i + 1, n);
  }
}

InterceptMetrics replayMonteCarloRun(int index, std::uint64_t seed0, int maneuver_level, const std::string& out_dir,
                                     InterceptConfig* cfg_out, const ConfigTweak& tweak) {
  InterceptConfig cfg = monteCarloConfig(index, seed0, maneuver_level, nullptr, tweak);
  cfg.out_dir = out_dir;
  if (cfg_out) *cfg_out = cfg;
  return runInterceptSim(cfg);
}

void printInterceptMetrics(const InterceptMetrics& m, double target_size) {
  std::printf("  initial range [m]                %8.1f\n", m.initial_range);
  std::printf("  first radar track at [s]         %8.1f\n", m.t_track);
  std::printf("  camera lock at [s]               %8.1f\n", m.t_lock);
  std::printf("  FOLLOW reached at [s]            %8.1f\n", m.t_follow);
  std::printf("  time in FOLLOW [%%]               %8.1f\n", m.follow_pct);
  std::printf("  distance error RMS in FOLLOW [m] %8.2f\n", m.follow_dist_err_rms);
  std::printf("  target estimate RMSE in FOLLOW   %8.2f\n", m.follow_est_err_rms);
  std::printf("  target NEES in FOLLOW (ideal 6)  %8.2f\n", m.follow_nees);
  std::printf("  target NEES in HANDOVER          %8.2f\n", m.handover_nees);
  std::printf("  estimate error, radar phases     %8.2f m (horizontal %.2f, vertical %.2f)\n",
              m.radar_phase_err_rms, m.radar_phase_err_h, m.radar_phase_err_v);
  std::printf("  FOLLOW smoothness: |a_cmd| p95 %.2f m/s^2, tilt p95 %.1f deg\n", m.follow_acc_p95, m.follow_tilt_p95);
  if (m.target_max_tilt > 0) std::printf("  target max tilt [deg]            %8.1f\n", m.target_max_tilt);
  std::printf("  min distance [m]                 %8.2f  (t = %.1f s, phase %s)\n", m.min_distance,
              m.t_min_distance, toString(static_cast<MissionPhase>(m.phase_min_distance)));
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
