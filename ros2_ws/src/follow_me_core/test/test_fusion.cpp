#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

#include "follow_me_core/fusion_tracker.hpp"

using namespace follow_me;

namespace {
CameraParams gazeboCamera() {
  CameraParams c;
  c.width = 640;
  c.height = 360;
  c.fx = c.fy = 400.0;
  c.sigma_px = 1.0;
  c.sigma_size_px = 0.5;
  c.max_range = 300.0;
  return c;
}

// A synthetic world: the target flies a circle, the follower sits or moves along a line,
// the ground radar scans at 2 Hz (0.3 s latency), the camera at 30 Hz (20 ms latency).
struct World {
  GroundRadarParams rp;
  CameraParams cp = gazeboCamera();
  double W_true = 0.6;  // real target size, UNKNOWN to the tracker (prior 0.5 +/- 0.3)
  Vec3 target(double t) const { return Vec3(250 + 40 * std::cos(0.1 * t), 150 + 40 * std::sin(0.1 * t), 40); }
  Vec3 targetVel(double t) const { return Vec3(-4 * std::sin(0.1 * t), 4 * std::cos(0.1 * t), 0); }
  Vec3 follower(double t, bool close) const {
    return close ? target(t) + Vec3(-15, -5, -3) : Vec3(-50, -35, 20);
  }
};

struct Arrival {
  double t_arrival, t_capture;
  bool radar;
  FusionTracker::RadarObs r;
  FusionTracker::CameraObs c;
};

// Generates every measurement with its capture and arrival times.
std::vector<Arrival> generate(const World& w, double duration, bool close, bool camera,
                              std::uint64_t seed) {
  GroundRadarSim radar(w.rp, seed);
  std::mt19937_64 rng(seed + 1);
  std::normal_distribution<double> n(0.0, 1.0);
  const PinholeCamera cam(w.cp);
  std::vector<Arrival> out;
  for (double t = 0.0; t < duration; t += 0.5) {  // radar scans
    Arrival a{t + w.rp.latency, t, true, {}, {}};
    const Vec3 pf = w.follower(t, close);
    a.r.plots = radar.scan(t, {{w.target(t), w.targetVel(t), true, false},
                               {pf, Vec3::Zero(), false, true}}).plots;
    a.r.own_pos = pf;
    a.r.own_P = 0.04 * Mat3::Identity();
    out.push_back(a);
  }
  if (camera) {
    for (double t = 0.0; t < duration; t += 1.0 / 30.0) {
      const Vec3 pf = w.follower(t, close), pt = w.target(t);
      const auto [yaw, pitch] = lookAt(pf, pt);
      const Mat3 R = rotWorldToCam(yaw, pitch);
      const Vec3 rc = PinholeCamera::toCamera(pt, pf, R);
      const double s = w.cp.fx * w.W_true / rc.z();
      if (s < 2.0) continue;  // too far to be seen
      Arrival a{t + 0.02, t, false, {}, {}};
      a.c.z = Vec3(w.cp.fx * rc.x() / rc.z() + 320 + n(rng), w.cp.fy * rc.y() / rc.z() + 180 + n(rng),
                   s + 0.5 * n(rng));
      a.c.cam_pos = pf;
      a.c.R_cw = R;
      out.push_back(a);
    }
  }
  return out;
}

// Feeds the measurements in ARRIVAL order (late radar plots -> out-of-sequence).
FusionTracker run(const World& w, std::vector<Arrival> arrivals, bool sort_by_capture) {
  std::stable_sort(arrivals.begin(), arrivals.end(), [&](const Arrival& a, const Arrival& b) {
    return sort_by_capture ? a.t_capture < b.t_capture : a.t_arrival < b.t_arrival;
  });
  FusionTracker tr(FusionParams{}, w.cp, w.rp);
  for (const auto& a : arrivals) {
    if (a.radar)
      tr.onRadar(a.t_capture, a.r);
    else
      tr.onCamera(a.t_capture, a.c);
    tr.updateStatus(a.t_arrival);
  }
  return tr;
}
}  // namespace

TEST(FusionTracker, RadarAloneStartsAndKeepsTheTrackDespiteClutterAndOwnEcho) {
  World w;
  w.rp.clutter_per_scan = 2.0;
  const auto tr = run(w, generate(w, 60.0, /*close=*/false, /*camera=*/false, 3), false);
  ASSERT_TRUE(tr.hasEstimate());
  const double err = (tr.predictedAt(60.0).position() - w.target(60.0)).norm();
  EXPECT_LT(err, 4.0);  // radar only, ~350 m from the radar: metres-level accuracy
  EXPECT_GT(tr.stats().radar_used, 90);
  EXPECT_GT(tr.stats().radar_own, 90);  // own echo recognised, never used as the target
}

TEST(FusionTracker, LateRadarPlotsAreFusedExactlyByReplay) {
  World w;
  const auto arrivals = generate(w, 30.0, /*close=*/true, /*camera=*/true, 5);
  const auto in_order = run(w, arrivals, /*sort_by_capture=*/true);   // ideal: no latency
  const auto late = run(w, arrivals, /*sort_by_capture=*/false);      // realistic arrivals
  EXPECT_GT(late.stats().replays, 50);  // every radar scan arrived after newer camera frames
  EXPECT_TRUE(late.predictedAt(30.0).x().isApprox(in_order.predictedAt(30.0).x(), 1e-9));
  EXPECT_NEAR(late.sizeEstimate(), in_order.sizeEstimate(), 1e-12);
}

TEST(FusionTracker, LearnsTheTargetSizeAndThenRangesWithTheCamera) {
  World w;  // true size 0.6 m, prior 0.5 +/- 0.3
  const auto tr = run(w, generate(w, 40.0, /*close=*/true, /*camera=*/true, 7), false);
  EXPECT_TRUE(tr.sizeKnown());
  EXPECT_NEAR(tr.sizeEstimate(), w.W_true, 3.0 * tr.sizeSigma());  // consistent (here ~3 %)
  EXPECT_TRUE(tr.cameraLocked(40.0));
  // With the size known, the camera gives the range. The remaining range error comes from the
  // size error (z * dW/W): the filter must know it (3 sigma), the bearing stays very precise.
  const auto est = tr.predictedAt(40.0);
  const Vec3 e = est.position() - w.target(40.0);
  const Vec3 los = (w.target(40.0) - w.follower(40.0, true)).normalized();
  const double sigma_los = std::sqrt(los.dot(est.positionCov() * los));
  EXPECT_LT(std::abs(e.dot(los)), 3.0 * sigma_los);   // ~0.4 m for sigma ~0.28 m at 16 m
  EXPECT_LT((e - e.dot(los) * los).norm(), 0.15);     // lateral: bearing-limited
}

// Scenery in the image (trees, roofs on the horizon) must not be fused as the target.
// Gazebo phase 4 with scenery: 2 px trees at 480 m, where the drone is 0.4 px, dragged the radar
// track away; a 35 px group of trees (= a 40 m object at that depth) was taken for the target.
TEST(FusionTracker, IgnoresBlobsThatCannotBeTheTarget) {
  World w;
  auto radar_only = generate(w, 30.0, /*close=*/false, /*camera=*/false, 9);
  // (1) Far away (~350 m): the target cannot be seen, any blob is something else.
  auto with_clutter = radar_only;
  const PinholeCamera cam(w.cp);
  for (double t = 0.0; t < 30.0; t += 1.0 / 30.0) {
    const Vec3 pf = w.follower(t, false), pt = w.target(t);
    const auto [yaw, pitch] = lookAt(pf, pt);
    Arrival a{t + 0.02, t, false, {}, {}};
    a.c.z = Vec3(320 + 15, 180 + 12, 2.0);  // a 2 px blob 19 px from the target
    a.c.cam_pos = pf;
    a.c.R_cw = rotWorldToCam(yaw, pitch);
    with_clutter.push_back(a);
  }
  const auto ref = run(w, radar_only, false);
  const auto tr = run(w, with_clutter, false);
  EXPECT_EQ(tr.stats().cam_used, 0);
  // = radar alone. (Not to 1e-9: every camera frame still triggers an IMM prediction step, and
  // IMM mixing is not invariant to how a prediction interval is split.)
  const double sigma = std::sqrt(ref.predictedAt(30.0).positionCov().trace());
  EXPECT_LT((tr.predictedAt(30.0).position() - ref.predictedAt(30.0).position()).norm(), 0.05 * sigma);

  // (2) Close (~16 m): a 250 px blob on the target bearing would be a 10 m object. (The gate is
  // conservative: depth taken 3 sigma closer, so a merely "too big" 4 m blob may pass early on.)
  auto close = generate(w, 30.0, /*close=*/true, /*camera=*/false, 10);
  for (double t = 0.0; t < 30.0; t += 1.0 / 30.0) {
    const Vec3 pf = w.follower(t, true), pt = w.target(t);
    const auto [yaw, pitch] = lookAt(pf, pt);
    Arrival a{t + 0.02, t, false, {}, {}};
    a.c.z = Vec3(320, 180, w.cp.fx * 10.0 / (pt - pf).norm());
    a.c.cam_pos = pf;
    a.c.R_cw = rotWorldToCam(yaw, pitch);
    close.push_back(a);
  }
  const auto tr2 = run(w, close, false);
  EXPECT_EQ(tr2.stats().cam_used, 0);
  EXPECT_GT(tr2.stats().cam_rejected, 500);
}

TEST(TargetImm, SingleModelIsTheEkfAndManoeuvreProbabilityRisesInATurn) {
  Vec6 x0;
  x0 << 0, 0, 40, 8, 0, 0;
  const Mat6 P0 = Mat6::Identity();
  // (1) one model: identical to the plain constant-velocity EKF
  TargetImm one({TargetEkfCA(TargetEkfCA::Model::CV, 3.0)}, {1e9}, {1.0});
  TargetEkf ekf(3.0);
  one.reset(x0, P0);
  ekf.reset(x0, P0);
  for (int k = 0; k < 10; ++k) {
    one.predict(0.1);
    ekf.predict(0.1);
  }
  EXPECT_TRUE(one.combined().x().isApprox(ekf.x(), 1e-12));
  EXPECT_TRUE(one.combined().P().isApprox(ekf.P(), 1e-12));
  // (2) two models, position measurements of a target that flies straight then turns hard
  TargetImm imm({TargetEkfCA(TargetEkfCA::Model::CV, 3.0), TargetEkfCA(TargetEkfCA::Model::Singer, 30.0, 3.0)},
                {10.0, 4.0}, {0.7, 0.3});
  imm.reset(x0, P0);
  std::mt19937_64 rng(3);
  std::normal_distribution<double> n(0.0, 0.3);
  Vec3 p(0, 0, 40), v(8, 0, 0);
  double mu_straight = 0, mu_turn = 0;
  for (int k = 1; k <= 200; ++k) {  // 20 s at 10 Hz; 8 m/s^2 turn from t = 10 s
    const double dt = 0.1;
    if (k > 100) v += dt * Vec3(-v.y(), v.x(), 0).normalized() * 8.0;
    p += dt * v;
    imm.predict(dt);
    const Vec3 z = p + Vec3(n(rng), n(rng), n(rng));
    imm.update([&](TargetEkfCA& m) {
      Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
      H.leftCols<3>().setIdentity();
      const Vec3 y = z - m.position();
      const Mat3 R = 0.09 * Mat3::Identity();
      const double ll = innovationLogLikelihood<3>(y, Mat3(H * m.P() * H.transpose() + R));
      m.updateGeneric<3>(y, H, R, std::numeric_limits<double>::infinity());
      return ll;
    });
    if (k > 50 && k <= 100) mu_straight += imm.mu()[1] / 50.0;
    if (k > 110 && k <= 160) mu_turn += imm.mu()[1] / 50.0;
  }
  std::printf("  manoeuvre probability: straight %.2f, turn %.2f\n", mu_straight, mu_turn);
  EXPECT_LT(mu_straight, 0.2);
  EXPECT_GT(mu_turn, 2.0 * mu_straight + 0.2);  // clearly detects the manoeuvre
}

TEST(TargetEkf, BearingUpdateKeepsTheRange) {
  TargetEkf e(3.0);
  Vec6 x;
  x << 30, 0, 40, 0, 0, 0;
  e.reset(x, 4.0 * Mat6::Identity());
  const Vec3 cam(0, 0, 40);
  const Vec3 los = (e.position() - cam).normalized();
  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  H(0, 1) = 400.0 / 30.0;  // bearing sensitivities of a camera looking along +x
  H(1, 2) = -400.0 / 30.0;
  e.updateBearingKeepRange(Eigen::Vector2d(20.0, -10.0), H, Eigen::Matrix2d::Identity(), los);
  EXPECT_NEAR((e.position() - cam).dot(los), 30.0, 1e-9);   // range untouched
  EXPECT_GT((e.position() - x.head<3>()).norm(), 0.5);      // cross-range corrected
  EXPECT_NEAR(los.dot(e.positionCov() * los), 4.0, 1e-9);   // range variance untouched
}

// A target that brakes from 8 m/s to a hover at 2.5 m/s^2. Measured trade-off (the reason why the
// default IMM keeps two constant-velocity models): a Singer acceleration model SEES the braking,
// but it also "sees" accelerations in the noise while the target flies steadily; the IMM mixture
// stays quiet in steady flight. (Closed loop: feeding this acceleration to the guidance followed
// tighter but made the follower work harder, so it is not used by default.)
TEST(TargetImm, AccelerationModelsTradeBrakingDetectionForSteadyNoise) {
  auto run = [](TargetImm imm) {
    Vec6 x0;
    x0 << 0, 0, 40, 8, 0, 0;
    imm.reset(x0, Mat6::Identity());
    std::mt19937_64 rng(5);
    std::normal_distribution<double> n(0.0, 0.3);
    Vec3 p(0, 0, 40), v(8, 0, 0);
    double a_brake = 0, a2_steady = 0;
    int kb = 0, ks = 0;
    for (int k = 1; k <= 300; ++k) {  // 30 Hz position measurements, braking from t = 5 s
      const double dt = 1.0 / 30.0, t = k * dt;
      if (t > 5.0 && v.x() > 0) v.x() = std::max(0.0, v.x() - 2.5 * dt);
      p += dt * v;
      imm.predict(dt);
      const Vec3 z = p + Vec3(n(rng), n(rng), n(rng));
      imm.update([&](TargetEkfCA& m) {
        Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
        H.leftCols<3>().setIdentity();
        const Mat3 R = 0.09 * Mat3::Identity();
        const Vec3 y = z - m.position();
        const double ll = innovationLogLikelihood<3>(y, Mat3(H * m.P() * H.transpose() + R));
        m.updateGeneric<3>(y, H, R, std::numeric_limits<double>::infinity());
        return ll;
      });
      const auto c = imm.combined9();
      if (t > 5.5 && t < 7.5) a_brake += c.acceleration().x(), ++kb;
      if (t > 2.0 && t < 5.0) a2_steady += c.acceleration().squaredNorm(), ++ks;
    }
    return std::pair<double, double>{a_brake / kb, std::sqrt(a2_steady / ks)};
  };
  using M = TargetEkfCA::Model;
  const auto singer = run(TargetImm({TargetEkfCA(M::Singer, 10.0, 3.0)}, {1e9}, {1.0}));
  const auto mix = run(TargetImm({TargetEkfCA(M::CV, 3.0), TargetEkfCA(M::Singer, 10.0, 3.0)}, {10.0, 4.0},
                                 {0.7, 0.3}));
  std::printf("  braking -2.5 m/s^2 seen as: Singer %.2f, IMM %.2f | steady-flight |a| rms: Singer %.2f, IMM %.2f\n",
              singer.first, mix.first, singer.second, mix.second);
  EXPECT_LT(singer.first, -1.5);            // the Singer model sees the braking...
  EXPECT_GT(singer.second, 1.0);            // ...and noise as acceleration in steady flight
  EXPECT_LT(mix.second, 0.5);               // the IMM stays quiet in steady flight...
  EXPECT_GT(mix.first, singer.first);       // ...and sees less of the braking
}

// Trilateration: a second ground radar whose lines of sight cross the first one's turns its
// precise RANGES into cross-range information. Radar-only track of a target ~350-400 m away.
TEST(FusionTracker, ASecondRadarSiteCutsTheCrossRangeError) {
  auto run = [](bool two) {
    World w;
    GroundRadarParams r2 = w.rp;
    r2.position = Vec3(450.0, -100.0, 3.0);
    GroundRadarSim sim1(w.rp, 11), sim2(r2, 12);
    FusionTracker tr(FusionParams{}, w.cp, w.rp);
    double s2 = 0;
    int n = 0;
    for (double t = 0.0; t < 40.0; t += 0.25) {  // interleaved scans, 2 Hz each
      const bool second = two && std::fmod(t, 0.5) > 0.1;
      if (!two && std::fmod(t, 0.5) > 0.1) continue;
      FusionTracker::RadarObs o;
      const Vec3 pf = w.follower(t, false);
      o.plots = (second ? sim2 : sim1).scan(t, {{w.target(t), w.targetVel(t), true, false},
                                                {pf, Vec3::Zero(), false, true}}).plots;
      o.own_pos = pf;
      o.own_P = 0.04 * Mat3::Identity();
      o.site = second ? r2.position : w.rp.position;
      tr.onRadar(t, o);
      tr.updateStatus(t);
      if (t > 10.0 && tr.hasEstimate()) {
        s2 += (tr.predictedAt(t).position() - w.target(t)).head<2>().squaredNorm();
        ++n;
      }
    }
    return std::sqrt(s2 / n);
  };
  const double one = run(false), two = run(true);
  std::printf("  horizontal error, radar only: 1 radar %.2f m, 2 radars %.2f m\n", one, two);
  EXPECT_LT(two, 0.6 * one);
}
