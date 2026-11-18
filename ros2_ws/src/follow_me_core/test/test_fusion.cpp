#include <gtest/gtest.h>

#include <algorithm>
#include <random>

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
  EXPECT_TRUE(tr.predictedAt(30.0).x().isApprox(ref.predictedAt(30.0).x(), 1e-9));  // = radar alone

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
