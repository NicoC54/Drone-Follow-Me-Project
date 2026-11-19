// Radar + camera target tracker (phase 4: unknown drone, ground radar, camera hand-over).
//
//  - Ground radar plots (range, az, el, range rate), 2 Hz, 0.3 s datalink latency. Each scan
//    contains the target, the follower's OWN echo and clutter: association by nearest neighbour
//    against two known "tracks" (own-ship from the navigation, target from the EKF).
//  - Track start from the radar: two consistent plots on consecutive scans (chi2 test), the
//    initial velocity comes from their difference.
//  - Camera: the target size is unknown, so the camera gives a BEARING (u, v) only... until the
//    size W has been learnt from pairs (apparent size s, depth z of the RADAR plot in the camera
//    frame): W = s * z / f. Careful: z is the camera->target depth, not the radar range (the
//    first version used the radar range and found W = 13 m instead of 0.6 m: unit test).
//    Then the camera gives a full [u, v, s] measurement, i.e. a precise range at short distance.
//  - Out-of-sequence measurements: radar plots arrive 0.3 s late, camera detections ~10-80 ms
//    late. Every measurement is kept for a few seconds with a snapshot of the filter after it;
//    a late one rolls the filter back to the snapshot before its time stamp, then all later
//    measurements are re-fused in time order (exact, unlike dropping or fusing it "now").
#pragma once

#include <deque>
#include <limits>
#include <optional>
#include <variant>
#include <vector>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/ekf.hpp"
#include "follow_me_core/imm.hpp"

#include <string>
#include "follow_me_core/radar_model.hpp"
#include "follow_me_core/track_manager.hpp"

namespace follow_me {

struct FusionParams {
  double q_accel = 3.0;            // manoeuvring unknown drone: larger than the beacon case
  // IMM (Monte-Carlo with random manoeuvres: with q = 3 only, the radar-only estimate during the
  // hand-over was inconsistent in 5 m/s^2 turns, the safety margin in sigmas was wrong and the
  // follower came within 2 m). Calm + manoeuvre models; imm = false -> single model q_accel.
  // "imm_singer" (calm CV + Singer acceleration, default), "imm_cv" (v1: CV q_calm + CV q_cv_manoeuvre),
  // "singer" (single Singer model), "cv" (single constant-velocity model q_accel).
  // Closed-loop comparison (100 runs x 2 levels, 2026-11-18): imm_singer + acceleration
  // feed-forward follows tighter (distance error 0.83 -> 0.66 m) but the follower works HARDER
  // (tilt p95 26 -> 28-30 deg) and the filter is less consistent (NEES 8 -> 11): default imm_cv.
  std::string model = "imm_cv";
  double q_cv_manoeuvre = 40.0;
  // (v1: two constant-velocity models q = 3 / 40. v2: the manoeuvre model is a Singer
  // acceleration model, so that a stopping / starting target is predicted, not lagged.)
  double q_calm = 3.0;                                // calm constant-velocity model [m^2/s^3]
  double q_jerk = 30.0, singer_tau = 3.0;             // manoeuvre model: jerk density [m^2/s^5], tau [s]
  double sojourn_calm = 10.0, sojourn_manoeuvre = 4.0;  // mean time in each model [s]
  double mu0_manoeuvre = 0.3;                         // probability at track start
  bool bearing_keeps_range = true;  // bearing-only camera updates do not touch the range (ekf.hpp)
  double gate_cam_3d = 16.27;      // chi2(3) 99.9 %
  double gate_cam_2d = 13.82;      // chi2(2)
  double gate_radar = 18.47;       // chi2(4) 99.9 %
  double gate_own = 16.27;         // radar plot vs own position, chi2(3)
  double gate_init = 16.27;        // two plots compatible for a track start
  double init_speed_max = 25.0;    // [m/s] plausible target speed between two scans
  double coast_time = 1.2;         // no update for this long -> COAST (radar alone: 0.5 s scans + latency)
  double lost_time = 4.0;          // -> SEARCH
  int lock_count = 5;              // consecutive camera updates to declare the camera "locked"
  double lock_timeout = 0.5;       // camera lock lost after this long without camera update
  double size_prior = 0.5;         // target size prior [m]
  double size_prior_sigma = 0.3;   // [m]
  double size_sigma_rel_ok = 0.08; // size known well enough (8 %) to use the camera range
  double size_process = 0.005;     // size random walk (aspect changes) [m/sqrt(s)]
  double history = 3.0;            // OOSM buffer [s]
  // The apparent size of an unknown drone depends on the resolution: far away its thin arms are
  // sub-pixel and only part of it is seen. Learn W, and use the camera range, only when the
  // target is well resolved (Gazebo phase 4: W learnt at 80 m was half the close-range size).
  double size_min_px = 10.0;
  int confirm_hits = 3;            // radar plots to confirm a tentative track (M-of-N start)
  // The silhouette of a non-spherical drone changes with its orientation: the bounding box of a
  // rotating X shape varies by ~10 %. Relative noise added to the size measurement (Gazebo phase
  // 4: without it, target NEES ~30 instead of ~6).
  // With the second-moment size (D36) the measured spread is 3.6 % frame to frame + 1.7 % slow
  // (Gazebo, physical target): 5 %. (10 % was set for the bounding-box size.)
  double size_rel_noise = 0.05;
  // ...and this aspect error is SLOW (it follows the target's turns), not white: at 30 Hz the
  // filter averaged it away and believed itself 0.3 m accurate with 1-2 m of bias (Gazebo
  // phase 4: target NEES 37). A correlated error of time constant tau is worth one independent
  // measurement every 2 tau: its variance is inflated by 2 tau / dt (dt = time since the last
  // camera range update). 0 = white noise (previous behaviour).
  double size_aspect_tau = 0.0;  // [s]
  // A thresholded silhouette loses its anti-aliased edges: the apparent size is short by a few
  // pixels (Gazebo phase 4: W = s z / f gave 0.47 m at 50 m, 0.79 m at 20 m, 0.98 m at 10 m).
  // Learnt during the hand-over at 30-80 m, W started 18 % short -> range 1.8 m short in FOLLOW.
  // Modelled as an absolute size uncertainty: far, poorly resolved views weigh little.
  double size_res_px = 4.0;
  // Largest plausible target [m]: a detection implying a bigger object at the tracked depth is
  // scenery (trees, roofs on the horizon), not the drone.
  double size_max = 3.0;
  // Camera ignored while the target is expected smaller than this (out of detection range) [px]
  double detectable_px = 2.0;
};

class FusionTracker {
 public:
  FusionTracker(const FusionParams& p, const CameraParams& cam, const GroundRadarParams& radar);

  struct CameraObs {
    Vec3 z = Vec3::Zero();        // [u, v, s] (s = apparent size, same convention as the size W)
    Vec3 cam_pos = Vec3::Zero();
    Mat3 R_cw = Mat3::Identity();
    Mat3 R_extra = Mat3::Zero();  // camera pose (navigation) uncertainty
    bool size_valid = true;
  };
  struct RadarObs {
    std::vector<RadarPlot> plots;
    Vec3 own_pos = Vec3::Zero();  // follower position (navigation) at the scan time
    Mat3 own_P = Mat3::Identity();
    // Site of the radar that took this scan (several radars: trilateration). Empty: the site of
    // the GroundRadarParams given to the tracker. Same noise model for every radar.
    std::optional<Vec3> site;
  };

  // Measurements may arrive in any order: t is the CAPTURE time.
  void onCamera(double t, const CameraObs& obs);
  void onRadar(double t, const RadarObs& obs);
  void updateStatus(double now);

  TargetEkf predictedAt(double t) const;
  // Probability of the manoeuvre model (0 with a single model): "is the target manoeuvring?"
  double manoeuvreProbability() const { return c_.imm.size() > 1 ? c_.imm.mu()[1] : 0.0; }
  // Target acceleration predicted to t (IMM mixture: ~0 while the calm model dominates).
  Vec3 accelerationAt(double t) const;

  // Pixel covariance induced by the camera pose (navigation) uncertainty, at the current estimate.
  Mat3 cameraPoseNoise(const Vec3& cam_pos, const Mat3& R_cw, const Mat3& P_cam_pos,
                       double sigma_att) const;
  TrackStatus status() const { return c_.status; }
  bool hasEstimate() const { return c_.status != TrackStatus::Search && !c_.tentative; }
  bool cameraLocked(double now) const {
    return c_.cam_consec >= p_.lock_count && now - c_.t_last_cam < p_.lock_timeout;
  }
  double sizeEstimate() const { return c_.W; }
  double sizeSigma() const { return std::sqrt(c_.W_var); }
  bool sizeKnown() const { return std::sqrt(c_.W_var) / c_.W < p_.size_sigma_rel_ok; }

  // Statistics (for logs and tests)
  struct Stats {
    int radar_used = 0, radar_own = 0, radar_clutter = 0, cam_used = 0, cam_rejected = 0;
    int replays = 0, replayed_entries = 0;
  };
  Stats stats() const {
    Stats s = c_.stats;
    s.replays = replays_;
    s.replayed_entries = replayed_;
    return s;
  }
  double timeLastCamera() const { return c_.t_last_cam; }
  double timeLastRadar() const { return c_.t_last_radar; }

 private:
  TargetImm makeImm() const;
  // Everything the filter "knows": copied in each snapshot for OOSM replays.
  struct Core {
    TargetEkf ekf{1.0};
    TrackStatus status = TrackStatus::Search;
    double t_filter = -std::numeric_limits<double>::infinity();
    double t_last_update = -std::numeric_limits<double>::infinity();
    double t_last_cam = -std::numeric_limits<double>::infinity();
    double t_last_radar = -std::numeric_limits<double>::infinity();
    int cam_consec = 0;
    bool tentative = false;  // started from 2 plots, waiting for confirmation by the radar
    int hits = 0;
    TargetImm imm;  // the filter; `ekf` below is its combined output, kept for readers
    double W = 0.5, W_var = 0.09, t_W = 0.0;
    double t_last_size_cam = -std::numeric_limits<double>::infinity();  // last camera RANGE update
    struct LastSize {
      double t, s;
      Vec3 cam_pos;
      Mat3 R_cw;
    };
    std::optional<LastSize> last_size;        // latest valid camera size, paired with radar ranges
    std::vector<PolarToCartesian> candidates; // track start: plots of the previous scan
    double t_candidates = -1.0;
    Stats stats;
  };
  struct Entry {
    double t;
    std::variant<CameraObs, RadarObs> obs;
  };

  void add(Entry e);
  void process(Core& c, const Entry& e) const;
  void processCamera(Core& c, double t, const CameraObs& o) const;
  void processRadar(Core& c, double t, const RadarObs& o) const;
  void startTrack(Core& c, double t, const Vec3& p, const Vec3& v, const Mat3& Pp, const Mat3& Pv) const;

  FusionParams p_;
  CameraParams cam_prm_;
  GroundRadarParams radar_prm_;
  Mat4 R_radar_;
  Core c_;                       // current state = snapshot after the last entry
  Core base_;                    // state before the oldest entry of the history
  std::deque<Entry> history_;    // sorted by time
  std::deque<Core> snapshots_;   // snapshots_[i] = state after history_[i]
  int replays_ = 0, replayed_ = 0;
};

}  // namespace follow_me
