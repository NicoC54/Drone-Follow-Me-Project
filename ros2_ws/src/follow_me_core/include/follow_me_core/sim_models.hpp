// Simulation models ("truth" side): leader kinematics, follower dynamics, gimbal, detector.
// Used by the offline simulator, and by the ROS nodes that drive Gazebo.
#pragma once

#include <optional>
#include <random>

#include "follow_me_core/camera_model.hpp"
#include "follow_me_core/params.hpp"

namespace follow_me {

// Kinematic aircraft flying a list of (speed, turn rate, climb rate) segments.
class Leader {
 public:
  explicit Leader(const LeaderParams& p);
  void step(double t, double dt);

  const Vec3& position() const { return p_; }
  const Vec3& velocity() const { return v_; }
  double heading() const { return psi_; }
  double speed() const { return speed_; }
  double turnRate() const { return turn_; }  // [rad/s]
  double climbRate() const { return vz_; }

 private:
  const Segment& segmentAt(double t) const;
  LeaderParams prm_;
  Vec3 p_;
  Vec3 v_ = Vec3::Zero();
  double psi_, speed_ = 0.0, turn_ = 0.0, vz_ = 0.0;
};

// 3D point mass driven by an acceleration command (gravity already compensated).
// Attitude loop = first-order lag; disturbances = drag on airspeed + Ornstein-Uhlenbeck gusts.
class FollowerDynamics {
 public:
  FollowerDynamics(const FollowerParams& p, std::uint64_t seed);
  void step(const Vec3& a_cmd, double dt);

  const Vec3& position() const { return p_; }
  const Vec3& velocity() const { return v_; }

 private:
  FollowerParams prm_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Vec3 p_, v_ = Vec3::Zero(), a_ = Vec3::Zero(), gust_ = Vec3::Zero();
};

// Stabilised 2-axis gimbal, world-frame angles, proportional + rate limited.
class GimbalModel {
 public:
  explicit GimbalModel(const GimbalParams& p, double yaw = 0.0, double pitch = 0.0)
      : p_(p), yaw_(yaw), pitch_(pitch) {}
  void point(double yaw_des, double pitch_des, double dt);
  void sweep(double dt);  // search pattern: rotate in yaw, pitch back to the horizon
  double yaw() const { return yaw_; }
  double pitch() const { return pitch_; }
  Mat3 R_cw() const { return rotWorldToCam(yaw_, pitch_); }

 private:
  GimbalParams p_;
  double yaw_, pitch_;
};

enum class MeasKind { NotVisible = 0, Missed = 1, Detection = 2, Outlier = 3 };

// Synthetic detector: noisy pixel + size, missed detections, false detections.
class SimulatedDetector {
 public:
  SimulatedDetector(const PinholeCamera& cam, const DetectorParams& p, std::uint64_t seed)
      : cam_(cam), p_(p), rng_(seed) {}

  struct Output {
    std::optional<Vec3> z;
    MeasKind kind;
  };
  Output measure(const Vec3& p_target, const Vec3& cam_pos, const Mat3& R_cw);

 private:
  PinholeCamera cam_;
  DetectorParams p_;
  std::mt19937_64 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::normal_distribution<double> normal_{0.0, 1.0};
};

}  // namespace follow_me
