#include "follow_me_core/sim_models.hpp"

#include <algorithm>
#include <random>

namespace follow_me {

std::vector<Segment> defaultScenario() {
  return {
      {0.0, 0.0, 0.0, 0.0},      // hover: follower acquires the target
      {3.0, 5.0, 0.0, 0.0},      // straight cruise
      {14.0, 5.0, 20.0, 0.0},    // left turn
      {21.0, 8.0, 0.0, 0.0},     // faster
      {30.0, 8.0, 0.0, 2.0},     // climb
      {36.0, 8.0, -25.0, 0.0},   // S-turn, right...
      {43.0, 6.0, 25.0, 0.0},    // ...then left
      {50.0, 4.0, 0.0, 0.0},     // slow down
      {58.0, 0.0, 0.0, 0.0},     // stop: heading undefined
      {66.0, 7.0, -40.0, -1.0},  // sharp turn + descent from hover
      {72.0, 10.0, 0.0, 0.0},    // dash
      {82.0, 3.0, 0.0, 0.0},     // slow
  };
}

ManeuverParams ManeuverParams::level(int l) {
  ManeuverParams m;
  if (l >= 2) {  // aggressive: short legs, sharp and large turns, hard braking, more stops
    m.v_min = 4.0;
    m.leg_min = 3.0;
    m.leg_max = 8.0;
    m.dpsi_min = 60.0;
    m.dpsi_max = 180.0;
    m.turn_min_deg = 30.0;
    m.turn_max_deg = 70.0;
    m.climb_max = 2.0;
    m.p_hover = 0.15;
    m.max_accel = 5.0;
  }
  return m;
}

std::vector<Segment> randomScenario(std::uint64_t seed, double duration, const ManeuverParams& m) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto uni = [&](double a, double b) { return a + (b - a) * u(rng); };
  std::vector<Segment> s;
  double z = m.z0;  // altitude kept within [z_min, z_max] (dead reckoning of the climb rates)
  double speed = uni(m.v_min, m.v_max);
  auto push = [&](double t, double v, double turn, double len, bool climb) {
    Segment g{t, v, turn, 0.0};
    if (climb && u(rng) < 0.4) {  // some legs climb or descend
      g.climb_rate = uni(-m.climb_max, m.climb_max);
      if (z + g.climb_rate * len < m.z_min || z + g.climb_rate * len > m.z_max) g.climb_rate = -g.climb_rate;
      z += g.climb_rate * len;
    }
    s.push_back(g);
    return t + len;
  };
  for (double t = 0.0; t < duration;) {
    // straight leg
    t = push(t, speed, 0.0, uni(m.leg_min, m.leg_max), true);
    if (t >= duration) break;
    // decisive manoeuvre
    const double r = u(rng);
    if (r < m.p_hover) {  // stop, then go again at a new speed
      t = push(t, 0.0, 0.0, uni(m.hover_min, m.hover_max), false);
      speed = uni(m.v_min, m.v_max);
    } else if (r < m.p_hover + m.p_speed) {  // sprint or brake: the next leg has a new speed
      speed = uni(m.v_min, m.v_max);
    } else {  // turn by a chosen heading change at a chosen rate, then straight again
      const double dpsi = uni(m.dpsi_min, m.dpsi_max), rate = uni(m.turn_min_deg, m.turn_max_deg);
      const double sign = u(rng) < 0.5 ? -1.0 : 1.0;
      if (u(rng) < 0.3) speed = uni(m.v_min, m.v_max);  // sometimes with a speed change
      t = push(t, speed, sign * rate, dpsi / rate, false);
    }
  }
  return s;
}

// ---------------- Leader ----------------
Leader::Leader(const LeaderParams& p)
    : prm_(p), p_(p.start_pos), psi_(deg2rad(p.start_heading_deg)) {}

const Segment& Leader::segmentAt(double t) const {
  const Segment* seg = &prm_.scenario.front();
  for (const auto& s : prm_.scenario)
    if (t >= s.t_start) seg = &s;
  return *seg;
}

void Leader::step(double t, double dt) {
  const Segment& seg = segmentAt(t);
  speed_ += std::clamp(seg.speed - speed_, -prm_.max_accel * dt, prm_.max_accel * dt);
  turn_ += (deg2rad(seg.turn_rate_deg) - turn_) * dt / prm_.tau_turn;
  vz_ += (seg.climb_rate - vz_) * dt / prm_.tau_climb;
  psi_ += turn_ * dt;
  v_ = Vec3(speed_ * std::cos(psi_), speed_ * std::sin(psi_), vz_);
  p_ += v_ * dt;
}

// ---------------- Follower ----------------
FollowerDynamics::FollowerDynamics(const FollowerParams& p, std::uint64_t seed)
    : prm_(p), rng_(seed), p_(p.start_pos) {}

void FollowerDynamics::step(const Vec3& a_cmd, double dt) {
  a_ += (a_cmd - a_) * dt / prm_.tau_accel;
  // Ornstein-Uhlenbeck gust: stationary std = gust_sigma
  const Vec3 w(normal_(rng_), normal_(rng_), normal_(rng_));
  gust_ += -gust_ * dt / prm_.gust_tau + prm_.gust_sigma * std::sqrt(2.0 * dt / prm_.gust_tau) * w;
  const Vec3 acc = a_ - prm_.drag * (v_ - prm_.wind) + gust_;
  v_ += acc * dt;
  p_ += v_ * dt;
}

// ---------------- Gimbal ----------------
void GimbalModel::point(double yaw_des, double pitch_des, double dt) {
  const double rmax = deg2rad(p_.rate_max_deg);
  yaw_ = wrapAngle(yaw_ + std::clamp(p_.gain * wrapAngle(yaw_des - yaw_), -rmax, rmax) * dt);
  pitch_ += std::clamp(p_.gain * (pitch_des - pitch_), -rmax, rmax) * dt;
  pitch_ = std::clamp(pitch_, deg2rad(p_.pitch_min_deg), deg2rad(p_.pitch_max_deg));
}

void GimbalModel::sweep(double dt) {
  point(yaw_ + deg2rad(p_.search_rate_deg) / p_.gain, 0.0, dt);
}

// ---------------- Detector ----------------
SimulatedDetector::Output SimulatedDetector::measure(const Vec3& p_target, const Vec3& cam_pos,
                                                     const Mat3& R_cw) {
  const auto& c = cam_.params();
  if (uniform_(rng_) < p_.p_outlier) {
    const Vec3 z(uniform_(rng_) * c.width, uniform_(rng_) * c.height,
                 c.min_size_px + uniform_(rng_) * (60.0 - c.min_size_px));
    return {z, MeasKind::Outlier};
  }
  const Vec3 rc = PinholeCamera::toCamera(p_target, cam_pos, R_cw);
  const Vec3 z_true = cam_.project(rc);
  if (!cam_.visible(z_true, rc)) return {std::nullopt, MeasKind::NotVisible};
  if (uniform_(rng_) < p_.p_miss) return {std::nullopt, MeasKind::Missed};
  const Vec3 noise(c.sigma_px * normal_(rng_), c.sigma_px * normal_(rng_),
                   c.sigma_size_px * normal_(rng_));
  return {z_true + noise, MeasKind::Detection};
}

}  // namespace follow_me
