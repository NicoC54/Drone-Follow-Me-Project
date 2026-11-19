// Command line:  follow_me_offline_sim [--seed N] [--duration T] [--occlusion T0 T1]...
//                                      [--latency S] [--naive] [--point-mass] [--perfect-nav] [--no-nav-noise] [--intercept [--out DIR] [--montecarlo N] [--aspect G] [--maneuvers L] [--mc-run I] [--no-imm] [--kinematic-target] [--k-sigma K] [--buffer B] [--set key=value]...] [--csv FILE]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "follow_me_core/intercept_sim.hpp"
#include "follow_me_core/offline_sim.hpp"

int main(int argc, char** argv) {
  follow_me::SimConfig cfg;
  bool intercept = false;
  int montecarlo = 0;
  std::string out_dir;
  double aspect_gain = -1.0;  // < 0: keep the InterceptConfig default
  int maneuvers = 0;
  int mc_run = -1;
  bool imm = true;
  bool target_physics = true;
  double k_sigma = -1.0, buffer = -1.0;
  std::vector<std::pair<std::string, std::string>> sets;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--seed" && i + 1 < argc) {
      cfg.seed = std::strtoull(argv[++i], nullptr, 10);
    } else if (a == "--duration" && i + 1 < argc) {
      cfg.duration = std::atof(argv[++i]);
    } else if (a == "--occlusion" && i + 2 < argc) {
      const double t0 = std::atof(argv[++i]);
      const double t1 = std::atof(argv[++i]);
      cfg.occlusions.emplace_back(t0, t1);
    } else if (a == "--latency" && i + 1 < argc) {
      cfg.detection_latency = std::atof(argv[++i]);
    } else if (a == "--intercept") {
      intercept = true;
    } else if (a == "--out" && i + 1 < argc) {
      out_dir = argv[++i];  // intercept: write intercept.csv + radar.csv for figures / web replay
    } else if (a == "--montecarlo" && i + 1 < argc) {
      montecarlo = std::atoi(argv[++i]);  // intercept: N randomised runs -> <out>/montecarlo.csv
    } else if (a == "--aspect" && i + 1 < argc) {
      aspect_gain = std::atof(argv[++i]);  // intercept: target aspect effect (0 = round target)
    } else if (a == "--maneuvers" && i + 1 < argc) {
      maneuvers = std::atoi(argv[++i]);  // intercept: 0 scripted, 1 random moderate, 2 random aggressive
    } else if (a == "--mc-run" && i + 1 < argc) {
      mc_run = std::atoi(argv[++i]);  // intercept: replay one run of the campaign (--seed, --maneuvers)
    } else if (a == "--set" && i + 1 < argc) {
      const std::string kv = argv[++i];
      const auto eq = kv.find('=');
      if (eq == std::string::npos) { std::fprintf(stderr, "--set expects key=value\n"); return 1; }
      sets.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
    } else if (a == "--k-sigma" && i + 1 < argc) {
      k_sigma = std::atof(argv[++i]);  // safety margin in standard deviations
    } else if (a == "--buffer" && i + 1 < argc) {
      buffer = std::atof(argv[++i]);   // safety buffer zone [m]
    } else if (a == "--kinematic-target") {
      target_physics = false;  // intercept: the target follows its scenario exactly (no dynamics)
    } else if (a == "--no-imm") {
      imm = false;  // intercept: single constant-velocity model instead of the IMM
    } else if (a == "--no-nav-noise") {
      cfg.nav_noise_in_tracker = false;
    } else if (a == "--perfect-nav") {
      cfg.use_nav = false;
    } else if (a == "--point-mass") {
      cfg.quadrotor = false;
    } else if (a == "--naive") {
      cfg.naive_latency = true;
    } else if (a == "--csv" && i + 1 < argc) {
      cfg.csv_path = argv[++i];
    } else {
      std::fprintf(stderr,
                   "usage: %s [--seed N] [--duration T] [--occlusion T0 T1]... [--latency S] [--naive] [--point-mass] [--perfect-nav] [--no-nav-noise] [--intercept [--out DIR] [--montecarlo N] [--aspect G] [--maneuvers L] [--mc-run I] [--no-imm] [--kinematic-target] [--k-sigma K] [--buffer B] [--set key=value]...] [--csv FILE]\n",
                   argv[0]);
      return 1;
    }
  }
  // Experiments: --no-imm and --set key=value apply to single runs, replays and campaigns alike.
  for (const auto& kv : sets) {
    static const char* known[] = {"fusion.model", "fusion.q_calm", "fusion.q_jerk", "fusion.singer_tau",
                                  "fusion.q_accel", "intercept.a_ff_gain", "intercept.follow_a_max",
                                  "intercept.follow_jerk_max", "intercept.transition_jerk",
                                  "intercept.transition_time", "intercept.follow_close_speed",
                                  "intercept.bumpless_tau", "intercept.bumpless_time", "intercept.bumpless_close_speed", "radar.count", "radar.third"};
    if (std::none_of(std::begin(known), std::end(known), [&](const char* k) { return kv.first == k; })) {
      std::fprintf(stderr, "unknown --set key %s\n", kv.first.c_str());
      return 1;
    }
  }
  std::string third = "north";  // --set radar.third=north|center (applies with radar.count=3)
  for (const auto& [k, v] : sets)
    if (k == "radar.third") third = v;
  const follow_me::ConfigTweak tweak = [&](follow_me::InterceptConfig& c) {
    if (!imm) c.fusion.model = "cv";
    for (const auto& [k, v] : sets) {
      if (k == "fusion.model") c.fusion.model = v;
      else if (k == "fusion.q_calm") c.fusion.q_calm = std::stod(v);
      else if (k == "fusion.q_jerk") c.fusion.q_jerk = std::stod(v);
      else if (k == "fusion.singer_tau") c.fusion.singer_tau = std::stod(v);
      else if (k == "fusion.q_accel") c.fusion.q_accel = std::stod(v);
      else if (k == "intercept.a_ff_gain") c.intercept.a_ff_gain = std::stod(v);
      else if (k == "intercept.follow_a_max") c.intercept.follow_a_max = std::stod(v);
      else if (k == "intercept.follow_jerk_max") c.intercept.follow_jerk_max = std::stod(v);
      else if (k == "intercept.transition_jerk") c.intercept.transition_jerk = std::stod(v);
      else if (k == "intercept.transition_time") c.intercept.transition_time = std::stod(v);
      else if (k == "intercept.follow_close_speed") c.intercept.follow_close_speed = std::stod(v);
      else if (k == "intercept.bumpless_tau") c.intercept.bumpless_tau = std::stod(v);
      else if (k == "intercept.bumpless_time") c.intercept.bumpless_time = std::stod(v);
      else if (k == "intercept.bumpless_close_speed") c.intercept.bumpless_close_speed = std::stod(v);
      else if (k == "radar.count") {  // 2: second radar at (450, -100, 3), same draws as the first
        c.extra_radars.clear();
        if (std::stoi(v) >= 2) c.extra_radars.push_back(follow_me::InterceptConfig::secondRadarSite(c.radar));
        if (std::stoi(v) >= 3) c.extra_radars.push_back(follow_me::InterceptConfig::thirdRadarSite(c.radar, third));
      }
    }
  };
  if (intercept && mc_run >= 0) {
    follow_me::InterceptConfig ic;
    const auto im = follow_me::replayMonteCarloRun(mc_run, cfg.seed, maneuvers, out_dir, &ic, tweak);
    std::printf("Monte-Carlo run %d (target %.2f m, radar latency %.2f s):\n", mc_run, ic.target_size, ic.radar.latency);
    follow_me::printInterceptMetrics(im, ic.target_size);
    return 0;
  }
  if (intercept && montecarlo > 0) {
    const std::string path = (out_dir.empty() ? std::string(".") : out_dir) + "/montecarlo.csv";
    std::printf("Monte-Carlo: %d randomised interceptions -> %s\n", montecarlo, path.c_str());
    follow_me::runInterceptMonteCarlo(montecarlo, cfg.seed, path, maneuvers, tweak);
    return 0;
  }
  if (intercept) {  // phase 4: radar-cued interception of an unknown drone
    follow_me::InterceptConfig ic;
    ic.base.seed = cfg.seed;
    ic.out_dir = out_dir;
    if (aspect_gain >= 0) ic.aspect_gain = aspect_gain;
    ic.maneuver_level = maneuvers;
    ic.maneuver_seed = cfg.seed + 7;
    tweak(ic);
    ic.target_physics = target_physics;
    if (k_sigma >= 0) ic.base.safety.k_sigma = k_sigma;
    if (buffer >= 0) ic.base.safety.buffer = buffer;
    const auto im = follow_me::runInterceptSim(ic);
    std::printf("Intercept metrics:\n");
    follow_me::printInterceptMetrics(im, ic.target_size);
    return 0;
  }
  const auto m = follow_me::runOfflineSim(cfg);
  std::printf("Metrics:\n");
  follow_me::printMetrics(m);
  return 0;
}
