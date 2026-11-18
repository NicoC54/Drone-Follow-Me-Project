// Command line:  follow_me_offline_sim [--seed N] [--duration T] [--occlusion T0 T1]...
//                                      [--latency S] [--naive] [--point-mass] [--perfect-nav] [--no-nav-noise] [--intercept [--out DIR] [--montecarlo N]] [--csv FILE]
#include <cstdio>
#include <cstdlib>
#include <string>

#include "follow_me_core/intercept_sim.hpp"
#include "follow_me_core/offline_sim.hpp"

int main(int argc, char** argv) {
  follow_me::SimConfig cfg;
  bool intercept = false;
  int montecarlo = 0;
  std::string out_dir;
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
                   "usage: %s [--seed N] [--duration T] [--occlusion T0 T1]... [--latency S] [--naive] [--point-mass] [--perfect-nav] [--no-nav-noise] [--intercept [--out DIR] [--montecarlo N]] [--csv FILE]\n",
                   argv[0]);
      return 1;
    }
  }
  if (intercept && montecarlo > 0) {
    const std::string path = (out_dir.empty() ? std::string(".") : out_dir) + "/montecarlo.csv";
    std::printf("Monte-Carlo: %d randomised interceptions -> %s\n", montecarlo, path.c_str());
    follow_me::runInterceptMonteCarlo(montecarlo, cfg.seed, path);
    return 0;
  }
  if (intercept) {  // phase 4: radar-cued interception of an unknown drone
    follow_me::InterceptConfig ic;
    ic.base.seed = cfg.seed;
    ic.out_dir = out_dir;
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
