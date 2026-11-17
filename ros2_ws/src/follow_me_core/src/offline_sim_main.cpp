// Command line:  follow_me_offline_sim [--seed N] [--duration T] [--occlusion T0 T1]...
//                                      [--latency S] [--naive] [--point-mass] [--perfect-nav] [--no-nav-noise] [--csv FILE]
#include <cstdio>
#include <cstdlib>
#include <string>

#include "follow_me_core/offline_sim.hpp"

int main(int argc, char** argv) {
  follow_me::SimConfig cfg;
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
                   "usage: %s [--seed N] [--duration T] [--occlusion T0 T1]... [--latency S] [--naive] [--point-mass] [--perfect-nav] [--no-nav-noise] [--csv FILE]\n",
                   argv[0]);
      return 1;
    }
  }
  const auto m = follow_me::runOfflineSim(cfg);
  std::printf("Metrics:\n");
  follow_me::printMetrics(m);
  return 0;
}
