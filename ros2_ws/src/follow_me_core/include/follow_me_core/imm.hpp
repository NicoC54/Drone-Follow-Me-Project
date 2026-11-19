// Interacting Multiple Model (IMM) filter over TargetEkfCA models that share the state [p, v, a]:
// a CALM constant-velocity model (acceleration forced to 0, precise when the target flies straight)
// and a MANOEUVRE model (Singer acceleration: braking, turning, starting are modelled).
// (First version, 2026-11-18 morning: two constant-velocity models q = 3 / 40 on [p, v]; it kept
// the covariance honest but the estimate still lagged a target that stopped or started.)
//
// Cycle (Blom & Bar-Shalom):
//   1. mixing: each model restarts from a mixture of all models, weighted by the probability
//      that the target switched (Markov chain, mean sojourn time per model);
//   2. model-matched predictions (each with its own q);
//   3. model-matched updates; each model's measurement likelihood reweights its probability;
//   4. output: moment-matched combination (mean + spread of the means).
// With a single model it is exactly the previous constant-velocity EKF.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "follow_me_core/ekf_ca.hpp"

namespace follow_me {

class TargetImm {
 public:
  TargetImm() = default;
  // models: one filter per model (kind + process noise); sojourn: mean time spent in each model [s];
  // mu0: model probabilities at track start.
  TargetImm(std::vector<TargetEkfCA> models, const std::vector<double>& sojourn, const std::vector<double>& mu0)
      : m_(std::move(models)), sojourn_(sojourn), mu0_(mu0), mu_(mu0) {}

  // Track start from [p, v]: acceleration 0 +- sigma_a0.
  void reset(const Vec6& x, const Mat6& P, double sigma_a0 = 2.0) {
    Vec9 x9 = Vec9::Zero();
    x9.head<6>() = x;
    Mat9 P9 = Mat9::Zero();
    P9.topLeftCorner<6, 6>() = P;
    P9.bottomRightCorner<3, 3>() = sigma_a0 * sigma_a0 * Mat3::Identity();
    for (auto& m : m_) m.reset(x9, P9);
    mu_ = mu0_;
  }

  void predict(double dt) {
    const size_t n = m_.size();
    if (n > 1) {
      // Transition probabilities over dt: stay with exp(-dt / sojourn), else switch (uniformly).
      std::vector<std::vector<double>> pi(n, std::vector<double>(n));
      for (size_t i = 0; i < n; ++i) {
        const double stay = std::exp(-dt / sojourn_[i]);
        for (size_t j = 0; j < n; ++j) pi[i][j] = (i == j) ? stay : (1.0 - stay) / double(n - 1);
      }
      std::vector<double> c(n, 0.0);
      for (size_t j = 0; j < n; ++j)
        for (size_t i = 0; i < n; ++i) c[j] += pi[i][j] * mu_[i];
      std::vector<TargetEkfCA> mixed = m_;
      for (size_t j = 0; j < n; ++j) {
        Vec9 x = Vec9::Zero();
        for (size_t i = 0; i < n; ++i) x += pi[i][j] * mu_[i] / c[j] * m_[i].x();
        Mat9 P = Mat9::Zero();
        for (size_t i = 0; i < n; ++i) {
          const Vec9 d = m_[i].x() - x;
          P += pi[i][j] * mu_[i] / c[j] * (m_[i].P() + d * d.transpose());
        }
        mixed[j].reset(x, P);
      }
      m_ = std::move(mixed);
      mu_ = c;
    }
    for (auto& m : m_) m.predict(dt);
  }

  // `upd(model)` updates one model and returns the measurement log-likelihood under it
  // (NaN: not computable, probabilities untouched).
  template <class F>
  void update(F&& upd) {
    std::vector<double> ll(m_.size());
    for (size_t j = 0; j < m_.size(); ++j) ll[j] = upd(m_[j]);
    if (m_.size() < 2 || std::any_of(ll.begin(), ll.end(), [](double v) { return !std::isfinite(v); })) return;
    const double mx = *std::max_element(ll.begin(), ll.end());
    double sum = 0.0;
    for (size_t j = 0; j < m_.size(); ++j) sum += (mu_[j] *= std::exp(ll[j] - mx));
    for (auto& m : mu_) m = std::max(m / sum, 1e-6);  // floor: a model must be able to come back
  }

  // Moment-matched combination, full state [p, v, a].
  TargetEkfCA combined9() const {
    Vec9 x = Vec9::Zero();
    for (size_t j = 0; j < m_.size(); ++j) x += mu_[j] * m_[j].x();
    Mat9 P = Mat9::Zero();
    for (size_t j = 0; j < m_.size(); ++j) {
      const Vec9 d = m_[j].x() - x;
      P += mu_[j] * (m_[j].P() + d * d.transpose());
    }
    TargetEkfCA e(TargetEkfCA::Model::CV, 0.0);
    e.reset(x, P);
    return e;
  }
  // The [p, v] view used by gating, the mission and the outputs.
  TargetEkf combined() const { return combined9().pv(); }

  const std::vector<double>& mu() const { return mu_; }
  size_t size() const { return m_.size(); }

 private:
  std::vector<TargetEkfCA> m_;
  std::vector<double> sojourn_, mu0_, mu_;
};

// Gaussian log-likelihood of an innovation y with covariance S (constant terms dropped: they are
// the same for every model fed with the same measurement).
template <int M>
double innovationLogLikelihood(const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, M>& S) {
  const auto ldlt = S.ldlt();
  const double logdet = ldlt.vectorD().array().abs().log().sum();
  return -0.5 * (y.dot(ldlt.solve(y)) + logdet);
}

}  // namespace follow_me
