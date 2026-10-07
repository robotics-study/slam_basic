#pragma once

#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/rng.hpp"
#include "slam/core/types.hpp"

// mcl — Monte Carlo Localization with KLD-sampling (MCL is Fox, Burgard, Dellaert &
// Thrun AAAI 1999; the adaptive sample count is Fox, "Adapting the Sample Size in
// Particle Filters through KLD-sampling", IJRR 2003; the textbook form is Thrun,
// Burgard & Fox 2005 ch. 4). The C++ mirror of python/slam/filtering/mcl.py,
// operation for operation.
//
// The filtering branch's fourth step. The particle_filter page ended on a question:
// a fixed N is a lottery — too few samples and the true mode dies, too many wastes
// every raycast. MCL keeps the SAME filter (local prior over the declared cell ×
// full heading, bootstrap motion model, beam-likelihood weighting, weighted-mean
// readout) and adds ONE mechanism: at every resample-move step the sample count is
// not fixed — samples are drawn one at a time until their bin coverage
// statistically certifies the cloud is dense enough.
//
// WHY A SAMPLE COUNT CAN BE CERTIFIED (Fox 2003, derived on the algorithm's page):
// discretize belief into bins (here: bin_xy × bin_xy × bin_theta boxes of pose
// space) and let p be the true belief discretized over them. Samples drawn i.i.d.
// from p land in bins multinomially; the likelihood-ratio statistic for "these n
// samples came from p" is λ_n = Π (p̂_i/p_i)^{x_i}, its log is n·KL(p̂‖p) — the KL
// between the sample MLE and the true belief, discretized — and Wilks' result gives
// 2·n·KL → χ²_{k−1} in distribution (k = bins with support). So P(KL > ε) ≤ δ once
// n ≥ (1/2ε)·χ²_{k−1,1−δ}; Wilson–Hilferty turns that quantile into the closed
// form below, whose z_q is core/stats' inv_norm_cdf(1 − δ).
//
//     n_chi(k) = 0  (k ≤ 1 — degenerate: one bin needs no guarantee beyond n_min)
//              = (ν / 2ε) · t³,  ν = k − 1, q₉ = 2/(9ν), t = (1 − q₉) + z_q·√q₉
//
// THE LOOP (fixed operation order — bit-identical across languages): t = 0 draws
// max_particles particles uniformly over the declared cell × heading, weights them
// by the first scan exactly like particle_filter. From t ≥ 1, every sample is drawn
// one at a time: ancestor walk over the previous weights (one uniform draw, strict
// `>`, guard i < n−1), motion noise drawn and composed (x ⊕ (u + ε)), the arriving
// scan scored inline in scan order, the new pose binned — a bin never seen this
// step bumps k, which recomputes n_chi; the loop stops when n ≥ n_chi and n ≥
// n_min, or at the max_particles cap. Weights are then exp(ll − max ll) normalized
// ascending (the old weights already acted — they drove the ancestor walk). Readout
// is unchanged from the previous page: weighted mean x/y, circular-mean heading,
// population std devs.

namespace slam::filtering {

// Fox 2003 eq. (14): the Wilson–Hilferty closed form of n = χ²_{k−1,1−δ}/2ε — the
// sample count that guarantees KL(p̂_n ‖ p) ≤ ε with probability 1 − δ when the
// support spans k bins. Fixed operation order (t·t·t, left-associative); k ≤ 1 is
// the degenerate case: a one-bin belief needs no guarantee beyond n_min, so the
// bound returns 0.0 and the loop stops at its floor.
double kld_bound(int k, double epsilon, double z_q);

// Not `final`: the unit tests drive init_particles/weight/kld_update directly and
// read the cloud through a probe subclass, mirroring how Python reaches _x/_y/
// _theta/_w/_rng.
class Mcl : public slam::core::Estimator {
 public:
  explicit Mcl(slam::core::ParamSet params);

  std::string name() const override { return "mcl"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 protected:
  // Draw max_particles particles in ascending order (the PRIOR is fixed-size —
  // adaptivity starts at the first KLD step): the start cell uniformly in x and y,
  // heading uniform on [−π, π); weights exactly 1/max_particles.
  void init_particles();
  // (r, β) per scan point, in scan order — the polar form of each robot-frame
  // endpoint. Fixed formulas: sqrt(zx² + zy²), atan2(zy, zx).
  static std::vector<slam::core::Point> polar_points(const std::vector<slam::core::Point>& scan);
  // t = 0 only: multiply every uniform weight by exp(ll − max ll); renormalize
  // (scan-order points; a miss keeps the sentinel range_max + res).
  void weight(const std::vector<slam::core::Point>& points);
  // One KLD-sampling step: resample-move-reweight ONE sample at a time until the
  // bound certifies coverage (or the cap) — ancestor walk, three gaussians,
  // x ⊕ (u + ε), inline scan likelihood, bin key on first sight bumps k.
  void kld_update(const slam::core::Twist& u, const std::vector<slam::core::Point>& scan);

  double x0_, y0_, epsilon_, delta_, bin_xy_, bin_theta_;
  int n_min_, max_particles_;
  double sigma_range_, sigma_xy_, sigma_theta_, range_max_;
  slam::core::Rng rng_;
  double z_q_;  // the bound's quantile — computed ONCE from delta

  std::vector<double> x_, y_, theta_, w_;  // the cloud (ascending particle order)
  bool initialized_ = false;

 private:
  std::vector<slam::core::Pose> poses_;  // readout history (index == step t)
};

}  // namespace slam::filtering
