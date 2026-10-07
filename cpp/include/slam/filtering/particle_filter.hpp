#pragma once

#include <string>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/rng.hpp"
#include "slam/core/types.hpp"

// particle_filter — the bootstrap filter (Gordon, Salmond & Smith 1993; the robot
// localization formulation is Fox, Burgard, Dellaert & Thrun AAAI 1999 and Dellaert,
// Fox, Burgard & Thrun ICRA 1999, textbook form Thrun, Burgard & Fox 2005 ch. 4).
// The C++ mirror of python/slam/filtering/particle_filter.py, operation for
// operation.
//
// The filtering branch's third step: the histogram filter kept a probability for
// every lattice state by brute force; this one keeps N SAMPLES instead — a weighted
// particle cloud over the CONTINUOUS pose (x, y, θ) — and pays for that freedom
// with sampling. Nothing else about the Bayes filter changed: predict still pushes
// every hypothesis through the motion model, update still multiplies in the sensor
// likelihood, and resampling is what replaces the exact marginalization the lattice
// could afford.
//
// - STATE / PRIOR: N particles drawn uniformly over the DECLARED start cell (the
//   one containing x0/y0) with heading uniform on [−π, π). Position is known to a
//   cell, heading not at all — LOCAL localization; how many samples a GLOBAL prior
//   needs is the next page's question (MCL / KLD-sampling).
// - PREDICT: every particle rides the arriving command with its own drawn noise,
//   x' = x ⊕ (u + ε), ε ~ N(0, diag(σ_xy², σ_θ²)), draws in ascending particle
//   order. The scenario commands exact twists (its odom_noise is 0): the drift this
//   branch teaches lives entirely inside the filter's model.
// - UPDATE: each scan point z carries r = |z| and β = atan2; the expectation is
//   raycast(particle) along heading + β (no wrap needed — cos/sin are periodic), a
//   miss keeps the sentinel range_max + res exactly like the sim, and the point
//   contributes −½·((r − expected)/σ_range)². Weights multiply by exp(ll − max ll)
//   and renormalize; a total of 0.0 leaves the belief unchanged (documented
//   degenerate case, identical in Python).
// - RESAMPLE: systematic (Thrun's pseudocode 2.8) when ESS = 1/Σw² < N/2 — one
//   uniform draw u ∈ [0, 1/N), a cumulative-weight walk with a strict `>` tie rule,
//   and weights reset to exactly 1/N afterwards. Without it the cloud collapses
//   onto one ancestor within a few steps (sample impoverishment).
// - READOUT: position = weighted mean, heading = atan2(Σ w·sin θ, Σ w·cos θ) (the
//   arithmetic mean of angles lies across the ±π seam; this one cannot), spread =
//   population std devs — x/y plain, θ through wrap so the seam never explodes.
//
// Everything below is a fixed operation order so both languages land on identical
// bits: draws in ascending particle order; likelihood per scan point in scan order
// per particle; max-shifted exps accumulated ascending into a fresh list, then
// divided ascending; resample walk with strict `>` and the loop's final (unused)
// u += 1/N kept.

namespace slam::filtering {

// Not `final`: the unit tests drive init_particles/move/weight/resample directly
// and read the cloud through a probe subclass, mirroring how Python reaches _x/_y/
// _theta/_w/_rng.
class ParticleFilter : public slam::core::Estimator {
 public:
  explicit ParticleFilter(slam::core::ParamSet params);

  std::string name() const override { return "particle_filter"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 protected:
  // Draw N particles in ascending order: the start cell (the one holding x0/y0)
  // uniformly in x and y, heading uniform on [−π, π). Three draws per particle —
  // x offset, y offset, heading — from the algorithm's own stream.
  void init_particles();
  // x ⊕ (u + ε), per-particle draws in ascending order (three gaussians = six
  // uniforms per particle, fixed order ex, ey, eθ).
  void move(const slam::core::Twist& u);
  // Multiply every weight by exp(ll − max ll) and renormalize (scan-order points;
  // a miss keeps the sentinel range_max + res).
  void weight(const std::vector<slam::core::Point>& scan);
  // Systematic resampling when ESS = 1/Σw² drops below N/2.
  void resample_if_effective();

  int n_;
  double sigma_range_, sigma_xy_, sigma_theta_;
  double x0_, y0_, range_max_;
  slam::core::Rng rng_;
  double inv_n_;

  std::vector<double> x_, y_, theta_, w_;  // the cloud (ascending particle order)
  bool initialized_ = false;

 private:
  std::vector<slam::core::Pose> poses_;  // readout history (index == step t)
};

}  // namespace slam::filtering
