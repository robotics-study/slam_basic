#pragma once

#include <string>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/types.hpp"

// Histogram filter — the Bayes filter of Chapter 2 over a DISCRETE pose space. The
// C++ mirror of python/slam/filtering/histogram_filter.py, operation for operation
// (same fixed orders: ascending (fi, k) table build, observation-major/state-minor
// update, ascending readout sums; scalar libm calls everywhere — see core/libm.hpp).
//
// - STATE: every (free cell × heading lattice point) pair, index s = fi * B + b.
//   Occupied cells are not states at all — their mass is exactly 0 forever.
// - LATTICE: the heading axis discretizes onto the SENSOR's own angular grid,
//   w = radians(fov)/(beams−1), B = 2π/w bins with centers θ_b = −π + b·w equal to
//   the beam angles themselves — a lattice-driven scenario's true heading lands ON
//   the lattice and the expected-range table is an exact rotation of one DDA table.
// - PRIOR: uniform over all states (global localization from nothing but a map).
// - LIKELIHOOD: precomputed R[fi][k] by DDA; H[j][s] = R[fi][(b + j − off) mod B].
//   Each observed point z adds −½·((r − H[j][s])/σ)²; missed beams carry no term.
// - UPDATE: Bayes product with the prior in log space, p'(s) ∝ p(s)·exp(L(s) − m);
//   if every weight underflows to 0 the belief stays unchanged (documented).
// - PREDICT: child-of-movement — each hypothesis rides the exact odometry twist
//   from its own cell-center/lattice-heading pose, quantized back onto the lattice
//   (nearest center, ties up), mixed with slip probability p_slip of staying put;
//   a move into an occupied cell or off-grid does not move (slide semantics).
// - READOUT: position = belief-weighted mean of cell centers; heading = the lattice
//   angle with the largest bin marginal (first max wins); cov = population stds.

namespace slam::filtering {

// Not `final`: the unit tests drive init_state/predict through a probe subclass.
class HistogramFilter : public slam::core::Estimator {
 public:
  explicit HistogramFilter(slam::core::ParamSet params);

  std::string name() const override { return "histogram_filter"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 protected:
  // Protected (not private) on purpose — the unit tests drive init_state/predict and
  // read the belief directly through a probe subclass, mirroring how the Python tests
  // reach _init_state/_predict/_belief.
  void init_state();
  void predict(const slam::core::Twist& u);
  void update_scan(const std::vector<slam::core::Point>& scan);

  double p_slip_;
  double sigma_range_;
  int beams_;
  double fov_deg_;
  double range_max_;

  bool initialized_ = false;
  double w_bin_ = 0.0;   // heading lattice step == the sensor's angular step
  int bins_ = 0;         // lattice size B = 2π / w_bin (aligned by config)
  double half_ = 0.0;    // fov / 2 (radians)
  int off_ = 0;          // lattice offset: half / w_bin ((beams − 1) // 2)
  int grid_h_ = 0;
  int grid_w_ = 0;
  int n_states_ = 0;
  std::vector<slam::core::Cell> free_cells_;    // free cells in row-major order (index = fi)
  std::vector<int> fi_of_cell_;                 // cell_index -> free index (-1 for occupied)
  std::vector<slam::core::Point> centers_;      // world center of each free cell (by fi)
  std::vector<std::vector<double>> h_tables_;   // h_tables_[j][s]: expected hit distance
  std::vector<double> belief_;                  // [n_states], sums to 1.0

 private:
  std::vector<slam::core::Pose> poses_;
};

}  // namespace slam::filtering
