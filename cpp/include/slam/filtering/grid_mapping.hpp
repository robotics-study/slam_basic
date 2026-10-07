#pragma once

#include <string>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/types.hpp"

// grid_mapping — log-odds occupancy mapping over GIVEN odometry (Thrun, Burgard &
// Fox 2005 ch. 9; the idea is Moravec 1988 / Elfes 1989 in log-odds form). The
// C++ mirror of python/slam/filtering/grid_mapping.py, operation for operation.
//
// This is the OTHER half of the filtering branch: the histogram filter put its
// belief over the ROBOT on a given map; this puts a belief over the MAP on a
// given pose. The recursion per cell is the same Bayes product, but cells are
// independent (the naive independence assumption IS the model) so each keeps one
// scalar log-odds:
//
//     l_t(m) = l_{t−1}(m) + log p(z_t | m occupied)/p(z_t | m free)
//
// - INVERSE MODEL: a scan point is an endpoint, so it says two things — the cell
//   it lands in was OCCUPIED (probability p_hit) and every cell before it along
//   the ray was FREE (probability p_free = p_hit, symmetric by contract). A
//   missed beam emits no point, so it updates NOTHING: unknown cells keep their
//   prior log-odds 0 (= probability 0.5, never "occupied"). With a symmetric
//   model both increments carry the same magnitude L = log(p/(1−p)).
// - POSE IS GIVEN: the scenario's NOISELESS odometry is integrated onto the
//   DECLARED anchor pose (x0/y0/theta_deg); ATE/RPE read ~0 by construction. What
//   blurs a real mapping-while-localizing run is what filter_based adds later.
// - RAY → CELLS (cells_along): the segment from the estimated pose point to the
//   endpoint point, walked cell-by-cell in traversal order with the same DDA step
//   rule as core/sim raycast (tie takes Y; an axis whose delta is 0 never steps).
//   The endpoint's cell is the hit (+L); cells before it get −L. A walk that
//   leaves the raster makes its last in-bounds cell the hit; a start off-raster
//   updates nothing.
//
// Fixed operation order so both languages land on identical bits: points in scan
// order, cells near→far per walk, per-cell accumulation in arrival order, touched
// set emitted row-major sorted (std::set<Cell> ordered by (row, col) walks in
// exactly Python's sorted(tuple) order).

namespace slam::filtering {

// Cells entered along the CLOSED segment from_pt → to_pt, in traversal order —
// the last one is the endpoint's cell (the "hit"). Fixed DDA arithmetic mirrors
// core/sim raycast's step rule on the raw segment vector (no angle roundtrip):
// tMax starts at the ray parameter of the next grid-line crossing and advances by
// res/|d|; an axis with delta 0 never steps; a tie (tMaxX == tMaxY) takes Y.
std::vector<slam::core::Cell> cells_along(const slam::core::ScanGrid& grid,
                                          slam::core::Point from_pt, slam::core::Point to_pt);

// Not `final`: the unit tests drive update() directly and read the field through a
// probe subclass, mirroring how the Python tests reach _l.
class GridMapping : public slam::core::Estimator {
 public:
  explicit GridMapping(slam::core::ParamSet params);

  std::string name() const override { return "grid_mapping"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 protected:
  // Protected (not private) on purpose — the unit tests read the log-odds field and
  // inject an episode directly through a probe subclass.
  double logit_;   // L = log(p_hit / (1 − p_hit)) — the single increment magnitude
  double x0_, y0_, theta_deg_;  // declared anchor pose (theta in degrees)
  bool initialized_ = false;
  std::vector<double> field_;   // row-major [H*W] log-odds, prior 0.0 everywhere

 private:
  std::vector<slam::core::Pose> poses_;
};

}  // namespace slam::filtering
