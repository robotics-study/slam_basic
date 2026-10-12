#pragma once

#include <array>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/types.hpp"

// ndt — Normal Distributions Transform (Biber & Strasser, IROS 2003) run as pairwise
// scan matching: the registration branch's second member and its other idea — where
// ICP matched point to point, NDT never matches anything. The reference scan is
// compiled into a per-cell probability field and the arriving scan is SCORED against
// that field directly; correspondence dissolves into the field itself. The C++
// mirror of python/slam/registration/ndt.py, operation for operation.
//
// THE FIELD: target points bucket into cells keyed by (floor(x/cell), floor(y/cell))
// in scan order; a cell with fewer than 3 points carries no distribution at all, and
// one whose scatter determinant is EXACTLY 0.0 carries none either. Every remaining
// cell stores mu = (1/n) sum x_j and the RAW scatter S = sum (x_j − mu)(x_j − mu)^T —
// deliberately not divided by n (the paper normalizes; this repo does not, measured).
// The score sums exp(−m/2) per source point with m = q^T S^-1 q, q = R(a)x + a − mu.
//
// THE SOLVE: f = −score; g_j = sum E·u_j and H_jk = sum E(−u_j u_k + W_jk) accumulate
// in scan order; the step solves (H + λI)Δ = −g by cofactor expansion along row 0, λ
// doubling from lam0 while any Sylvester leading principal minor is ≤ 0; Δ is accepted
// only on STRICT score improvement, halving (exact in binary) until it improves or
// max|Δ| ≤ eps forces a fixed point. Empty input on either side returns the identity
// twist — no information, no motion.

namespace slam::registration {

// One cell's distribution: (mu_x, mu_y, Sxx, Sxy, Syy, det S) — mean normalized by n,
// scatter RAW; a cell is absent from the map when it holds fewer than 3 points or its
// scatter determinant is exactly 0.0. Keyed by (floor(x/cell), floor(y/cell)); lookup
// order never matters (every lookup is keyed), only the SCAN-order sums are fixed.
using CellDist = std::array<double, 6>;

// Bucket `points` (scan order) and fit one distribution per cell — ascending sums in
// insertion order, exactly like the Python dict iteration it mirrors.
std::map<std::pair<int, int>, CellDist> build_ndt(const std::vector<slam::core::Point>& points,
                                                  double cell_size);

// score(a): raw sum of Gaussian bumps, one per source point, in scan order; a point
// landing in a cell with no distribution contributes exactly 0. The quadratic form is
// (dxq*u1 + dyq*u2)/2 with u = S^-1 q — the same expression the gradient differentiates.
double ndt_score(const std::vector<slam::core::Point>& source,
                 const std::map<std::pair<int, int>, CellDist>& dists, double cell_size,
                 double ax, double ay, double ang);

// One NDT solve: the SE(2) element carrying `source` onto the field of `target`. Fixed
// order (see the module comment): scan-order accumulation, λ-doubling on the Sylvester
// minors, row-0 cofactor solve, strict-improvement halving line search. cos/sin pairs
// route through core/libm.hpp; exp binds directly (probe-verified bit-identical to
// CPython's math.exp across the score's exponent domain, like atan2/log — no pair fold
// exists for it) and floor is a hardware instruction.
slam::core::Twist ndt_step(const std::vector<slam::core::Point>& source,
                           const std::vector<slam::core::Point>& target, double cell_size,
                           double lam0, double eps, int max_iters);

// Normal Distributions Transform scan matching on the staircase scenario; the pose is
// the declared gauge pose composed with every recovered twist (Step.odom is never read
// — that is the branch). `final`: ndt_step is a free function, so no unit test needs
// to reach inside this class.
class Ndt final : public slam::core::Estimator {
 public:
  explicit Ndt(slam::core::ParamSet params);

  std::string name() const override { return "ndt"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  // t = 0 adopts the declared gauge pose (registration measures RELATIVE motion —
  // the absolute frame is a declaration, not a measurement). Every later step scores
  // the arriving scan against the previous one's field and composes the result; a
  // step without a scan carries the estimate forward untouched.
  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 private:
  double x0_, y0_, theta_deg_, cell_size_, lam0_, eps_;
  int max_iters_;
  bool has_pose_ = false;                // false until the first update adopts the gauge
  slam::core::Pose pose_{};              // running estimate (gauge composed with twists)
  std::vector<slam::core::Point> prev_;  // the previous step's scan (empty = none yet)
  std::vector<slam::core::Pose> poses_;  // readout history (index == step t)
};

}  // namespace slam::registration
