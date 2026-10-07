#pragma once

#include <set>
#include <string>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/types.hpp"

// icp — point-to-point Iterative Closest Point (Besl & McKay, TPAMI 1992) run as
// pairwise scan matching: the registration branch's first member and its whole idea
// — pose from measurements alone, odometry without wheels. The C++ mirror of
// python/slam/registration/icp.py, operation for operation.
//
// The filtering branch rode on odometry; this branch replaces it. There is no map
// to match against either (that is the graph_based branch's move): the reference
// cloud is the PREVIOUS scan. At every step t >= 1 the arriving scan is the
// source, the previous scan is the target, and one ICP solve returns the SE(2)
// element that carries one onto the other; composing it onto the running estimate
// integrates motion into a pose. The absolute frame is not measured — it is
// DECLARED (x0/y0/theta_deg are the gauge), and everything after step 0 comes from
// scans alone. Step 0 emits that declared pose unchanged.
//
// THE SOLVE (fixed operation order — bit-identical against the Python module):
// source points are walked in scan order; each is transformed by the CURRENT
// estimate, matched to the target point minimizing the squared distance (ascending
// index scan, strict `<` — a tie keeps the lower index), and kept only if that
// distance is <= d_max under the current estimate. That truncation IS Besl &
// McKay's "truncated least squares". With the kept pairs (q_j -> p_j) the
// point-to-point error is minimized in CLOSED form — in 2D the SVD of Besl & McKay
// collapses to a centroid alignment plus one atan2:
//
//     dtheta = atan2( sum zx_j*wy_j - zy_j*wx_j , sum zx_j*wx_j + zy_j*wy_j )
//     dt = p_bar - R(dtheta) * q_bar   (centroids over the KEPT pairs, ascending sums)
//
// with z = q - q_bar, w = p - p_bar. The loop repeats (re-pair under the new
// estimate) until the pose update stops moving
// (max(|dx|, |dy|, dtheta) <= eps) or max_iters is spent. An empty source or
// target scan, or a step whose every pair was truncated away, returns the identity
// twist — no information, no motion.

namespace slam::registration {

// One ICP solve: the SE(2) element carrying `source` onto `target`. Fixed order:
// transform each source point by the current estimate (cos first), find its nearest
// target point by squared distance in ascending index order (strict `<` keeps the
// lower index on a tie), keep the pair iff that squared distance <= d_max^2, then
// closed-form solve over the kept pairs. cos/sin route through core/libm.hpp (the
// same-argument pair is exactly what Apple clang folds into __sincos_stret); atan2
// binds directly. sqrt-free throughout: squared distances only.
slam::core::Twist icp_step(const std::vector<slam::core::Point>& source,
                           const std::vector<slam::core::Point>& target, double d_max,
                           double eps, int max_iters);

// Point-to-point ICP scan matching on the scenario's empty room; the pose is the
// declared gauge pose composed with every recovered twist (Step.odom is never read
// — that is the branch). `final`: icp_step is a free function, so no unit test
// needs to reach inside this class.
class Icp final : public slam::core::Estimator {
 public:
  explicit Icp(slam::core::ParamSet params);

  std::string name() const override { return "icp"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  // t = 0 adopts the declared gauge pose (registration measures RELATIVE motion —
  // the absolute frame is a declaration, not a measurement). Every later step
  // aligns the arriving scan onto the previous one and composes the result; a step
  // without a scan carries the estimate forward untouched.
  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 private:
  double x0_, y0_, theta_deg_, d_max_, eps_;
  int max_iters_;
  bool has_pose_ = false;             // false until the first update adopts the gauge
  slam::core::Pose pose_{};           // running estimate (gauge composed with twists)
  std::vector<slam::core::Point> prev_;  // the previous step's scan (empty = none yet)
  std::vector<slam::core::Pose> poses_;  // readout history (index == step t)
};

}  // namespace slam::registration
