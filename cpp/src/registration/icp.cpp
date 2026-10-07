#include "slam/registration/icp.hpp"

#include <cmath>
#include <utility>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"

// Bit-identical mirror of the Python module. cos/sin PAIRS on one argument (the
// current estimate's heading, and the closed form's dtheta) route through
// core/libm.hpp (see there for why); atan2 binds directly to libSystem's scalar
// _atan2 (no pair fold exists for it). Every float op rounds exactly where the
// Python line does: individual multiply/add/subtract, no fusion
// (-ffp-contract=off), and `** 2` in Python is a correctly-rounded square —
// identical to x * x here.

namespace slam::registration {

using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

slam::core::Twist icp_step(const std::vector<Point>& source, const std::vector<Point>& target,
                           double d_max, double eps, int max_iters) {
  // Empty on either side: no pairs are possible — the identity twist exactly.
  if (source.empty() || target.empty()) return Twist{0.0, 0.0, 0.0};
  const double dm2 = d_max * d_max;  // squared once — no sqrt anywhere below
  double ax = 0.0, ay = 0.0, ang = 0.0;
  for (int it = 0; it < max_iters; ++it) {
    const double ca = slam::core::libm_cos(ang);  // cos first — fixed order
    const double sa = slam::core::libm_sin(ang);
    std::vector<double> qx, qy, px, py;  // the KEPT pairs (source / target sides)
    for (const Point& s : source) {      // ascending scan order — fixed
      const double tx = ca * s.x - sa * s.y + ax;
      const double ty = sa * s.x + ca * s.y + ay;
      // Nearest target point by SQUARED distance: j = 0's squared distance first,
      // then ascending strict `<` — a tie keeps the lower index.
      double best_d = (tx - target[0].x) * (tx - target[0].x) +
                      (ty - target[0].y) * (ty - target[0].y);
      std::size_t best_j = 0;
      for (std::size_t j = 1; j < target.size(); ++j) {
        const double dd = (tx - target[j].x) * (tx - target[j].x) +
                          (ty - target[j].y) * (ty - target[j].y);
        if (dd < best_d) {  // strict — a tie keeps the lower index
          best_d = dd;
          best_j = j;
        }
      }
      if (best_d <= dm2) {  // truncated least squares: beyond d_max there is no pair
        qx.push_back(s.x);
        qy.push_back(s.y);
        px.push_back(target[best_j].x);
        py.push_back(target[best_j].y);
      }
    }
    const std::size_t n = qx.size();
    if (n == 0) return Twist{ax, ay, ang};  // nothing left to align — carry the estimate
    double qx_bar = 0.0, qy_bar = 0.0, px_bar = 0.0, py_bar = 0.0;
    for (std::size_t j = 0; j < n; ++j) {  // ascending centroid sums — fixed
      qx_bar += qx[j];
      qy_bar += qy[j];
      px_bar += px[j];
      py_bar += py[j];
    }
    const double dn = static_cast<double>(n);
    qx_bar /= dn;
    qy_bar /= dn;
    px_bar /= dn;
    py_bar /= dn;
    double num = 0.0, den = 0.0;
    for (std::size_t j = 0; j < n; ++j) {  // centered cross/dot sums, ascending — fixed
      const double zx = qx[j] - qx_bar;
      const double zy = qy[j] - qy_bar;
      const double wx = px[j] - px_bar;
      const double wy = py[j] - py_bar;
      num += zx * wy - zy * wx;
      den += zx * wx + zy * wy;
    }
    // The 2D collapse of Besl & McKay's SVD: one atan2 for the rotation, then the
    // translation that aligns the centroids. cos before sin — fixed order.
    const double d_new = std::atan2(num, den);
    const double cr = slam::core::libm_cos(d_new);
    const double sr = slam::core::libm_sin(d_new);
    const double ax_new = px_bar - (cr * qx_bar - sr * qy_bar);
    const double ay_new = py_bar - (sr * qx_bar + cr * qy_bar);
    // Convergence: the max over all three components, compared AFTER the update.
    const double moved = std::fmax(std::fabs(ax_new - ax),
                                   std::fmax(std::fabs(ay_new - ay), std::fabs(d_new - ang)));
    ax = ax_new;
    ay = ay_new;
    ang = d_new;
    if (moved <= eps) break;  // fixed point of "pair then solve"
  }
  return Twist{ax, ay, ang};
}

Icp::Icp(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)),
      x0_(params_.get_float("x0")),
      y0_(params_.get_float("y0")),
      theta_deg_(params_.get_float("theta_deg")),
      d_max_(params_.get_float("d_max")),
      eps_(params_.get_float("eps")),
      max_iters_(static_cast<int>(params_.get_int("max_iters"))) {}

void Icp::update(const Step& step, TraceRecorder* recorder) {
  if (!has_pose_) {
    // t = 0 adopts the declared gauge pose. math.radians(theta_deg) is exactly
    // x * (pi / 180) — the same expression CPython evaluates, bit-for-bit.
    pose_ = Pose{x0_, y0_, theta_deg_ * (kPi / 180.0)};
    prev_ = step.scan;  // empty when the step carries no scan — the documented case
    has_pose_ = true;
  } else {
    static const std::vector<Point> kEmpty;
    const Twist twist = icp_step(step.has_scan ? step.scan : kEmpty, prev_, d_max_, eps_,
                                 max_iters_);
    pose_ = slam::core::pose_compose(pose_, twist);
    if (step.has_scan) prev_ = step.scan;  // a scan-less step leaves the target alone
  }
  poses_.push_back(pose_);
  if (recorder != nullptr) {
    // No covariance — ICP carries no uncertainty model (that is what the
    // filter_based branch adds on top of this).
    recorder->pose_estimated(step.t, pose_);
  }
}

EstimateResult Icp::finalize(TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  return result;
}

}  // namespace slam::registration
