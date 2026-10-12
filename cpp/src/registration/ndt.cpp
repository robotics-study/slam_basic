#include "slam/registration/ndt.hpp"

#include <cmath>
#include <utility>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"

// Bit-identical mirror of the Python module. cos/sin PAIRS on one argument (the
// current estimate's heading) route through core/libm.hpp (see there for why); exp
// binds directly to libSystem's scalar _exp — probe-verified bit-identical to
// CPython's math.exp across 40k inputs spanning the score's exponent domain
// (including the denormal underflow tail), because no pair fold exists for it, just
// like atan2/log. floor is a hardware instruction and exact by construction. Every
// float op rounds exactly where the Python line does: individual multiply/add/subtract,
// no fusion (-ffp-contract=off).

namespace slam::registration {

using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

std::map<std::pair<int, int>, CellDist> build_ndt(const std::vector<Point>& points,
                                                  double cell_size) {
  // Bucket in scan order (insertion order in Python; here a keyed map — per-bucket
  // sums are independent, so bucket ORDER cannot change a single bit), then fit one
  // distribution per cell with ascending sums — plain += accumulation, which is what
  // the Python side runs too: it deliberately avoids the builtin sum(), because from
  // CPython 3.12 (gh-100425) that is Neumaier COMPENSATED summation, a different
  // arithmetic expression that lands ulps off these left-to-right sums.
  std::map<std::pair<int, int>, std::vector<Point>> buckets;
  for (const Point& p : points) {
    buckets[{static_cast<int>(std::floor(p.x / cell_size)),
             static_cast<int>(std::floor(p.y / cell_size))}]
        .push_back(p);
  }
  std::map<std::pair<int, int>, CellDist> dists;
  for (const auto& [key, pts] : buckets) {
    const double n = static_cast<double>(pts.size());
    if (pts.size() < 3) continue;  // a 2-point cluster cannot span the plane
    double mx = 0.0, my = 0.0;
    for (const Point& p : pts) {
      mx += p.x;
      my += p.y;
    }
    mx /= n;
    my /= n;
    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (const Point& p : pts) {
      const double dx = p.x - mx;
      const double dy = p.y - my;
      sxx += dx * dx;
      syy += dy * dy;
      sxy += dx * dy;
    }
    const double det = sxx * syy - sxy * sxy;
    if (det == 0.0) continue;  // strictly degenerate — not a measurement of that axis
    dists[key] = CellDist{mx, my, sxx, sxy, syy, det};
  }
  return dists;
}

double ndt_score(const std::vector<Point>& source,
                 const std::map<std::pair<int, int>, CellDist>& dists, double cell_size,
                 double ax, double ay, double ang) {
  // score(a): raw sum of Gaussian bumps in scan order; a point landing in a cell with
  // no distribution contributes exactly 0. m = q^T S^-1 q evaluated as (dxq*u1 +
  // dyq*u2)/2 with u = S^-1 q — the same expression the gradient differentiates.
  double s = 0.0;
  const double c = slam::core::libm_cos(ang);  // cos first — fixed order
  const double sn = slam::core::libm_sin(ang);
  for (const Point& q : source) {  // ascending scan order — fixed
    const double x = c * q.x - sn * q.y + ax;
    const double y = sn * q.x + c * q.y + ay;
    const auto it = dists.find({static_cast<int>(std::floor(x / cell_size)),
                                static_cast<int>(std::floor(y / cell_size))});
    if (it == dists.end()) continue;  // no distribution here — contributes exactly 0
    const CellDist& d = it->second;
    const double mx = d[0], my = d[1], sxx = d[2], sxy = d[3], syy = d[4], det = d[5];
    const double ixx = syy / det;  // S^-1 = adj(S)/det(S), raw scatter
    const double iyy = sxx / det;
    const double ixy = -sxy / det;
    const double dxq = x - mx;
    const double dyq = y - my;
    const double u1 = ixx * dxq + ixy * dyq;
    const double u2 = ixy * dxq + iyy * dyq;
    s += std::exp(-(dxq * u1 + dyq * u2) / 2.0);
  }
  return s;
}

slam::core::Twist ndt_step(const std::vector<Point>& source, const std::vector<Point>& target,
                           double cell_size, double lam0, double eps, int max_iters) {
  // Empty on either side: no field or nothing to score — the identity twist exactly.
  if (source.empty() || target.empty()) return Twist{0.0, 0.0, 0.0};
  const auto dists = build_ndt(target, cell_size);
  double ax = 0.0, ay = 0.0, ang = 0.0;
  for (int it = 0; it < max_iters; ++it) {
    std::array<double, 3> g{0.0, 0.0, 0.0};
    std::array<std::array<double, 3>, 3> h{{}};
    const double ca = slam::core::libm_cos(ang);
    const double sa = slam::core::libm_sin(ang);
    for (const Point& q : source) {  // ascending scan order — fixed
      const double x = ca * q.x - sa * q.y + ax;
      const double y = sa * q.x + ca * q.y + ay;
      const auto found = dists.find({static_cast<int>(std::floor(x / cell_size)),
                                     static_cast<int>(std::floor(y / cell_size))});
      if (found == dists.end()) continue;
      const CellDist& d = found->second;
      const double mx = d[0], my = d[1], sxx = d[2], sxy = d[3], syy = d[4], det = d[5];
      const double ixx = syy / det;  // S^-1 = adj(S)/det(S), raw scatter
      const double iyy = sxx / det;
      const double ixy = -sxy / det;
      const double dxq = x - mx;
      const double dyq = y - my;
      const double u1 = ixx * dxq + ixy * dyq;
      const double u2 = ixy * dxq + iyy * dyq;
      const double e = std::exp(-(dxq * u1 + dyq * u2) / 2.0);
      const double j3x = -sa * q.x - ca * q.y;
      const double j3y = ca * q.x - sa * q.y;
      const double qddx = -(ca * q.x - sa * q.y);
      const double qddy = -(sa * q.x + ca * q.y);
      const double u3 = j3x * u1 + j3y * u2;
      const double w11 = ixx, w12 = ixy, w22 = iyy;
      const double w13 = ixx * j3x + ixy * j3y;
      const double w23 = ixy * j3x + iyy * j3y;
      const double w33 = (qddx * u1 + qddy * u2) +
                         (j3x * (ixx * j3x + ixy * j3y) + j3y * (ixy * j3x + iyy * j3y));
      g[0] += e * u1;
      g[1] += e * u2;
      g[2] += e * u3;
      h[0][0] += e * (-u1 * u1 + w11);
      h[0][1] = h[1][0] = h[0][1] + e * (-u1 * u2 + w12);
      h[0][2] = h[2][0] = h[0][2] + e * (-u1 * u3 + w13);
      h[1][1] += e * (-u2 * u2 + w22);
      h[1][2] = h[2][1] = h[1][2] + e * (-u2 * u3 + w23);
      h[2][2] += e * (-u3 * u3 + w33);
    }
    // λ doubles from lam0 while any Sylvester leading principal minor of H + λI is ≤ 0
    // (terminates: every entry of H is finite).
    double hl[3][3];
    double det;
    {
      double lam = lam0;
      while (true) {
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j) hl[i][j] = h[i][j] + (i == j ? lam : 0.0);
        const double m1 = hl[0][0];
        const double m2 = hl[0][0] * hl[1][1] - hl[0][1] * hl[1][0];
        det = hl[0][0] * (hl[1][1] * hl[2][2] - hl[1][2] * hl[2][1]) -
              hl[0][1] * (hl[1][0] * hl[2][2] - hl[1][2] * hl[2][0]) +
              hl[0][2] * (hl[1][0] * hl[2][1] - hl[1][1] * hl[2][0]);
        if (m1 > 0.0 && m2 > 0.0 && det > 0.0) break;
        lam *= 2.0;
      }
    }
    // Cramer's rule, cofactor expansion along row 0 — fixed order.
    double step[3];
    for (int k = 0; k < 3; ++k) {
      double a_mat[3][3];
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a_mat[i][j] = hl[i][j];
      for (int i = 0; i < 3; ++i) a_mat[i][k] = -g[i];
      const double num = a_mat[0][0] * (a_mat[1][1] * a_mat[2][2] - a_mat[1][2] * a_mat[2][1]) -
                         a_mat[0][1] * (a_mat[1][0] * a_mat[2][2] - a_mat[1][2] * a_mat[2][0]) +
                         a_mat[0][2] * (a_mat[1][0] * a_mat[2][1] - a_mat[1][1] * a_mat[2][0]);
      step[k] = num / det;
    }
    const double s_cur = ndt_score(source, dists, cell_size, ax, ay, ang);
    double d0 = step[0], d1 = step[1], d2 = step[2];
    while (true) {  // strict improvement or fixed point — halving is exact in binary
      if (ndt_score(source, dists, cell_size, ax + d0, ay + d1, ang + d2) > s_cur) break;
      if (std::fmax(std::fabs(d0), std::fmax(std::fabs(d1), std::fabs(d2))) <= eps) {
        return Twist{ax, ay, ang};  // fixed point: no halving level improves
      }
      d0 /= 2.0;
      d1 /= 2.0;
      d2 /= 2.0;
    }
    ax += d0;
    ay += d1;
    ang += d2;
    if (std::fmax(std::fabs(d0), std::fmax(std::fabs(d1), std::fabs(d2))) <= eps) break;
  }
  return Twist{ax, ay, ang};
}

Ndt::Ndt(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)),
      x0_(params_.get_float("x0")),
      y0_(params_.get_float("y0")),
      theta_deg_(params_.get_float("theta_deg")),
      cell_size_(params_.get_float("cell_size")),
      lam0_(params_.get_float("lam0")),
      eps_(params_.get_float("eps")),
      max_iters_(static_cast<int>(params_.get_int("max_iters"))) {}

void Ndt::update(const Step& step, TraceRecorder* recorder) {
  if (!has_pose_) {
    // t = 0 adopts the declared gauge pose. math.radians(theta_deg) is exactly
    // x * (pi / 180) — the same expression CPython evaluates, bit-for-bit.
    pose_ = Pose{x0_, y0_, theta_deg_ * (kPi / 180.0)};
    prev_ = step.has_scan ? step.scan : std::vector<Point>{};  // empty = documented case
    has_pose_ = true;
  } else {
    static const std::vector<Point> kEmpty;
    const Twist twist = ndt_step(step.has_scan ? step.scan : kEmpty, prev_, cell_size_,
                                 lam0_, eps_, max_iters_);
    pose_ = slam::core::pose_compose(pose_, twist);
    if (step.has_scan) prev_ = step.scan;  // a scan-less step leaves the field alone
  }
  poses_.push_back(pose_);
  if (recorder != nullptr) {
    // No covariance — NDT carries no uncertainty model (that is what the
    // filter_based branch adds on top of this).
    recorder->pose_estimated(step.t, pose_);
  }
}

EstimateResult Ndt::finalize(slam::core::TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  return result;
}

}  // namespace slam::registration
