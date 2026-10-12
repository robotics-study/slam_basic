#include "slam/features/dbscan.hpp"

#include <utility>

#include "slam/core/geometry.hpp"

// Bit-identical mirror of the Python module. Every float op rounds exactly where the
// Python line does: individual multiply/subtract/add, no fusion (-ffp-contract=off);
// robot_to_world and pose_compose route through core/geometry (libm cos/sin pairs).
// No sqrt appears anywhere — the closed ball is compared on SQUARED distance.

namespace slam::features {

using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::LandmarkEstimate;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;

namespace {

// Indices j (ASCENDING, self included) with dx² + dy² ≤ eps² — the closed ball,
// compared on squared distance so no sqrt exists anywhere in this module. Self is
// always inside (0 ≤ eps²), which is what makes min_pts count itself.
std::vector<int> neighborhood(const std::vector<Point>& points, size_t i, double eps_sq) {
  const double xi = points[i].x;
  const double yi = points[i].y;
  std::vector<int> nb;
  for (size_t j = 0; j < points.size(); ++j) {
    const double dx = points[j].x - xi;
    const double dy = points[j].y - yi;
    if (dx * dx + dy * dy <= eps_sq) nb.push_back(static_cast<int>(j));
  }
  return nb;
}

}  // namespace

std::vector<std::vector<Point>> dbscan_cluster(const std::vector<Point>& points, double eps,
                                               int min_pts) {
  const size_t n = points.size();
  std::vector<int> labels(n, kUnvisited);
  const double eps_sq = eps * eps;
  std::vector<std::vector<Point>> clusters;
  for (size_t i = 0; i < n; ++i) {  // ascending visit order — fixed
    if (labels[i] != kUnvisited) continue;  // already a member or already sticky noise
    std::vector<int> nb = neighborhood(points, i, eps_sq);
    if (static_cast<int>(nb.size()) < min_pts) {
      labels[i] = kNoise;  // sparse at visit time → noise forever (sticky)
      continue;
    }
    const int k = static_cast<int>(clusters.size());  // discovery order IS the id
    clusters.emplace_back();
    std::vector<Point>& members = clusters.back();
    members.push_back(points[i]);  // seed first, in append order
    labels[i] = k;
    size_t j = 0;
    while (j < nb.size()) {  // ExpandCluster: growing list, index pointer — fixed
      const int q = nb[j];
      ++j;
      if (labels[q] != kUnvisited) continue;  // member already (possibly twice) or sticky noise
      std::vector<int> qnb = neighborhood(points, static_cast<size_t>(q), eps_sq);
      if (static_cast<int>(qnb.size()) >= min_pts) {  // q is core → its neighborhood extends the frontier
        for (const int m : qnb) {  // ascending append; labels filter at processing time
          if (labels[m] == kUnvisited) nb.push_back(m);
        }
      }
      labels[q] = k;
      members.push_back(points[static_cast<size_t>(q)]);
    }
  }
  return clusters;
}

Dbscan::Dbscan(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)),
      x0_(params_.get_float("x0")),
      y0_(params_.get_float("y0")),
      theta_deg_(params_.get_float("theta_deg")),
      eps_(params_.get_float("eps")),
      min_pts_(static_cast<int>(params_.get_int("min_pts"))) {}

void Dbscan::update(const Step& step, TraceRecorder* recorder) {
  // Integrate odometry onto the declared gauge (this branch measures NO motion), lift
  // this scan's points into world frame in beam order, append them to the
  // accumulation, re-cluster ALL accumulated points, and emit this step's events
  // (pose first, landmarks second — same event order as the other branches).
  if (poses_.empty()) {
    // t = 0 adopts the declared gauge pose. math.radians(theta_deg) is exactly
    // x * (pi / 180) — the same expression CPython evaluates, bit-for-bit.
    pose_ = Pose{x0_, y0_, theta_deg_ * (kPi / 180.0)};
  } else {
    if (!step.has_odom) {  // contract: t ≥ 1 always carries the command
      throw std::runtime_error("dbscan: step " + std::to_string(step.t) +
                               " arrived without an odom command");
    }
    pose_ = slam::core::pose_compose(pose_, step.odom);
  }
  poses_.push_back(pose_);

  if (step.has_scan) {
    for (const Point& z : step.scan) {  // beam ascending — fixed accumulation order
      points_.push_back(slam::core::robot_to_world(z, pose_));
    }
  }
  const std::vector<std::vector<Point>> clusters = dbscan_cluster(points_, eps_, min_pts_);

  std::vector<LandmarkEstimate> estimates;
  for (size_t k = 0; k < clusters.size(); ++k) {  // discovery order IS the id
    const std::vector<Point>& members = clusters[k];
    double mx = 0.0, my = 0.0;
    const double n = static_cast<double>(members.size());
    for (const Point& p : members) {  // append order — fixed; plain += sums, no compensation
      mx += p.x;
      my += p.y;
    }
    estimates.push_back(
        LandmarkEstimate{static_cast<int>(k), mx / n, my / n, false, 0.0, 0.0});
  }
  landmarks_ = estimates;

  if (recorder != nullptr) {
    // No covariance — dbscan carries no uncertainty model (that is what the
    // filter_based branch adds on top of this).
    recorder->pose_estimated(step.t, pose_);
    recorder->landmarks_updated(step.t, landmarks_);
  }
}

EstimateResult Dbscan::finalize(slam::core::TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  result.has_landmarks = true;
  result.landmarks = landmarks_;
  return result;
}

}  // namespace slam::features
