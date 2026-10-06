#include "slam/core/metrics.hpp"

#include <cmath>
#include <map>
#include <stdexcept>

#include "slam/core/geometry.hpp"

namespace slam::core {

double ate_rmse(const std::vector<Pose>& poses, const std::vector<Pose>& gts) {
  // Absolute trajectory error (position only) over the common index range.
  if (poses.size() != gts.size()) {
    throw std::runtime_error("ATE needs one estimate per ground-truth step");
  }
  double total = 0.0;
  for (size_t i = 0; i < poses.size(); ++i) {
    double dx = poses[i].x - gts[i].x;
    double dy = poses[i].y - gts[i].y;
    total += dx * dx + dy * dy;
  }
  return std::sqrt(total / static_cast<double>(poses.size()));
}

double rpe_rmse(const std::vector<Pose>& poses, const std::vector<Pose>& gts) {
  // Relative pose error over consecutive steps (fixed twist-norm convention).
  if (poses.size() != gts.size()) {
    throw std::runtime_error("RPE needs one estimate per ground-truth step");
  }
  if (poses.size() < 2) return 0.0;
  double total = 0.0;
  for (size_t t = 1; t < poses.size(); ++t) {
    Twist e_hat = pose_minus(poses[t - 1], poses[t]);
    Twist e_gt = pose_minus(gts[t - 1], gts[t]);
    double dx = e_hat.dx - e_gt.dx;
    double dy = e_hat.dy - e_gt.dy;
    double dt = e_hat.dtheta - e_gt.dtheta;
    total += dx * dx + dy * dy + dt * dt;
  }
  return std::sqrt(total / static_cast<double>(poses.size() - 1));
}

double map_iou(const LogOddsGrid& est, const ScanGrid& gt) {
  // Occupied-cell IoU over the shared raster. The estimate must cover exactly the
  // ground-truth raster (same shape/resolution/origin) — a mapping estimator builds
  // its grid ON the scenario raster by construction.
  if (est.resolution != gt.resolution() || est.origin[0] != gt.origin()[0] ||
      est.origin[1] != gt.origin()[1] || est.height != gt.height() || est.width != gt.width()) {
    throw std::runtime_error("map_iou: estimate grid must share the ground-truth raster");
  }
  long long inter = 0, union_ = 0;
  for (int r = 0; r < gt.height(); ++r) {
    for (int c = 0; c < gt.width(); ++c) {
      bool est_occ = est.at(r, c) > 0.0;
      bool gt_occ = gt.occupied(r, c);
      if (est_occ && gt_occ) ++inter;
      if (est_occ || gt_occ) ++union_;
    }
  }
  if (union_ == 0) return 1.0;
  return static_cast<double>(inter) / static_cast<double>(union_);
}

std::optional<double> landmark_rmse(const std::vector<LandmarkEstimate>& estimated,
                                    const std::vector<Point>& gts) {
  // Position RMSE over ids present on both sides; absent when nothing matches.
  std::map<int, Point> gt_by_id;
  for (size_t id = 0; id < gts.size(); ++id) gt_by_id[static_cast<int>(id)] = gts[id];
  double total = 0.0;
  int n = 0;
  for (const LandmarkEstimate& e : estimated) {
    auto it = gt_by_id.find(e.id);
    if (it != gt_by_id.end()) {
      double dx = e.x - it->second.x;
      double dy = e.y - it->second.y;
      total += dx * dx + dy * dy;
      ++n;
    }
  }
  if (n == 0) return std::nullopt;
  return std::sqrt(total / static_cast<double>(n));
}

std::map<std::string, double> evaluate(const EstimateResult& result, const Episode& episode) {
  // The run's metric bundle (keys are sorted only at trace emission — a std::map
  // here is already sorted).
  std::vector<Pose> gts;
  for (const Step& s : episode.steps) gts.push_back(s.gt);
  std::map<std::string, double> metrics;
  metrics["ate_rmse"] = ate_rmse(result.poses, gts);
  metrics["rpe_rmse"] = rpe_rmse(result.poses, gts);
  if (result.has_grid) {
    metrics["map_iou"] = map_iou(result.grid, *episode.grid);
  }
  if (result.has_landmarks && episode.has_landmarks) {
    std::optional<double> lm = landmark_rmse(result.landmarks, episode.landmarks);
    if (lm.has_value()) metrics["landmark_rmse"] = *lm;
  }
  return metrics;
}

}  // namespace slam::core
