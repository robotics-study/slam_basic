#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "slam/core/types.hpp"

namespace slam::core {

// Benchmark metrics — the language-neutral scoring contract (core, not demos).
// Every demo reports these through run_finished; the definitions here ARE the spec
// (Python mirrors them operation for operation):
//
// - ate_rmse      sqrt(mean_t || p_hat(t) - p_gt(t)||^2) over ALL steps t = 0..T
//                 (position only, meters).
// - rpe_rmse      relative pose error: for every step t >= 1 the robot-frame twist
//                 difference e_t = pose_minus(est_{t-1}, est_t) - pose_minus(gt_{t-1},
//                 gt_t) (componentwise), and sqrt(mean ||e_t||^2) with the FIXED norm
//                 sqrt(dx^2 + dy^2 + dtheta^2) — meters and radians share one norm by
//                 convention, so this number is a convention too: compare across
//                 algorithms/languages, not against other tools.
// - map_iou       occupied-cell IoU between the estimate's log-odds grid (occupied
//                 iff l > 0; unknown stays at its prior) and the ground-truth grid,
//                 over every cell of the (identically shaped — enforced here) raster.
//                 Both-empty is defined as 1.0.
// - landmark_rmse sqrt(mean over ids present in BOTH estimate and ground truth of
//                 ||(ex, ey) - (gx, gy)||^2); absent when either side has none.

double ate_rmse(const std::vector<Pose>& poses, const std::vector<Pose>& gts);
double rpe_rmse(const std::vector<Pose>& poses, const std::vector<Pose>& gts);
double map_iou(const LogOddsGrid& est, const ScanGrid& gt);
std::optional<double> landmark_rmse(const std::vector<LandmarkEstimate>& estimated,
                                    const std::vector<Point>& gts);

// The run's metric bundle. Returns a std::map — iteration is already key-sorted,
// the order Python sorts into at trace emission. ate/rpe always present; map_iou
// only when the result carries a grid, landmark_rmse only when both sides carry
// landmarks and at least one id matches.
std::map<std::string, double> evaluate(const EstimateResult& result, const Episode& episode);

}  // namespace slam::core
