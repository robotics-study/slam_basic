"""Benchmark metrics — the language-neutral scoring contract (core, not demos).

Every demo reports these through run_finished; the definitions here ARE the spec
(C++ mirrors them operation for operation):

- ate_rmse      sqrt(mean_t || p_hat(t) - p_gt(t) ||^2) over ALL steps t = 0..T
                (position only, meters).
- rpe_rmse      relative pose error: for every step t >= 1 the robot-frame twist
                difference e_t = pose_minus(est_{t-1}, est_t) - pose_minus(gt_{t-1}, gt_t)
                (componentwise), and sqrt(mean ||e_t||^2) with the FIXED norm
                sqrt(dx^2 + dy^2 + dtheta^2) — meters and radians share one norm by
                convention, so this number is a convention too: compare across
                algorithms/languages, not against other tools.
- map_iou       occupied-cell IoU between the estimate's log-odds grid (occupied
                iff l > 0; unknown stays at its prior) and the ground-truth grid,
                over every cell of the (identically shaped — enforced here) raster.
                Both-empty is defined as 1.0.
- landmark_rmse sqrt(mean over ids present in BOTH estimate and ground truth of
                ||(ex, ey) - (gx, gy)||^2); omitted when either side has none.
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import pose_minus
from .types import Episode, EstimateResult, LandmarkEstimate, LogOddsGrid, Pose, ScanGrid


def ate_rmse(poses: list[Pose], gts: list[Pose]) -> float:
    """Absolute trajectory error (position only) over the common index range."""
    assert len(poses) == len(gts), "ATE needs one estimate per ground-truth step"
    total = 0.0
    for est, gt in zip(poses, gts, strict=True):
        dx = est.x - gt.x
        dy = est.y - gt.y
        total += dx * dx + dy * dy
    return math.sqrt(total / len(poses))


def rpe_rmse(poses: list[Pose], gts: list[Pose]) -> float:
    """Relative pose error over consecutive steps (fixed twist-norm convention)."""
    assert len(poses) == len(gts), "RPE needs one estimate per ground-truth step"
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for t in range(1, len(poses)):
        e_hat = pose_minus(poses[t - 1], poses[t])
        e_gt = pose_minus(gts[t - 1], gts[t])
        dx = e_hat.dx - e_gt.dx
        dy = e_hat.dy - e_gt.dy
        dt = e_hat.dtheta - e_gt.dtheta
        total += dx * dx + dy * dy + dt * dt
    return math.sqrt(total / (len(poses) - 1))


def map_iou(est: LogOddsGrid, gt: ScanGrid) -> float:
    """Occupied-cell IoU over the shared raster (see module docstring). The estimate
    must cover exactly the ground-truth raster (same shape/resolution/origin) — a
    mapping estimator builds its grid ON the scenario raster by construction."""
    est_arr = np.asarray(est.log_odds)
    gt_occ = ~gt.free_mask()
    if (
        est.resolution != gt.resolution
        or est.origin != gt.origin
        or est_arr.shape != gt_occ.shape
    ):
        raise ValueError("map_iou: estimate grid must share the ground-truth raster")
    est_occ = est_arr > 0.0
    inter = int(np.count_nonzero(est_occ & gt_occ))
    union = int(np.count_nonzero(est_occ | gt_occ))
    if union == 0:
        return 1.0
    return inter / union


def landmark_rmse(
    estimated: list[LandmarkEstimate], gts: list[tuple[float, float]]
) -> float | None:
    """Position RMSE over ids present on both sides; None when nothing matches."""
    gt_by_id = dict(enumerate(gts))
    total = 0.0
    n = 0
    for e in estimated:
        if e.id in gt_by_id:
            gx, gy = gt_by_id[e.id]
            dx = e.x - gx
            dy = e.y - gy
            total += dx * dx + dy * dy
            n += 1
    if n == 0:
        return None
    return math.sqrt(total / n)


def evaluate(result: EstimateResult, episode: Episode) -> dict[str, float]:
    """The run's metric bundle (keys are sorted only at trace emission)."""
    gts = [s.gt for s in episode.steps]
    metrics: dict[str, float] = {
        "ate_rmse": ate_rmse(list(result.poses), gts),
        "rpe_rmse": rpe_rmse(list(result.poses), gts),
    }
    if result.grid is not None:
        metrics["map_iou"] = map_iou(result.grid, episode.grid)
    if result.landmarks is not None and episode.landmarks is not None:
        lm = landmark_rmse(list(result.landmarks), list(episode.landmarks))
        if lm is not None:
            metrics["landmark_rmse"] = lm
    return metrics
