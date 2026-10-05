"""Estimator base (run template) + metrics contract tests."""

import io
import json

import numpy as np
from conftest import grid_from

from slam.core.estimator import Capability, Estimator
from slam.core.geometry import pose_compose
from slam.core.metrics import ate_rmse, evaluate, landmark_rmse, map_iou, rpe_rmse
from slam.core.params import ParamSet
from slam.core.trace import TraceRecorder
from slam.core.types import (
    Episode,
    EstimateResult,
    LandmarkEstimate,
    LogOddsGrid,
    Pose,
    Step,
    Twist,
)


class _OdometryEcho(Estimator):
    """Minimal estimator: dead-reckons its pose from odometry only (no sensor use)."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self.poses: list[Pose] = []
        self._pose: Pose | None = None

    @property
    def name(self) -> str:
        return "echo"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM, Capability.LANDMARKS}

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        if self._pose is None:
            # Step 0 has no odom: the estimator starts at the origin pose.
            self._pose = Pose(0.0, 0.0, 0.0)
        elif step.odom is not None:
            self._pose = pose_compose(self._pose, step.odom)
        self.poses.append(self._pose)
        if recorder is not None:
            recorder.pose_estimated(step.t, self._pose)

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        return EstimateResult(poses=tuple(self.poses))


def test_run_template_echoes_steps_and_returns_poses() -> None:
    steps = (
        Step(t=0, gt=Pose(1.0, 2.0, 0.0), odom=None, scan=(), obs=None),
        Step(t=1, gt=Pose(2.0, 2.0, 0.0), odom=Twist(1.0, 0.0, 0.0), scan=(), obs=None),
    )
    est = _OdometryEcho(ParamSet("echo", "filtering", {}))
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    episode = Episode(steps=steps, landmarks=None, grid=grid_from([".."]))
    result = est.run(episode, rec)

    # Dead reckoning from the origin: (0,0,0) then compose(Twist(1,0,0)) -> (1,0,0).
    assert [(p.x, p.y) for p in result.poses] == [(0.0, 0.0), (1.0, 0.0)]
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    # step_observed echoed BEFORE the estimate event, in seq order.
    assert [e["event"] for e in events] == [
        "step_observed", "pose_estimated", "step_observed", "pose_estimated",
    ]
    assert events[0]["t"] == 0 and events[1]["t"] == 0

    # A None recorder costs nothing: no events, same result.
    est2 = _OdometryEcho(ParamSet("echo", "filtering", {}))
    r2 = est2.run(episode, None)
    assert [(p.x, p.y) for p in r2.poses] == [(0.0, 0.0), (1.0, 0.0)]


def test_ate_rpe_golden() -> None:
    gts = [Pose(0.0, 0.0, 0.0), Pose(1.0, 0.0, 0.0)]
    ests = [Pose(0.0, 1.0, 0.0), Pose(1.0, 1.0, 0.0)]
    # ATE: position errors (0,1) and (0,1): sqrt(mean(1+1)/2)... mean of squares = 1.
    assert ate_rmse(ests, gts) == 1.0
    # RPE: both trajectories move exactly +x per step -> relative twist difference 0.
    assert rpe_rmse(ests, gts) == 0.0


def test_map_iou_and_landmarks() -> None:
    grid = grid_from([".#", ".."])  # occupied at (row0,col1)
    est = LogOddsGrid(
        resolution=1.0, origin=(0.0, 0.0), log_odds=np.array([[-1.0, 1.0], [-1.0, -1.0]])
    )
    assert map_iou(est, grid) == 1.0  # identical occupancy

    wrong = LogOddsGrid(
        resolution=1.0, origin=(0.0, 0.0), log_odds=np.array([[-1.0, -1.0], [-1.0, -1.0]])
    )
    assert map_iou(wrong, grid) == 0.0  # est sees nothing -> inter 0, union 1

    lm = landmark_rmse([LandmarkEstimate(0, 1.0, 2.0)], [(1.0, 3.0)])
    assert lm == 1.0
    assert landmark_rmse([], [(1.0, 3.0)]) is None


def test_evaluate_bundle_keys() -> None:
    gt_grid = grid_from([".#", ".."], resolution=2.0)

    class _SameRasterGrid:
        """ScanGrid view over the same raster (episode-side ground truth)."""

        def __init__(self, inner) -> None:
            self._inner = inner

        @property
        def width(self): return self._inner.width

        @property
        def height(self): return self._inner.height

        @property
        def resolution(self): return self._inner.resolution

        @property
        def origin(self): return self._inner.origin

        def in_bounds(self, row, col): return self._inner.in_bounds(row, col)

        def occupied(self, row, col): return self._inner.occupied(row, col)

        def free_mask(self): return self._inner.free_mask()

        def cell_to_world(self, row, col): return self._inner.cell_to_world(row, col)

        def world_to_cell(self, x, y): return self._inner.world_to_cell(x, y)

    gt_pose = Pose(0.0, 0.0, 0.0)
    episode = Episode(
        steps=(Step(t=0, gt=gt_pose, odom=None, scan=(), obs=None),),
        landmarks=((1.0, 3.0),),
        grid=_SameRasterGrid(gt_grid),
    )
    # Estimate grid must share the raster: same resolution/origin as gt_grid.
    est_grid = LogOddsGrid(
        resolution=gt_grid.resolution,
        origin=gt_grid.origin,
        log_odds=np.array([[-1.0, 1.0], [-1.0, -1.0]]),
    )
    result = EstimateResult(
        poses=(Pose(0.0, 1.0, 0.0),),
        landmarks=(LandmarkEstimate(0, 1.0, 2.0),),
        grid=est_grid,
    )
    m = evaluate(result, episode)
    assert set(m) == {"ate_rmse", "rpe_rmse", "map_iou", "landmark_rmse"}
