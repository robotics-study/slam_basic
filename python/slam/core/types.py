"""Language-neutral state / result types shared by every estimator.

Mirrors the C++ ``core/types.hpp``. Pose and twist are the contract's fixed-frame
types (world pose in meters/radians, robot-frame motion command); ``Step`` is one
simulator step (ground truth + what the algorithm actually sees) and ``Episode`` is
the whole stream an estimator consumes. The wire shapes in ``core/trace.py`` mirror
these types field for field.

Coordinate frames are owned by the map layer only: world points stay floats here,
grid indices stay ``(row, col)`` ints there (``ScanGrid`` exposes both frames and the
conversion between them).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

# World point (x, y) in meters — scan endpoints live in the ROBOT frame, every other
# point in this codebase is world-frame.
Point = tuple[float, float]
# Grid index (row, col), ints; row 0 = top image row. Owned by the map layer.
Cell = tuple[int, int]


@dataclass(frozen=True)
class Pose:
    """World pose (x, y meters, theta radians in [-pi, pi) after ``wrap``)."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Twist:
    """Robot-frame motion command: displacement (dx, dy) meters + rotation dtheta rad.

    A Step's twist is the EXACT inverse composition gt_{t-1}^-1 (+) gt_t plus drawn
    noise — see core/sim.py; algorithms never see the ground-truth pose itself.
    """

    dx: float
    dy: float
    dtheta: float


@dataclass(frozen=True)
class LandmarkObs:
    """One landmark observation: id = scenario list order, bearing relative to the
    robot heading (rad), range in meters. Association is GIVEN (as in the FastSLAM
    papers) — matching observations to landmarks is not an algorithm's problem."""

    id: int
    bearing: float
    range: float


@dataclass(frozen=True)
class LandmarkEstimate:
    """An estimator's current belief about landmark `id` (world position + optional
    per-axis standard deviations, emitted on landmarks_updated events)."""

    id: int
    x: float
    y: float
    sx: float | None = None
    sy: float | None = None


@dataclass(frozen=True)
class Step:
    """One simulator step t. `gt` is ground truth (visualization + metrics ONLY —
    algorithms read `odom` and the observation, never gt). `odom` is the noisy
    robot-frame command that ARRIVED at t (None on step 0: no move arrived there);
    exactly one of scan / obs carries the sensor reading per the scenario's type."""

    t: int
    gt: Pose
    odom: Twist | None
    scan: tuple[Point, ...] | None
    obs: tuple[LandmarkObs, ...] | None


@dataclass(frozen=True)
class SensorConfig:
    """Parsed `sensor:` block of a scenario yaml (see spec/data_formats.md). The
    `type` string equals the Capability value an estimator must declare."""

    type: str  # "beam" | "landmarks"
    range_max: float
    sigma_range: float
    beams: int | None = None
    fov_deg: float | None = None
    sigma_bearing: float | None = None


@dataclass(frozen=True)
class LogOddsGrid:
    """A log-odds occupancy map (the mapping branch's output): one float per cell on
    the SAME raster geometry as the ground-truth grid it is scored against. A cell
    counts occupied iff log_odds > 0 (unknown stays at its prior, never "occupied")."""

    resolution: float
    origin: tuple[float, float]
    log_odds: np.ndarray  # [H, W] float64


@runtime_checkable
class ScanGrid(Protocol):
    """Read-only grid view the simulator raycasts against and metrics score against.
    Algorithms depend on THIS (via Episode.grid), never on a concrete map class."""

    @property
    def width(self) -> int: ...

    @property
    def height(self) -> int: ...

    @property
    def resolution(self) -> float: ...

    @property
    def origin(self) -> tuple[float, float]: ...

    def in_bounds(self, row: int, col: int) -> bool: ...

    def occupied(self, row: int, col: int) -> bool: ...

    def free_mask(self) -> np.ndarray:
        """Boolean [H, W] mask of free cells (read-only view; viz + metrics read it)."""
        ...

    def cell_to_world(self, row: int, col: int) -> Point: ...

    def world_to_cell(self, x: float, y: float) -> Cell: ...


@dataclass(frozen=True)
class Episode:
    """Everything one estimator run consumes: the step stream (ground truth +
    noisy odometry + observations, drawn in contract order from the scenario seed),
    the ground-truth landmark points (id = list order; None for beam scenarios —
    visualization and metrics only), and the ground-truth grid."""

    steps: tuple[Step, ...]
    landmarks: tuple[Point, ...] | None
    grid: ScanGrid


@dataclass(frozen=True)
class EstimateResult:
    """What an estimator returns. `poses` is the per-step estimated pose sequence
    (index == step t). `landmarks` / `grid` are set only by branches that estimate
    them; metrics skip the matching metric when absent."""

    poses: tuple[Pose, ...]
    landmarks: tuple[LandmarkEstimate, ...] | None = None
    grid: LogOddsGrid | None = None
