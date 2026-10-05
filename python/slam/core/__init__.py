"""Core abstractions: state types, RNG contract, geometry, params, trace, simulator,
metrics, estimator base. Depends only on stdlib + numpy. Knows nothing about maps
concretely (only the ScanGrid protocol) and nothing about algorithms."""

from .estimator import Capability, Estimator
from .geometry import (
    point_in_rect,
    pose_compose,
    pose_minus,
    robot_to_world,
    segment_intersects_rect,
    segments_intersect,
    wrap,
)
from .metrics import ate_rmse, evaluate, landmark_rmse, map_iou, rpe_rmse
from .params import ParamDecl, ParamError, ParamSet, ParamValue
from .rng import Rng
from .sim import build_episode, landmark_visible, raycast, resample
from .trace import TraceRecorder, open_trace
from .types import (
    Cell,
    Episode,
    EstimateResult,
    LandmarkEstimate,
    LandmarkObs,
    LogOddsGrid,
    Point,
    Pose,
    ScanGrid,
    SensorConfig,
    Step,
    Twist,
)

__all__ = [
    "Capability",
    "Estimator",
    "point_in_rect",
    "pose_compose",
    "pose_minus",
    "robot_to_world",
    "segments_intersect",
    "segment_intersects_rect",
    "wrap",
    "ate_rmse",
    "evaluate",
    "landmark_rmse",
    "map_iou",
    "rpe_rmse",
    "ParamDecl",
    "ParamError",
    "ParamSet",
    "ParamValue",
    "Rng",
    "build_episode",
    "landmark_visible",
    "raycast",
    "resample",
    "TraceRecorder",
    "open_trace",
    "Cell",
    "Episode",
    "EstimateResult",
    "LandmarkEstimate",
    "LandmarkObs",
    "LogOddsGrid",
    "Point",
    "Pose",
    "ScanGrid",
    "SensorConfig",
    "Step",
    "Twist",
]
