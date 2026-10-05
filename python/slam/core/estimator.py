"""The Estimator base — the common surface every algorithm in this repo implements.

Mirrors the C++ `core/estimator.hpp`. One abstract per-step hook (`update`) plus a
`finalize` that returns the run's EstimateResult; the concrete `run` template is
SHARED (not per-algorithm): it walks the episode's steps, echoes each input Step to
the trace as `step_observed` before the estimator sees it, then calls finalize. An
algorithm therefore never emits step_observed itself and never touches ground truth
except through what run() hands it.

An algorithm depends ONLY on these core abstractions (Step / Episode / EstimateResult
/ TraceRecorder / ParamSet) — never on a concrete map class (Episode.grid is the
ScanGrid protocol) and never on another algorithm module. `required_capabilities()`
declares which sensor type the estimator consumes; the demo asserts it matches the
scenario's sensor before running.
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod

from .params import ParamSet
from .trace import TraceRecorder
from .types import Episode, EstimateResult, Step


class Capability(enum.Enum):
    """Sensor kinds an estimator can consume (matches the scenario `sensor.type`)."""

    BEAM = "beam"  # laser beam scans (DDA raycast endpoints, robot frame)
    LANDMARKS = "landmarks"  # point landmark observations (id, bearing, range)


class Estimator(ABC):
    """Base for every estimator: filtering / registration / filter_based / graph_based."""

    def __init__(self, params: ParamSet) -> None:
        self.params = params
        self.episode: Episode | None = None  # set by run() before the first update

    @property
    @abstractmethod
    def name(self) -> str:
        """Algorithm id; matches the config filename and trace `algorithm` field."""
        ...

    @abstractmethod
    def required_capabilities(self) -> set[Capability]: ...

    @abstractmethod
    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        """Consume one step (odometry + observation); emit this step's estimation
        events on `recorder` (None = tracing off — zero cost on the hot path)."""
        ...

    @abstractmethod
    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        """Build the run's result; batch families emit their trajectory_found here."""
        ...

    def run(
        self, episode: Episode, recorder: TraceRecorder | None = None
    ) -> EstimateResult:
        """Shared template: echo each step, update per step, then finalize."""
        self.episode = episode
        for step in episode.steps:
            if recorder is not None:
                recorder.step_observed(step)
            self.update(step, recorder)
        return self.finalize(recorder)
