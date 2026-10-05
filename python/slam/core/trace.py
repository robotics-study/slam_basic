"""Step-by-step trace recorder — the contract for visualization.

Mirrors the C++ `core/trace.hpp`. Emits JSON Lines per `spec/trace_schema.json`.
`seq` starts at 0 and increments per event; both languages serialize each event's
fields in exactly the order documented there, so parsed traces match field-for-field
across languages. A SLAM trace carries NO wall-clock time: replay is driven purely
by `seq` order, and the only time an event ever carries is the step number `t`.

The demo driver emits `run_started` (the one event the estimator cannot emit — it
knows neither its scenario path nor its seed) and, through the base Estimator.run
loop, every `step_observed` (the input side: ground-truth pose, noisy odometry,
observation). The ESTIMATOR emits only estimation events (`pose_estimated`,
`belief_updated`, `particles_updated`, `landmarks_updated`, `map_updated`,
`constraint_added`, `trajectory_found`) at the same `t`. Floats are compared parsed,
not byte-wise (Python 5.0 vs C++ 5 is the same value); ints stay ints on both sides.

A null recorder must cost nothing on the hot path: callers guard every emit with
``if recorder is not None`` so a None recorder never runs any of this code.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from types import TracebackType
from typing import TextIO

from .params import ParamValue
from .types import LandmarkEstimate, Pose, SensorConfig, Step, Twist


class TraceRecorder:
    def __init__(self, out: TextIO, owns: bool = False) -> None:
        self._out = out
        self._owns = owns
        self._seq = 0

    def __enter__(self) -> TraceRecorder:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        # The C++ recorder writes through an ostream the caller owns; Python mirrors
        # that: open_trace() owns its file and closes it, a buffer-backed recorder
        # just flushes.
        self._out.flush()
        if self._owns:
            self._out.close()

    def _emit(self, event: str, fields: dict[str, object]) -> None:
        record: dict[str, object] = {"seq": self._seq}
        record["event"] = event
        record.update(fields)
        self._out.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._seq += 1

    # --- events -----------------------------------------------------------
    def run_started(
        self,
        algorithm: str,
        scenario: str,
        params: Mapping[str, ParamValue],
        seed: int,
        sensor: SensorConfig,
        landmarks: Sequence[tuple[float, float]] | None,
    ) -> None:
        # params is serialized with keys sorted so the Python and C++ recorders
        # (std::map iterates sorted) emit identical field order. The sensor block
        # keeps the scenario yaml's fixed field order (type, beams, fov_deg,
        # range_max, sigma_range, sigma_bearing — optional keys omitted).
        sensor_fields: dict[str, object] = {"type": sensor.type}
        if sensor.beams is not None:
            sensor_fields["beams"] = sensor.beams
        if sensor.fov_deg is not None:
            sensor_fields["fov_deg"] = sensor.fov_deg
        sensor_fields["range_max"] = sensor.range_max
        sensor_fields["sigma_range"] = sensor.sigma_range
        if sensor.sigma_bearing is not None:
            sensor_fields["sigma_bearing"] = sensor.sigma_bearing
        fields: dict[str, object] = {
            "algorithm": algorithm,
            "scenario": scenario,
            "params": dict(sorted(params.items())),
            "seed": seed,
            "sensor": sensor_fields,
        }
        if landmarks is not None:
            fields["landmarks"] = [[p[0], p[1]] for p in landmarks]
        self._emit("run_started", fields)

    def step_observed(self, step: Step) -> None:
        # One input step: the ground-truth pose (visualization + metrics only), the
        # noisy odometry command that ARRIVED at t (omitted on step 0 — no move
        # arrived there), and the observation: beam-scan endpoints in the ROBOT
        # frame (replay restores world points with the gt pose) or landmark
        # observations sorted by id.
        fields: dict[str, object] = {
            "t": step.t,
            "gt": [step.gt.x, step.gt.y, step.gt.theta],
        }
        if step.odom is not None:
            fields["odom"] = [step.odom.dx, step.odom.dy, step.odom.dtheta]
        if step.scan is not None:
            fields["scan"] = [[p[0], p[1]] for p in step.scan]
        if step.obs is not None:
            fields["obs"] = [
                {"id": o.id, "bearing": o.bearing, "range": o.range} for o in step.obs
            ]
        self._emit("step_observed", fields)

    def pose_estimated(
        self, t: int, pose: Pose, cov: Sequence[float] | None = None
    ) -> None:
        # The estimator's own pose belief at step t; `cov` is the diagonal standard
        # deviation triple [sx, sy, stheta] (filter families only — replay draws the
        # error ellipse from it).
        fields: dict[str, object] = {"t": t, "pose": [pose.x, pose.y, pose.theta]}
        if cov is not None:
            fields["cov"] = list(cov)
        self._emit("pose_estimated", fields)

    def belief_updated(self, t: int, cells: Sequence[tuple[int, int, float]]) -> None:
        # Histogram-filter belief: EVERY cell as [row, col, p] in row-major order
        # (p = probability the cell is occupied).
        self._emit(
            "belief_updated",
            {"t": t, "cells": [[int(r), int(c), p] for (r, c, p) in cells]},
        )

    def particles_updated(
        self, t: int, particles: Sequence[tuple[float, float, float, float]]
    ) -> None:
        # Particle cloud after this step's update/resampling: [x, y, theta, w] per
        # particle (w = normalized weight; uniform right after resampling).
        self._emit(
            "particles_updated",
            {"t": t, "particles": [[p[0], p[1], p[2], p[3]] for p in particles]},
        )

    def landmarks_updated(
        self, t: int, estimated: Sequence[LandmarkEstimate]
    ) -> None:
        # Current landmark estimates (id = the scenario list order it tracks).
        entries: list[dict[str, object]] = []
        for e in estimated:
            entry: dict[str, object] = {"id": e.id, "x": e.x, "y": e.y}
            if e.sx is not None:
                entry["sx"] = e.sx
            if e.sy is not None:
                entry["sy"] = e.sy
            entries.append(entry)
        self._emit("landmarks_updated", {"t": t, "estimated": entries})

    def map_updated(self, t: int, cells: Sequence[tuple[int, int, float]]) -> None:
        # Log-odds map growth: ONLY cells whose log-odds changed since the previous
        # emission, as [row, col, l] (sparse by construction).
        self._emit(
            "map_updated",
            {"t": t, "cells": [[int(r), int(c), lo] for (r, c, lo) in cells]},
        )

    def constraint_added(
        self, i: int, j: int, d: Twist, is_loop: bool | None = None
    ) -> None:
        # Graph branch: a pose-pose constraint between trajectory nodes i and j —
        # the observed relative pose `d` in node i's frame. `loop` marks a loop
        # closure (a re-observation of a NON-adjacent node); omitted on odometry edges.
        fields: dict[str, object] = {
            "i": i,
            "j": j,
            "d": [d.dx, d.dy, d.dtheta],
        }
        if is_loop is not None:
            fields["loop"] = is_loop
        self._emit("constraint_added", fields)

    def trajectory_found(self, poses: Sequence[Pose]) -> None:
        # Batch (graph_based) final trajectory: index == step t.
        self._emit(
            "trajectory_found", {"poses": [[p.x, p.y, p.theta] for p in poses]}
        )

    def run_finished(self, metrics: Mapping[str, float]) -> None:
        # metrics keys are sorted for byte-identical cross-language output.
        self._emit("run_finished", {"metrics": dict(sorted(metrics.items()))})


def open_trace(path: str) -> TraceRecorder:
    """Open ``path`` for writing and return a recorder that owns the file."""
    return TraceRecorder(open(path, "w", encoding="utf-8"), owns=True)
