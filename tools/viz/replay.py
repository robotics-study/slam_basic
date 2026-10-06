#!/usr/bin/env python3
"""Replay a slam_basic trace jsonl over its scenario map (matplotlib).

Depends only on the trace/map spec + slam core/maps loaders — never on an
algorithm module. Everything visual arrives via trace events: per-step ground
truth (+ noisy odom), scan hits / landmark observations (restored to world with
that step's GT pose, per the contract), estimated poses (+ covariance), particle
clouds, belief/log-odds grids, landmark estimates, graph constraints and the batch
trajectory. A trace carries NO wall-clock time: replay is driven purely by `seq`
order — every drawable carries the normalized seq at which it appeared, so a frame
at cutoff c shows exactly the events with order <= c (accumulating maps accumulate).

Output modes (combinable; all but interactive are headless via the Agg backend):
  (default)          interactive window on the full accumulated frame
  --save out.png     the completed frame (with metrics overlay) to a PNG
  --gif out.gif      animated replay: sweep cutoff 0 -> 1, then hold the final
                     frame for a few seconds before looping
  --snapshots dir/   evenly-spaced mid-run PNG snapshots + the final frame
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from slam.maps.occupancy_grid import OccupancyGrid2D

Point = tuple[float, float]

# Frame budget for the GIF sweep + the hold on the completed frame (same watchability
# convention as the sibling repos).
_TARGET_FRAMES = 150
_DEFAULT_SNAPSHOTS = 8
_HOLD_SECONDS = 3

_GT_COLOR = "#0f172a"       # ground truth trail / robot
_EST_COLOR = "#0d9488"      # estimated pose trail / robot
_PARTICLE_COLOR = "#2563eb"
_LANDMARK_COLOR = "#ca8a04"
_LOOP_COLOR = "#dc2626"


@dataclass
class StepFrame:
    """One step_observed event: the GT pose and the observation RESTORED to world
    frame with that step's GT pose (scan points arrive in the robot frame; landmark
    obs carry polar bearing/range — both are untransformed here, per the contract)."""

    order: float  # normalized seq of the step_observed event
    t: int
    gt: tuple[float, float, float]
    points: list[Point]  # scan hits or restored landmark observation points


@dataclass
class PoseEst:
    order: float
    pose: tuple[float, float, float]
    cov: tuple[float, float, float] | None


@dataclass
class Cloud:
    """particles_updated payload at its normalized seq (x, y, theta, w per row)."""

    order: float
    particles: list[tuple[float, float, float, float]]


@dataclass
class BeliefFrame:
    order: float
    cells: list[tuple[int, int, float]]  # full-grid p per cell (histogram branch)


@dataclass
class LandmarkFrame:
    order: float
    estimated: list[dict[str, float | int]]


@dataclass
class MapDiff:
    """One map_updated diff — accumulated into the log-odds raster by cutoff."""

    order: float
    cells: list[tuple[int, int, float]]


@dataclass
class Constraint:
    order: float
    i: int
    j: int
    loop: bool | None


@dataclass
class Scene:
    """Draw-ready state extracted from a trace. The scenario map arrives via the
    run_started event's scenario path (resolved by build_scene)."""

    grid: OccupancyGrid2D
    algorithm: str = ""
    gt_landmarks: list[Point] = field(default_factory=list)
    steps: list[StepFrame] = field(default_factory=list)
    poses_est: list[PoseEst] = field(default_factory=list)
    clouds: list[Cloud] = field(default_factory=list)
    beliefs: list[BeliefFrame] = field(default_factory=list)
    landmark_frames: list[LandmarkFrame] = field(default_factory=list)
    map_diffs: list[MapDiff] = field(default_factory=list)
    constraints: list[Constraint] = field(default_factory=list)
    trajectory: tuple[list[tuple[float, float, float]], float] | None = None  # (poses, order)
    metrics: dict[str, float] | None = None


def _read_events(trace_path: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with open(trace_path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _restore_scan(gt: tuple[float, float, float], points: list[Point]) -> list[Point]:
    """Robot-frame scan endpoints -> world with the step's GT pose (the contract's
    inverse of the sim's robot-frame storage — replay always restores with GT)."""
    gx, gy, th = gt
    cos_t, sin_t = math.cos(th), math.sin(th)
    return [
        (gx + cos_t * rx - sin_t * ry, gy + sin_t * rx + cos_t * ry) for (rx, ry) in points
    ]


def _restore_obs(
    gt: tuple[float, float, float], obs: list[dict[str, Any]]
) -> list[Point]:
    """Landmark observations (id, bearing, range) -> world points via the GT pose."""
    gx, gy, th = gt
    return [
        (gx + o["range"] * math.cos(th + o["bearing"]),
         gy + o["range"] * math.sin(th + o["bearing"]))
        for o in obs
    ]


def build_scene(trace_path: str) -> Scene:
    events = _read_events(trace_path)
    scenario_path: str | None = None
    algorithm = ""
    gt_landmarks: list[Point] = []
    for ev in events:
        if ev.get("event") == "run_started":
            scenario_path = str(ev["scenario"])
            algorithm = str(ev.get("algorithm", ""))
            gt_landmarks = [(float(p[0]), float(p[1])) for p in (ev.get("landmarks") or [])]
            break
    if scenario_path is None:
        raise SystemExit(f"{trace_path}: no run_started event — cannot resolve the map")

    from slam.maps.loader import load_scenario

    grid = load_scenario(scenario_path).grid
    scene = Scene(grid=grid, algorithm=algorithm, gt_landmarks=gt_landmarks)

    total = len(events) or 1
    for ev in events:
        order = ev["seq"] / (total - 1) if total > 1 else 1.0
        kind = ev.get("event")
        if kind == "step_observed":
            gt = (float(ev["gt"][0]), float(ev["gt"][1]), float(ev["gt"][2]))
            points: list[Point] = []
            if ev.get("scan") is not None:
                points = _restore_scan(
                    gt, [(float(p[0]), float(p[1])) for p in ev["scan"]]
                )
            elif ev.get("obs") is not None:
                points = _restore_obs(gt, ev["obs"])
            scene.steps.append(StepFrame(order, int(ev["t"]), gt, points))
        elif kind == "pose_estimated":
            cov = ev.get("cov")
            scene.poses_est.append(
                PoseEst(
                    order,
                    (float(ev["pose"][0]), float(ev["pose"][1]), float(ev["pose"][2])),
                    None if cov is None else (float(cov[0]), float(cov[1]), float(cov[2])),
                )
            )
        elif kind == "particles_updated":
            scene.clouds.append(
                Cloud(order, [(p[0], p[1], p[2], p[3]) for p in ev["particles"]])
            )
        elif kind == "belief_updated":
            scene.beliefs.append(
                BeliefFrame(order, [(c[0], c[1], c[2]) for c in ev["cells"]])
            )
        elif kind == "landmarks_updated":
            scene.landmark_frames.append(LandmarkFrame(order, list(ev["estimated"])))
        elif kind == "map_updated":
            scene.map_diffs.append(
                MapDiff(order, [(c[0], c[1], c[2]) for c in ev["cells"]])
            )
        elif kind == "constraint_added":
            loop = ev.get("loop")
            scene.constraints.append(
                        Constraint(order, int(ev["i"]), int(ev["j"]),
                                   None if loop is None else bool(loop))
            )
        elif kind == "trajectory_found":
            scene.trajectory = (
                [(p[0], p[1], p[2]) for p in ev["poses"]], order
            )
        elif kind == "run_finished":
            scene.metrics = dict(ev.get("metrics", {}))
    return scene


def to_display(
    p: Point, origin: tuple[float, float], resolution: float, height: int
) -> tuple[float, float]:
    """world point -> display cell units — the RAW world/res transform (y UP).

    The raster is drawn flipud + origin="lower", which already places cell row r on
    the band [h-1-r, h-r) — exactly that cell's own world band — so a point lands on
    its own cell only with the raw transform (mirrors web GridCanvas: same map, same
    points). A second flip here would mirror points onto mirrored cells — invisible
    on symmetric maps, wrong everywhere else (test_replay pins both halves)."""
    return ((p[0] - origin[0]) / resolution, (p[1] - origin[1]) / resolution)


def draw(ax: Axes, scene: Scene, cutoff: float) -> None:
    """Render the accumulated state at normalized seq cutoff in [0, 1]."""
    import numpy as np
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Ellipse

    ax.clear()
    grid = scene.grid
    h, w = grid.height, grid.width
    ox, oy = grid.origin
    res = grid.resolution

    def disp(p: Point) -> tuple[float, float]:
        return to_display(p, (ox, oy), res, h)

    grid_cmap = LinearSegmentedColormap.from_list("slam_grid", ["#0f172a", "#e2e8f0"])
    ax.imshow(
        np.flipud(grid.free_mask().astype(float)), cmap=grid_cmap, origin="lower",
        extent=(0, w, 0, h), vmin=0.0, vmax=1.0, interpolation="nearest", zorder=1,
    )

    # Belief (histogram branch): the latest FULL belief grid at or before cutoff —
    # a probability raster, drawn as an accent heat over the GT map.
    visible_belief = [b for b in scene.beliefs if b.order <= cutoff]
    if visible_belief:
        arr = np.full((h, w), np.nan)
        for (r, c, prob) in visible_belief[-1].cells:
            arr[int(r), int(c)] = prob
        heat = LinearSegmentedColormap.from_list("slam_belief", ["#f8fafc", "#2563eb"])
        ax.imshow(np.flipud(arr), cmap=heat, origin="lower", extent=(0, w, 0, h),
                  vmin=0.0, vmax=1.0, interpolation="nearest", alpha=0.55, zorder=2)

    # Log-odds map (mapping branch): accumulate every diff at or before cutoff;
    # l -> p = sigmoid(l) so both branches share one heat rendering.
    visible_diffs = [m for m in scene.map_diffs if m.order <= cutoff]
    if visible_diffs:
        acc: dict[tuple[int, int], float] = {}
        for diff in visible_diffs:
            for (r, c, l) in diff.cells:
                acc[(int(r), int(c))] = l
        arr = np.full((h, w), np.nan)
        for ((r, c), l) in acc.items():
            arr[r, c] = 1.0 / (1.0 + math.exp(-l))
        heat = LinearSegmentedColormap.from_list("slam_map", ["#f8fafc", "#7c3aed"])
        ax.imshow(np.flipud(arr), cmap=heat, origin="lower", extent=(0, w, 0, h),
                  vmin=0.0, vmax=1.0, interpolation="nearest", alpha=0.55, zorder=2)

    # GT trail (accumulated polyline of step gt poses) + the current GT robot.
    steps_visible = [s for s in scene.steps if s.order <= cutoff]
    if steps_visible:
        pts = [disp((s.gt[0], s.gt[1])) for s in steps_visible]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=_GT_COLOR, lw=1.2,
                alpha=0.55, ls="--", solid_capstyle="round", zorder=4)
        last = steps_visible[-1]
        gx, gy = disp((last.gt[0], last.gt[1]))
        th = last.gt[2]
        ax.scatter([gx], [gy], s=70, color=_GT_COLOR, edgecolors="white",
                   linewidths=1.2, zorder=8)
        ux, uy = disp((last.gt[0] + math.cos(th), last.gt[1] + math.sin(th)))
        ax.annotate("", xy=(ux, uy), xytext=(gx, gy), zorder=8,
                    arrowprops={"arrowstyle": "-", "color": _GT_COLOR, "lw": 1.4})

    # Current scan fan / landmark hits: ONLY the latest visible step's points (the
    # accumulated map/belief layers already carry history).
    if steps_visible and steps_visible[-1].points:
        gx, gy = disp(steps_visible[-1].gt[:2])
        for p in steps_visible[-1].points:
            u, v = disp(p)
            ax.plot([gx, u], [gy, v], color="#94a3b8", lw=0.4, alpha=0.5, zorder=3)
            ax.scatter([u], [v], s=2, color="#334155", edgecolors="none", zorder=3)

    # Estimated trail + robot + covariance ellipse (latest visible estimate).
    est_visible = [p for p in scene.poses_est if p.order <= cutoff]
    if est_visible:
        pts = [disp((p.pose[0], p.pose[1])) for p in est_visible]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=_EST_COLOR, lw=1.6,
                alpha=0.95, solid_capstyle="round", zorder=5)
        ex, ey = disp(est_visible[-1].pose[:2])
        eth = est_visible[-1].pose[2]
        ax.scatter([ex], [ey], s=70, color=_EST_COLOR, edgecolors="white",
                   linewidths=1.2, zorder=9)
        ux, uy = disp((est_visible[-1].pose[0] + math.cos(eth),
                       est_visible[-1].pose[1] + math.sin(eth)))
        ax.annotate("", xy=(ux, uy), xytext=(ex, ey), zorder=9,
                    arrowprops={"arrowstyle": "-", "color": _EST_COLOR, "lw": 1.4})
        if est_visible[-1].cov is not None:
            sx, sy, _st = est_visible[-1].cov
            ax.add_patch(Ellipse((ex, ey), width=2 * sx / res, height=2 * sy / res,
                                 fill=False, edgecolor=_EST_COLOR, lw=1.0, ls=":",
                                 alpha=0.8, zorder=7))

    # Particle cloud (latest visible): size fixed, alpha = w*N so dominant particles
    # read strong and dead ones fade out.
    clouds_visible = [c for c in scene.clouds if c.order <= cutoff]
    if clouds_visible:
        cloud = clouds_visible[-1].particles
        n = len(cloud) or 1
        xs, ys, weights = [], [], []
        for (x, y, _th, weight) in cloud:
            u, v = disp((x, y))
            xs.append(u)
            ys.append(v)
            # w*N averages 1 over the cloud — color-mapping it makes dominant
            # particles read strong and dead ones fade out.
            weights.append(min(1.0, weight * n))
        ax.scatter(xs, ys, s=6, c=np.asarray(weights), cmap="Blues", vmin=0.0,
                   vmax=1.0, edgecolors="none", alpha=0.9, zorder=6)

    # Landmarks: GT x marks (always all of them — they are scenario truth) + the
    # latest estimated set with per-axis error bars when present.
    if scene.gt_landmarks:
        pts = [disp(p) for p in scene.gt_landmarks]
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], marker="x", s=45,
                   color="#64748b", linewidths=1.6, zorder=7)
    lm_visible = [f for f in scene.landmark_frames if f.order <= cutoff]
    if lm_visible:
        for entry in lm_visible[-1].estimated:
            u, v = disp((float(entry["x"]), float(entry["y"])))
            sx_err: float | None = float(entry["sx"]) / res if "sx" in entry else None
            sy_err: float | None = float(entry["sy"]) / res if "sy" in entry else None
            ax.scatter([u], [v], marker="+", s=60, color=_LANDMARK_COLOR,
                       linewidths=1.8, zorder=7)
            if sx_err is not None and sy_err is not None:
                ax.errorbar([u], [v], xerr=[[sx_err], [sx_err]], yerr=[[sy_err], [sy_err]],
                            fmt="none", ecolor=_LANDMARK_COLOR, elinewidth=0.8,
                            capsize=1.5, alpha=0.7, zorder=6)

    # Batch trajectory (graph_based): the whole polyline appears at its event seq;
    # constraints draw between node poses once both endpoints exist on screen.
    if scene.trajectory is not None and scene.trajectory[1] <= cutoff:
        poses = [disp(p[:2]) for p in scene.trajectory[0]]
        ax.plot([p[0] for p in poses], [p[1] for p in poses], color=_EST_COLOR,
                lw=1.8, alpha=0.95, solid_capstyle="round", zorder=6)
    if scene.trajectory is not None and scene.trajectory[1] <= cutoff:
        traj = scene.trajectory[0]
        for con in scene.constraints:
            if con.order > cutoff or con.j >= len(traj) or con.i >= len(traj):
                continue
            a, b = disp(traj[con.i][:2]), disp(traj[con.j][:2])
            if con.loop is None:  # odometry chain edge — faint
                ax.plot([a[0], b[0]], [a[1], b[1]], color="#94a3b8", lw=0.7,
                        alpha=0.5, zorder=5)
            else:  # loop closure — the red chord
                ax.plot([a[0], b[0]], [a[1], b[1]], color=_LOOP_COLOR, lw=1.6,
                        ls="--", alpha=0.9, solid_capstyle="round", zorder=5)

    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect("equal")
    ax.axis("off")
    label = scene.algorithm or "trace"
    if steps_visible:
        label += f"  t={steps_visible[-1].t}"
    if cutoff >= 1.0 and scene.metrics is not None:
        text = "\n".join(f"{k} {v:.4f}" for k, v in scene.metrics.items())
        ax.text(0.02, 0.98, text, transform=ax.transAxes, va="top", family="monospace",
                fontsize=8, bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cbd5e1"})
    ax.set_title(label)


def main() -> None:
    parser = argparse.ArgumentParser(description="replay a slam trace over its map")
    parser.add_argument("trace", help="trace .jsonl path")
    parser.add_argument("--save", default=None, help="write the final frame to a PNG")
    parser.add_argument("--gif", default=None, help="animated replay (sweep + hold)")
    parser.add_argument("--snapshots", default=None, help="directory for evenly-spaced PNGs")
    args = parser.parse_args()

    scene = build_scene(args.trace)
    os.environ.setdefault("MPLBACKEND", "Agg" if (args.save or args.gif or args.snapshots) else "")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    fig_size = (6.0, 6.0)

    def new_fig() -> tuple[Any, Axes]:
        return plt.subplots(figsize=fig_size)

    if args.gif:
        fps = 30
        hold_frames = fps * _HOLD_SECONDS
        total_frames = _TARGET_FRAMES + hold_frames
        fig, ax = new_fig()

        def update(i: int) -> list[Axes]:
            draw(ax, scene, min(i + 1, _TARGET_FRAMES) / _TARGET_FRAMES)
            return [ax]

        anim = FuncAnimation(fig, update, frames=total_frames, blit=False)
        Path(args.gif).parent.mkdir(parents=True, exist_ok=True)
        anim.save(args.gif, writer=PillowWriter(fps=fps))
        plt.close(fig)
        print(f"gif: {args.gif}")

    if args.snapshots:
        out = Path(args.snapshots)
        out.mkdir(parents=True, exist_ok=True)
        for i in range(_DEFAULT_SNAPSHOTS):
            cutoff = (i + 1) / _DEFAULT_SNAPSHOTS
            fig, ax = new_fig()
            draw(ax, scene, cutoff)
            fig.savefig(out / f"frame_{i:02d}.png", dpi=110, bbox_inches="tight")
            plt.close(fig)
        print(f"snapshots: {out}")

    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        fig, ax = new_fig()
        draw(ax, scene, 1.0)
        fig.savefig(args.save, dpi=140, bbox_inches="tight")
        plt.close(fig)
        print(f"saved: {args.save}")

    if not (args.save or args.gif or args.snapshots):
        fig, ax = new_fig()
        draw(ax, scene, 1.0)
        plt.show()


if __name__ == "__main__":
    main()
