"""grid_mapping — log-odds occupancy mapping over GIVEN odometry (Thrun, Burgard &
Fox 2005 ch. 9; the idea is Moravec 1988 / Elfes 1989 in log-odds form).

This is the OTHER half of the filtering branch: the histogram filter put its belief
over the ROBOT on a given map; this puts a belief over the MAP on a given pose.
The recursion per cell is the same Bayes product, but cells are independent (the
naive independence assumption IS the model) so each keeps one scalar log-odds:

    l_t(m) = l_{t−1}(m) + log p(z_t | m occupied)/p(z_t | m free)

- INVERSE MODEL: a scan point is an endpoint, so it says two things — the cell it
  lands in was OCCUPIED (probability p_hit) and every cell before it along the ray
  was FREE (probability p_free = p_hit, symmetric by contract). A missed beam emits
  no point, so it updates NOTHING: unknown cells keep their prior log-odds 0
  (= probability 0.5, which never counts as occupied). With a symmetric model both
  increments carry the same magnitude L = log(p/(1−p)): hit +L, pass-through −L.
  A noisy endpoint that lands one cell short of the wall is not special-cased —
  that free cell collects −log-odds from every beam passing through it on the way
  to real walls, so a few stray hits never flip it.
- POSE IS GIVEN: this branch does not estimate pose — it integrates the scenario's
  NOISELESS odometry onto a DECLARED initial pose (x0/y0/theta_deg parameters). The
  anchor equals the ground-truth start bit-for-bit, so every estimated pose equals
  its ground truth up to float rounding and ATE/RPE read ~0: this page is about the
  map, not the pose. What blurs a real mapping-while-localizing run (drift smearing
  walls) is what the filter_based branch adds on top — here the residual blur is
  only σ_range noise on each endpoint landing inside or one cell short of a face.
- WHY IoU < 1 ANYWAY: thick obstacles' interiors and fully enclosed corner cells
  are physically unobservable (a beam stops at the first occupied cell it enters),
  so those cells keep their prior forever. unknown ≠ occupied is half the lesson.
- RAY → CELLS (cells_along): the segment from the estimated pose point to the
  endpoint point, walked cell-by-cell in traversal order with the same DDA step
  rule as the simulator's raycast (tie takes Y; an axis whose delta is 0 never
  steps). The endpoint's cell is the hit cell (+L); cells before it get −L. If the
  walk leaves the raster before reaching the endpoint's cell, the last in-bounds
  cell absorbs the +L — beyond the raster no cell exists to update. A point whose
  start cell itself is off-raster updates nothing (impossible with exact pose).

Everything below is a fixed operation order so C++ mirrors it bit-for-bit: points
in scan order, cells near→far per walk, per-cell accumulation in arrival order,
touched set emitted row-major sorted.
"""

from __future__ import annotations

import math

import numpy as np

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose, robot_to_world
from ..core.params import ParamSet
from ..core.types import Cell, EstimateResult, LogOddsGrid, Point, Pose


def cells_along(grid, from_pt: Point, to_pt: Point) -> list[Cell]:
    """Cells entered along the CLOSED segment from_pt → to_pt, in traversal order —
    the last one is the endpoint's cell (the "hit"). Fixed DDA arithmetic mirrors
    core/sim.raycast's step rule on the raw segment vector (no angle roundtrip):
    tMax starts at the ray parameter of the next grid-line crossing and advances by
    res/|d|; an axis with delta 0 never steps; a tie (tMaxX == tMaxY) takes Y. The
    walk terminates because the endpoint lies inside the target cell — or it leaves
    the raster, and the last in-bounds cell becomes the hit."""
    x0, y0 = from_pt
    dx = to_pt[0] - x0
    dy = to_pt[1] - y0
    h = grid.height  # row index grows downward — only the height enters tMax
    r, c = grid.world_to_cell(x0, y0)
    if not grid.in_bounds(r, c):
        return []  # the estimate itself sits off-raster — nothing exists out there
    target = grid.world_to_cell(to_pt[0], to_pt[1])
    cells: list[Cell] = [(r, c)]
    if (r, c) == target:
        return cells  # degenerate segment (or same cell): the start IS the hit

    ox, oy = grid.origin
    res = grid.resolution
    if dx > 0.0:
        step_x, t_max_x = 1, (ox + (c + 1) * res - x0) / dx
    elif dx < 0.0:
        step_x, t_max_x = -1, (ox + c * res - x0) / dx
    else:
        step_x, t_max_x = 0, float("inf")
    if dy > 0.0:
        step_y, t_max_y = -1, (oy + (h - r) * res - y0) / dy  # row index grows downward
    elif dy < 0.0:
        step_y, t_max_y = 1, (oy + (h - 1 - r) * res - y0) / dy
    else:
        step_y, t_max_y = 0, float("inf")
    delta_x = res / abs(dx) if dx != 0.0 else float("inf")
    delta_y = res / abs(dy) if dy != 0.0 else float("inf")

    while True:
        if t_max_x < t_max_y:
            c += step_x
            t_max_x += delta_x
        else:
            r += step_y
            t_max_y += delta_y
        if not grid.in_bounds(r, c):
            return cells  # left the raster — last in-bounds cell absorbs the hit
        cells.append((r, c))
        if (r, c) == target:
            return cells


class GridMapping(Estimator):
    """Independent per-cell log-odds occupancy over odometry-integrated poses."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._p_hit = params.get_float("p_hit")
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._theta_deg = params.get_float("theta_deg")
        # The single log-odds increment magnitude (symmetric model, module docstring).
        self._logit = math.log(self._p_hit / (1.0 - self._p_hit))
        self._l: np.ndarray | None = None  # float64 [H, W], prior 0.0 everywhere
        self._poses: list[Pose] = []

    @property
    def name(self) -> str:
        return "grid_mapping"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    # --- update -------------------------------------------------------------

    def update(self, step, recorder) -> None:
        """Integrate odometry onto the anchor pose, push every scan point's segment
        into the log-odds field, then emit this step's events (pose first, map diff
        second — same event order as the filtering branch)."""
        episode = self.episode
        assert episode is not None
        grid = episode.grid
        if self._l is None:  # first update: allocate on the ground-truth raster
            self._l = np.zeros((grid.height, grid.width), dtype=np.float64)

        if step.odom is None:  # t = 0 — the declared anchor IS the pose
            pose = Pose(self._x0, self._y0, math.radians(self._theta_deg))
        else:
            pose = pose_compose(self._poses[-1], step.odom)
        self._poses.append(pose)

        if step.scan is not None:
            hit_l = self._logit
            field = self._l
            touched: set[Cell] = set()
            for z in step.scan:  # scan order (beam ascending) — fixed accumulation order
                w_pt = robot_to_world(z, pose)
                cells = cells_along(grid, (pose.x, pose.y), w_pt)
                if not cells:
                    continue
                for cell in cells[:-1]:  # passed through → free evidence
                    field[cell[0]][cell[1]] -= hit_l
                    touched.add(cell)
                hr, hc = cells[-1]      # the endpoint's cell → occupied evidence
                field[hr][hc] += hit_l
                touched.add((hr, hc))
            if recorder is not None:
                recorder.pose_estimated(step.t, pose)
                # Row-major sorted emission (sorted tuples sort by row then col —
                # identical order to a C++ std::set<std::pair> walk).
                recorder.map_updated(
                    step.t, [(r, c, float(field[r][c])) for (r, c) in sorted(touched)]
                )
            return
        if recorder is not None:  # no scan (impossible on beam scenarios, honest anyway)
            recorder.pose_estimated(step.t, pose)

    def finalize(self, recorder) -> EstimateResult:
        episode = self.episode
        assert episode is not None and self._l is not None
        return EstimateResult(
            poses=tuple(self._poses),
            grid=LogOddsGrid(
                resolution=episode.grid.resolution,
                origin=episode.grid.origin,
                log_odds=self._l,
            ),
        )
