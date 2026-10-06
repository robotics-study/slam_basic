"""Histogram filter — the Bayes filter of Chapter 2 over a DISCRETE pose space.

The genealogy's root: before particles or graphs, the belief was just a histogram
over quantized states (Thrun, Burgard & Fox, Probabilistic Robotics 2005; the grid
localization formulation goes back to Cowgill 1970 and Elfes' occupancy-grid era).
This module is the complete recursive Bayes filter with nothing hidden:

- STATE: every (free cell × heading lattice point) pair. Occupied cells are not
  states at all — their mass is exactly 0 forever, which is why free-cell indexing
  exists. A state index is s = fi * B + b (fi = row-major index among FREE cells).
- LATTICE: the heading axis discretizes onto the SENSOR's own angular grid. With
  w = radians(fov)/(beams−1) the beams fan out at exactly w radians; extending that
  grid to the full circle gives B = 2π/w heading bins whose centers are the beam
  angles themselves (bin b center θ_b = −π + b·w). This is what makes the filter
  exact where it can be: a scenario whose path drives axis-aligned has its true
  heading ON the lattice, so the state at the true cell and true heading predicts
  every beam ray along the true beam angle — zero model error. A coarse uniform
  heading grid cannot do this: a ±20° quantization flips grazing beams against any
  range precision, and no Bayes update recovers the likelihood from that.
- PRIOR: uniform over all states ("the robot could be in any free cell facing any
  lattice direction") — global localization from nothing but a map.
- LIKELIHOOD (precomputed): for every free cell fi and every lattice angle k one
  DDA raycast gives R[fi][k] = hit distance (miss → sentinel range_max + res). A
  state (cell fi, heading θ_b) facing beam j therefore expects
  H[j][s] = R[fi][(b + j − off) mod B], an exact rotation of the table — building
  it is data movement, not geometry. Each observed point z contributes Gaussian
  log-likelihood −½·((r − H[j][s])/σ)² with r = |z| and beam index j recovered from
  the point's robot-frame bearing. Beams the ground truth never fired emit no point,
  so their table entries are compared against NOTHING (a miss carries no information
  here — the model says so honestly); what discriminates is the other direction: a
  state whose own prediction hits where the ground truth missed eats the full
  Gaussian penalty of its sentinel value on every beam it shares with the scan.
- UPDATE: Bayes product with the prior in log space, p'(s) ∝ p(s)·exp(L(s) − m),
  m = max L — multiplying by the prior is what makes this RECURSIVE (a normalized
  likelihood alone would erase the prediction). If every term underflows to 0 the
  belief stays unchanged (documented degenerate case, identical in C++).
- PREDICT (motion): child-of-movement. Each hypothesis is pushed through the exact
  odometry twist from its own cell-center/lattice-heading pose, quantized back onto
  the lattice (nearest center, ties up), and mixed with a slip probability p_slip
  of staying put. A move whose target cell is occupied or off-grid does not move
  (slide semantics — the heading still rotates). The scenario's commanded twists are
  exact lattice multiples (waypoints on cell centers, step = cell size,
  sigma_theta = 0), so quantization SELF-HEALS: a hypothesis at the true state lands
  back on the true state exactly, and slip is the only uncertainty the filter can
  even see — sub-quantum position noise cannot cross a cell boundary.
- READOUT: position = belief-weighted mean of cell centers; heading = the lattice
  angle with the largest bin marginal (first max wins); cov = population std devs.

Everything below is a fixed operation order so C++ mirrors it bit-for-bit:
observation-major / state-minor accumulation, ascending sums everywhere, scalar
math.exp for every exponential (numpy only for the exact elementwise + - * /).
"""

from __future__ import annotations

import math

import numpy as np

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose, wrap
from ..core.params import ParamSet
from ..core.sim import raycast
from ..core.trace import TraceRecorder
from ..core.types import EstimateResult, Pose, ScanGrid, Step, Twist


class HistogramFilter(Estimator):
    """Exhaustive grid-form Bayes filter over (cell × heading lattice) states."""

    # Set in _init_state (first update — the episode only exists once run() starts).
    _w_bin: float                 # heading lattice step == the sensor's angular step
    _bins: int                    # lattice size B = 2π / w_bin (aligned by config)
    _half: float                  # fov / 2 (radians)
    _off: int                     # lattice offset: half / w_bin ((beams − 1) // 2)
    _grid_h: int
    _grid_w: int
    _n_states: int
    _free_cells: list[tuple[int, int]]   # free cells in row-major order (index = fi)
    _fi_of_cell: list[int]               # cell_index -> free index (-1 for occupied)
    _centers: list[tuple[float, float]]  # world center of each free cell (by fi)
    _h: list[np.ndarray]                 # H[j][s]: expected hit distance per state
    _belief: np.ndarray                  # float64 [n_states], sums to 1.0
    _poses: list[Pose]

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._p_slip = params.get_float("p_slip")
        self._sigma_range = params.get_float("sigma_range")
        self._beams = params.get_int("beams")
        self._fov_deg = params.get_float("fov_deg")
        self._range_max = params.get_float("range_max")
        self._initialized = False

    @property
    def name(self) -> str:
        return "histogram_filter"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    # --- init ---------------------------------------------------------------

    def _init_state(self) -> None:
        """Uniform prior over free cells x the heading lattice + the ray table.

        The heading lattice is the beam grid extended to the full circle (module
        docstring): bin b's center θ_b = −π + b·w equals a possible beam direction,
        so a state facing beam j expects exactly R[fi][(b + j − off) mod B] — the
        ray the simulator fires when the true pose IS that state. One DDA per
        (free cell, lattice angle), ascending (fi, k) order; a miss stores the
        sentinel range_max + resolution."""
        episode = self.episode
        assert episode is not None
        grid: ScanGrid = episode.grid
        h, w, res = grid.height, grid.width, grid.resolution

        step_a = math.radians(self._fov_deg) / float(self._beams - 1)
        half = math.radians(self._fov_deg) / 2.0
        b_bins = int(round(2.0 * math.pi / step_a))
        off = (self._beams - 1) // 2
        # The config must keep the lattice aligned: B steps of w close the circle and
        # off steps of w land exactly on half the field of view.
        assert abs(float(b_bins) * step_a - 2.0 * math.pi) < 1e-9, "heading lattice not closed"
        assert abs(float(off) * step_a - half) < 1e-9, "heading lattice not aligned to beams"

        centers: list[tuple[float, float]] = []  # per free cell (ascending cell index)
        fi_of_cell = [-1] * (h * w)
        for r in range(h):
            for c in range(w):
                if grid.occupied(r, c):
                    continue
                fi_of_cell[r * w + c] = len(centers)
                centers.append(grid.cell_to_world(r, c))

        self._w_bin = step_a
        self._bins = b_bins
        self._half = half
        self._off = off
        self._free_cells = [(r, c) for r in range(h) for c in range(w)
                            if fi_of_cell[r * w + c] >= 0]
        self._fi_of_cell = fi_of_cell
        self._centers = centers
        self._grid_h, self._grid_w = h, w
        n_states = len(centers) * b_bins
        self._n_states = n_states

        # R: one DDA per (free cell, lattice angle), ascending.
        bin_centers = [-math.pi + float(k) * step_a for k in range(b_bins)]
        miss = self._range_max + res
        r_flat: list[float] = []
        for cx, cy in centers:
            for k in range(b_bins):
                hit = raycast(grid, cx, cy, bin_centers[k], self._range_max)
                r_flat.append(miss if hit is None else hit)

        # H[j] per beam j: the same numbers gathered through the rotation identity
        # H[j][fi*B + b] = R[fi][(b + j − off) mod B] (ascending s order).
        h_tables: list[np.ndarray] = []
        for j in range(self._beams):
            col = np.empty(n_states, dtype=np.float64)
            i = 0
            for fi in range(len(centers)):
                base = fi * b_bins
                for b in range(b_bins):
                    col[i] = r_flat[base + (b + j - off) % b_bins]
                    i += 1
            h_tables.append(col)
        self._h = h_tables

        p0 = 1.0 / float(n_states)
        self._belief = np.full(n_states, p0, dtype=np.float64)
        self._poses: list[Pose] = []
        self._initialized = True

    # --- update -------------------------------------------------------------

    def _predict(self, u: Twist) -> None:
        """Child-of-movement per state + slip mix, ascending s order.

        A hypothesis whose commanded move lands on an occupied cell or off the grid
        does not move (slide semantics — its heading still rotates). Exact-zero
        beliefs are skipped; adding 0.0 could not change any sum anyway."""
        w_bin = self._w_bin
        bins = self._bins
        grid = self.episode.grid if self.episode is not None else None
        assert grid is not None
        w = self._grid_w
        p_stay = self._p_slip
        belief = self._belief
        new_p: list[float] = [0.0] * self._n_states
        for fi, (r0, c0) in enumerate(self._free_cells):
            cx, cy = self._centers[fi]
            base = fi * bins
            for b in range(bins):
                p = float(belief[base + b])
                if p == 0.0:
                    continue  # exact zero — adding it could not change any sum
                moved = pose_compose(Pose(cx, cy, -math.pi + float(b) * w_bin), u)
                qr, qc = grid.world_to_cell(moved.x, moved.y)
                qb = int(math.floor((moved.theta + math.pi) / w_bin + 0.5))
                if qb >= bins:
                    qb -= bins
                q_ci = r0 * w + c0
                # Slide semantics: off-grid or occupied target => the move does not move.
                if grid.in_bounds(qr, qc) and self._fi_of_cell[qr * w + qc] >= 0:
                    q_ci = qr * w + qc
                q = self._fi_of_cell[q_ci] * bins + qb
                new_p[q] += (1.0 - p_stay) * p
                new_p[base + b] += p_stay * p
        self._belief = np.array(new_p, dtype=np.float64)

    def _update_scan(self, scan: list[tuple[float, float]]) -> None:
        """Bayes product with the prior (observation-major, state-minor — fixed order).

        Every observed point contributes −½·((r − H[j][s])/σ)² to every state at
        once (numpy elementwise — exact ops only); the exponential itself is scalar
        math.exp so all three languages hit the same libm. States whose prior is
        exactly 0 skip the exp (their weight is +0.0 either way, bit-identically)."""
        sigma = self._sigma_range
        n = self._n_states
        ll = np.zeros(n, dtype=np.float64)
        for (zx, zy) in scan:
            r = math.sqrt(zx * zx + zy * zy)
            beta = math.atan2(zy, zx)
            j = int(math.floor((beta + self._half) / self._w_bin + 0.5))
            if j < 0:
                j = 0
            if j > self._beams - 1:
                j = self._beams - 1
            col = self._h[j]
            d = (r - col) / sigma
            ll += (-0.5 * d) * d
        m = float(ll.max())  # max is exact — order-independent by construction
        prior = self._belief
        w_list: list[float] = []
        total = 0.0
        for s in range(n):
            p_prior = float(prior[s])
            if p_prior == 0.0:
                wv = 0.0
            else:
                wv = p_prior * math.exp(float(ll[s]) - m)
            w_list.append(wv)
            total += wv
        if total != 0.0:
            self._belief = np.array([wv / total for wv in w_list], dtype=np.float64)

    # --- Estimator interface --------------------------------------------------

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        if not self._initialized:
            self._init_state()
        if step.odom is not None:
            self._predict(step.odom)
        if step.scan:
            self._update_scan(list(step.scan))

        bins = self._bins
        belief = self._belief

        # Readout (fixed order): position mean ascending over all states, then the
        # bin-marginal argmax (first max wins), then population std devs.
        x_hat = 0.0
        y_hat = 0.0
        bin_marginal = [0.0] * bins
        for fi in range(len(self._free_cells)):
            cx, cy = self._centers[fi]
            base = fi * bins
            for b in range(bins):
                p = float(belief[base + b])
                x_hat += p * cx
                y_hat += p * cy
                bin_marginal[b] += p
        theta_hat = -math.pi
        best = float("-inf")
        for b in range(bins):
            if bin_marginal[b] > best:
                best = bin_marginal[b]
                theta_hat = -math.pi + float(b) * self._w_bin

        var_x = 0.0
        var_y = 0.0
        var_t = 0.0
        for fi in range(len(self._free_cells)):
            cx, cy = self._centers[fi]
            base = fi * bins
            for b in range(bins):
                p = float(belief[base + b])
                dx = cx - x_hat
                dy = cy - y_hat
                dt = wrap(-math.pi + float(b) * self._w_bin - theta_hat)
                var_x += p * (dx * dx)
                var_y += p * (dy * dy)
                var_t += p * (dt * dt)
        pose = Pose(x_hat, y_hat, theta_hat)
        cov = [math.sqrt(var_x), math.sqrt(var_y), math.sqrt(var_t)]

        if recorder is not None:
            # EVERY cell row-major ([r, c, p] per the trace contract); occupied cells
            # are not states and emit exactly 0.0.
            cells: list[tuple[int, int, float]] = []
            for ci in range(self._grid_h * self._grid_w):
                fi = self._fi_of_cell[ci]
                r0, c0 = divmod(ci, self._grid_w)
                p_cell = 0.0
                if fi >= 0:
                    base = fi * bins
                    for b in range(bins):
                        p_cell += float(belief[base + b])
                cells.append((r0, c0, p_cell))
            recorder.pose_estimated(step.t, pose, cov)
            recorder.belief_updated(step.t, cells)
        self._poses.append(pose)

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        return EstimateResult(poses=tuple(self._poses))
