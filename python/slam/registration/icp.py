"""icp — point-to-point Iterative Closest Point (Besl & McKay, TPAMI 1992) run as
pairwise scan matching: the registration branch's first member and its whole idea —
pose from measurements alone, odometry without wheels.

The filtering branch rode on odometry; this branch replaces it. There is no map to
match against either (that is the graph_based branch's move): the reference cloud is
the PREVIOUS scan. At every step t ≥ 1 the arriving scan is the source, the previous
scan is the target, and one ICP solve returns the SE(2) element that carries one onto
the other; composing it onto the running estimate integrates motion into a pose. The
absolute frame is not measured — it is DECLARED (x0/y0/theta_deg are the gauge, the
same convention grid_mapping anchors its map on), and everything after step 0 comes
from scans alone. Step 0 emits that declared pose unchanged: there is no earlier scan
to align against.

THE SOLVE (fixed operation order — C++ and TS mirror it bit-for-bit / ulp-for-ulp).
Source points are walked in scan order; each is transformed by the CURRENT estimate,
matched to the target point minimizing the squared distance (ascending index scan,
strict `<` — a tie keeps the lower index), and kept only if that distance is ≤ d_max
under the current estimate. That truncation IS Besl & McKay's "truncated least
squares": points whose true pair left the other scan (new structure entering the fan)
have no honest partner, and an untruncated sum would let their wrong pairs drag the
solution toward identity. With the kept pairs (q_j → p_j) the point-to-point error is
minimized in CLOSED form — in 2D the SVD of Besl & McKay collapses to a centroid
alignment plus one atan2:

    dθ = atan2( Σ zx_j·wy_j − zy_j·wx_j ,  Σ zx_j·wx_j + zy_j·wy_j )
    dt = p̄ − R(dθ)·q̄          (centroids over the KEPT pairs, ascending sums)

with z = q − q̄, w = p − p̄. The loop repeats (re-pair under the new estimate) until
the pose update stops moving (max(|Δdx|, |Δdy|, |dθ|) ≤ eps) or max_iters is spent;
convergence is a fixed point of "pair then solve", not a guarantee — which is exactly
what the scenario below makes visible.

WHY THE SCENARIO IS AN EMPTY ROOM (this is the algorithm's honest limit, measured,
not decoration): NN correspondence needs the source point's PHYSICAL twin present in
the target scan. A fin the robot passes through violates this at the passing step —
its far face was occluded (the fan points forward) while it mattered, so its points
in the new scan have no true pair; they match whatever sits at the same ROBOT-frame
position in the old scan, and those wrong pairs pull the estimate toward identity.
Measured: with fins in the corridor, dx error at the passing step equals the full
step length. In an empty box the only structures are the end wall (always in range,
head-on — its points' twins always exist) and the two long side walls; their lattice
points re-pair to their own physical twin under the true twist and contribute noise
of order (r·Δβ)/√k instead of bias. And a straight line is not a limitation nobody
noticed: an instantaneous 90° turn has no shared physical points at all, so NO
correspondence rule can recover it — rotation in this demo only ever comes from the
lever arm of points spread along the walls it passes.

The residual error floor is honest too: beams sample angle discretely (1° here), so
even a noise-free scan quantizes where along a wall its points sit — the demo's σ=0
run lands at ATE ≈ 0.003 m, not 0. Everything above that floor is σ_range and the
quantization; both are visible in the numbers.

Everything below is a fixed operation order so C++ mirrors it exactly: source walked
in scan order, ascending strict-`<` nearest search, squared-distance comparison
against d_max² (no sqrt anywhere), ascending centroid sums, one atan2, and the
convergence check max(|Δdx|, |Δdy|, |Δθ|) ≤ eps evaluated after each update. An
empty source or target scan, or a step whose every pair was truncated away, returns
the identity twist — the documented degenerate case: no information, no motion.
"""

from __future__ import annotations

import math

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose
from ..core.params import ParamSet
from ..core.trace import TraceRecorder
from ..core.types import EstimateResult, Point, Pose, Step, Twist


def icp_step(
    source: tuple[Point, ...],
    target: tuple[Point, ...],
    d_max: float,
    eps: float,
    max_iters: int,
) -> Twist:
    """One ICP solve: the SE(2) element carrying `source` onto `target`.

    Fixed order (module docstring): transform each source point by the current
    estimate (cos first), find its nearest target point by squared distance in
    ascending index order (strict `<` keeps the lower index on a tie), keep the pair
    iff that squared distance ≤ d_max², then close-form solve over the kept pairs.
    Iterate until the update stops moving or max_iters is spent. Empty inputs or zero
    kept pairs return the identity twist — and once a step has nothing left to align,
    the current estimate is simply carried forward."""
    if not source or not target:
        return Twist(0.0, 0.0, 0.0)
    dm2 = d_max * d_max
    ax = 0.0
    ay = 0.0
    ang = 0.0
    for _ in range(max_iters):
        ca = math.cos(ang)
        sa = math.sin(ang)
        qx: list[float] = []
        qy: list[float] = []
        px: list[float] = []
        py: list[float] = []
        for (qx_, qy_) in source:  # ascending scan order — fixed
            tx = ca * qx_ - sa * qy_ + ax
            ty = sa * qx_ + ca * qy_ + ay
            best_d = (tx - target[0][0]) ** 2 + (ty - target[0][1]) ** 2
            best_j = 0
            for j in range(1, len(target)):
                dd = (tx - target[j][0]) ** 2 + (ty - target[j][1]) ** 2
                if dd < best_d:  # strict — a tie keeps the lower index
                    best_d = dd
                    best_j = j
            if best_d <= dm2:
                qx.append(qx_)
                qy.append(qy_)
                px.append(target[best_j][0])
                py.append(target[best_j][1])
        n = len(qx)
        if n == 0:
            return Twist(ax, ay, ang)  # nothing left to align — carry the estimate
        qx_bar = 0.0
        qy_bar = 0.0
        px_bar = 0.0
        py_bar = 0.0
        for j in range(n):  # ascending centroid sums — fixed
            qx_bar += qx[j]
            qy_bar += qy[j]
            px_bar += px[j]
            py_bar += py[j]
        qx_bar /= n
        qy_bar /= n
        px_bar /= n
        py_bar /= n
        num = 0.0
        den = 0.0
        for j in range(n):  # centered cross/dot sums, ascending — fixed
            zx = qx[j] - qx_bar
            zy = qy[j] - qy_bar
            wx = px[j] - px_bar
            wy = py[j] - py_bar
            num += zx * wy - zy * wx
            den += zx * wx + zy * wy
        d_new = math.atan2(num, den)
        cr = math.cos(d_new)
        sr = math.sin(d_new)
        ax_new = px_bar - (cr * qx_bar - sr * qy_bar)
        ay_new = py_bar - (sr * qx_bar + cr * qy_bar)
        moved = max(abs(ax_new - ax), abs(ay_new - ay), abs(d_new - ang))
        ax = ax_new
        ay = ay_new
        ang = d_new
        if moved <= eps:
            break
    return Twist(ax, ay, ang)


class Icp(Estimator):
    """Point-to-point ICP scan matching on a known-free straight corridor; the pose
    is the declared gauge composed with every recovered twist (odometry never read)."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._theta_deg = params.get_float("theta_deg")
        self._d_max = params.get_float("d_max")
        self._eps = params.get_float("eps")
        self._max_iters = params.get_int("max_iters")
        self._pose: Pose | None = None  # set at t = 0 to the declared gauge pose
        self._prev: tuple[Point, ...] | None = None  # the previous step's scan
        self._poses: list[Pose] = []  # readout history (index == step t)

    @property
    def name(self) -> str:
        return "icp"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        """t = 0 adopts the declared gauge pose (registration measures RELATIVE
        motion — the absolute frame is a declaration, not a measurement). Every later
        step aligns the arriving scan onto the previous one and composes the result;
        Step.odom is never read — that is the branch. A step without a scan carries
        the estimate forward untouched (the documented degenerate case)."""
        if self._pose is None:
            self._pose = Pose(self._x0, self._y0, math.radians(self._theta_deg))
            self._prev = step.scan if step.scan is not None else ()
        else:
            twist = icp_step(
                step.scan if step.scan is not None else (),
                self._prev if self._prev is not None else (),
                self._d_max, self._eps, self._max_iters,
            )
            self._pose = pose_compose(self._pose, twist)
            if step.scan is not None:
                self._prev = step.scan
        self._poses.append(self._pose)
        if recorder is not None:
            # No covariance — ICP carries no uncertainty model (that is what the
            # filter_based branch adds on top of this).
            recorder.pose_estimated(step.t, self._pose)

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        return EstimateResult(poses=tuple(self._poses))
