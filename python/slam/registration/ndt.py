"""ndt — Normal Distributions Transform (Biber & Strasser, IROS 2003) run as
pairwise scan matching: the registration branch's second member and its other idea —
where ICP matched point to point, NDT never matches anything. The reference scan is
compiled into a per-cell probability field, and the arriving scan is SCORED against
that field directly; correspondence is not computed anywhere, it is dissolved into
the field itself. Like Icp, this estimator reads no odometry: step 0 adopts the
declared gauge pose (x0/y0/theta_deg) and every later step composes one recovered
SE(2) element onto the running estimate.

THE FIELD (fixed operation order — C++ and TS mirror it bit-for-bit / ulp-for-ulp).
Target points bucket into cells keyed by (floor(x/cell_size), floor(y/cell_size)) in
scan order; a cell with fewer than 3 points carries no distribution at all, and one
whose scatter determinant is EXACTLY 0.0 (a strictly degenerate cluster) carries none
either. Every remaining cell k stores μ_k = (1/n)Σ x_j and the RAW scatter matrix
S_k = Σ_j (x_j − μ_k)(x_j − μ_k)ᵗ — deliberately not divided by n. The score sums a
Gaussian bump per source point over whatever distribution its transformed position
lands in:

    score(a) = Σ_i exp(−½ · q_iᵀ S_k(i)⁻¹ q_i),   q_i = R(a)x_i + a − μ_k(i)

The paper normalizes (Σ = S/n); this repo does not, and that is the honest design
choice, not an oversight: Σ⁻¹ = n·S⁻¹ sharpens every bump by the cluster's own point
count, and running the SAME scenario with the normalized kernel measures worse —
ATE 0.078 against 0.052, step 2 overshooting to dx = +0.365 against a true +0.25.
The scatter-scaled kernel keeps each blob as wide as the cluster that made it, and
that width is what captures a 0.25 m step cleanly — NDT's capture range is not a
d_max cutoff (ICP's device) but the blob width itself.

THE SOLVE (fixed operation order, same reason). f = −score; with E = exp(−½qᵀS⁻¹q)
and u_j = (S⁻¹q)·J_j for J_1 = e₁, J_2 = e₂, J_3 = ∂R(φ)x/∂φ, the gradient and
Hessian accumulate in scan order as g_j = Σ E·u_j and H_jk = Σ E(−u_j u_k + W_jk)
(W_jk = ∂u_j/∂a_k, written out below). The step solves (H + λI)Δ = −g by Cramer's
rule on the 3×3 system (cofactor expansion along row 0 — fixed); λ starts at lam0 and
doubles while any Sylvester leading principal minor of H + λI is ≤ 0, which terminates
because every entry of H is finite. Far from a peak the surface is flat along the
aliased axis and H goes strongly indefinite — measured at step 1's second iteration:
eigenvalues [−859, +70675], and λ doubles ×30 to ≈ 1.07×10³ before the solve proceeds.
The step then passes a backtracking line search: Δ is accepted only on STRICT score
improvement f(a+Δ) < f(a); otherwise it halves (exact in binary) until it does, and if
max|Δ| ≤ eps without any strict improvement the solver returns the current estimate
unchanged — a fixed point of "Newton then backtrack", which is what convergence means
here, not a guarantee. An empty source or target scan returns the identity twist — no
information, no motion (same contract as Icp).

WHY THE SCENARIO IS A STAIRCASE (this is the algorithm's honest limit, measured, not
decoration): an axis-aligned wall's points spread along its whole visible length, so
their clusters are translation-invariant ALONG the wall — score carries no signal in
that direction, and whether a step recovers at all hangs on coverage-edge effects. Run
this estimator on room01_straight, ICP's home map, and it recovers 6 of the 8 steps
but collapses to identity at step 2 (dx = −0.0011 against a true +0.25): there the
measured gradient is |∂score/∂x| = 0.037 against |∂score/∂y| = 169 and |∂score/∂θ| =
220 — the x signal literally vanishes, the line search finds no improvement along any
halving of that direction, and ATE lands at 0.334 (ICP: 0.021 on the same map). The
staircase in room02_diagonal is the minimum structure that fixes this: every riser
spans exactly one row height, so each cluster is bounded on BOTH axes and both axes
carry signal — inside, the estimator tracks each 0.25 m step to centimeters (the demo
pins dx = +0.2439/+0.2211/+0.2377/+0.2593 per step). The path stops at x = 2.75
because free space on the path row ends at the face x = 3.5; extending it to x = 3.25
is measured too — no collapse, but the last step's y/θ error grows (twist +0.2498 /
+0.0498 / +0.0103, ATE 0.054 against 0.052) as the head-on riser crowds out the treads.

Everything below is a fixed operation order so C++ mirrors it exactly: scan-order
bucketing, ascending sums — explicit accumulation loops, NOT the builtin sum(), which
from CPython 3.12 (gh-100425) switches to Neumaier COMPENSATED summation, a different
arithmetic expression that lands ulps off the plain left-to-right sums the C++ mirror
runs (measured: one mean 1 ulp off was enough to move the demo's ATE at digit 10) —
exp of (−m/2) with m = qᵀS⁻¹q computed as (dxq·u1 + dyq·u2)/2, the λ-doubling loop,
the row-0 cofactor solve, and halving that is exact in binary.
"""

from __future__ import annotations

import math

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose
from ..core.params import ParamSet
from ..core.trace import TraceRecorder
from ..core.types import EstimateResult, Point, Pose, Step, Twist

# One cell's distribution: (μx, μy, Sxx, Sxy, Syy, det S) — mean normalized by n,
# scatter RAW (see module docstring); a cell is absent from the dict when it holds
# fewer than 3 points or its scatter determinant is exactly 0.0.
CellDist = tuple[float, float, float, float, float, float]


def build_ndt(points: tuple[Point, ...], cell_size: float) -> dict[tuple[int, int], CellDist]:
    """Bucket `points` (scan order) into cells keyed by (floor(x/cell), floor(y/cell))
    and fit one distribution per cell. Points are bucketed in scan order and the sums
    below accumulate in that same insertion order — fixed, so every language mirrors.

    A cell with fewer than 3 points carries NO distribution (a 2-point cluster cannot
    span the plane; its scatter would be degenerate anyway) and a scatter matrix whose
    determinant is EXACTLY 0.0 (collinear-on-an-axis hits) also carries none: an
    infinite ridge in one direction is not a measurement of that direction."""
    buckets: dict[tuple[int, int], list[Point]] = {}
    for p in points:
        key = (math.floor(p[0] / cell_size), math.floor(p[1] / cell_size))
        buckets.setdefault(key, []).append(p)
    dists: dict[tuple[int, int], CellDist] = {}
    for key, pts in buckets.items():
        n = len(pts)
        if n < 3:
            continue
        # Explicit ascending accumulation — NOT the builtin sum(): from CPython 3.12
        # (gh-100425) sum() switches to Neumaier compensated summation for floats,
        # which is a DIFFERENT arithmetic expression than "add in scan order" and can
        # land 1 ulp off the plain left-to-right sums the C++ mirror runs. The repo's
        # requires-python spans both semantics, so the loop below is the contract.
        mx = 0.0
        my = 0.0
        for (x, y) in pts:
            mx += x
            my += y
        mx /= n
        my /= n
        sxx = 0.0
        syy = 0.0
        sxy = 0.0
        for (x, y) in pts:
            dx = x - mx
            dy = y - my
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy
        det = sxx * syy - sxy * sxy
        if det == 0.0:
            continue
        dists[key] = (mx, my, sxx, sxy, syy, det)
    return dists


def ndt_score(
    source: tuple[Point, ...],
    dists: dict[tuple[int, int], CellDist],
    cell_size: float,
    ax: float,
    ay: float,
    ang: float,
) -> float:
    """score(a): raw sum of Gaussian bumps, one per source point, in scan order. A
    source point whose transformed position lands in a cell with no distribution
    contributes exactly 0. The quadratic form is evaluated as (dxq·u1 + dyq·u2)/2
    with u = S⁻¹q — the same expression the gradient below differentiates."""
    s = 0.0
    c = math.cos(ang)
    sn = math.sin(ang)
    for (qx_, qy_) in source:
        x = c * qx_ - sn * qy_ + ax
        y = sn * qx_ + c * qy_ + ay
        d = dists.get((math.floor(x / cell_size), math.floor(y / cell_size)))
        if d is None:
            continue
        mx, my, sxx, sxy, syy, det = d
        ixx = syy / det
        iyy = sxx / det
        ixy = -sxy / det
        dxq = x - mx
        dyq = y - my
        u1 = ixx * dxq + ixy * dyq
        u2 = ixy * dxq + iyy * dyq
        s += math.exp(-(dxq * u1 + dyq * u2) / 2.0)
    return s


def ndt_step(
    source: tuple[Point, ...],
    target: tuple[Point, ...],
    cell_size: float,
    lam0: float,
    eps: float,
    max_iters: int,
) -> Twist:
    """One NDT solve: the SE(2) element carrying `source` onto the field of `target`.

    Fixed order (module docstring): build the field from `target`, then iterate —
    accumulate g and H in scan order; double λ from lam0 while any Sylvester leading
    principal minor of H + λI is ≤ 0; solve (H + λI)Δ = −g by cofactor expansion along
    row 0; accept the step only on STRICT score improvement, halving Δ (exact in
    binary) until it improves or max|Δ| ≤ eps forces a fixed point. Empty input on
    either side returns the identity twist."""
    if not source or not target:
        return Twist(0.0, 0.0, 0.0)
    dists = build_ndt(target, cell_size)
    ax = 0.0
    ay = 0.0
    ang = 0.0
    for _ in range(max_iters):
        g = [0.0] * 3
        h = [[0.0] * 3 for _ in range(3)]
        ca = math.cos(ang)
        sa = math.sin(ang)
        for (qx_, qy_) in source:  # ascending scan order — fixed
            x = ca * qx_ - sa * qy_ + ax
            y = sa * qx_ + ca * qy_ + ay
            d = dists.get((math.floor(x / cell_size), math.floor(y / cell_size)))
            if d is None:
                continue
            mx, my, sxx, sxy, syy, det = d
            ixx = syy / det
            iyy = sxx / det
            ixy = -sxy / det
            dxq = x - mx
            dyq = y - my
            u1 = ixx * dxq + ixy * dyq
            u2 = ixy * dxq + iyy * dyq
            e = math.exp(-(dxq * u1 + dyq * u2) / 2.0)
            j3x = -sa * qx_ - ca * qy_
            j3y = ca * qx_ - sa * qy_
            qddx = -(ca * qx_ - sa * qy_)
            qddy = -(sa * qx_ + ca * qy_)
            u3 = j3x * u1 + j3y * u2
            w11 = ixx
            w12 = ixy
            w22 = iyy
            w13 = ixx * j3x + ixy * j3y
            w23 = ixy * j3x + iyy * j3y
            w33 = (qddx * u1 + qddy * u2) + (
                j3x * (ixx * j3x + ixy * j3y) + j3y * (ixy * j3x + iyy * j3y)
            )
            g[0] += e * u1
            g[1] += e * u2
            g[2] += e * u3
            h[0][0] += e * (-u1 * u1 + w11)
            h[0][1] = h[1][0] = h[0][1] + e * (-u1 * u2 + w12)
            h[0][2] = h[2][0] = h[0][2] + e * (-u1 * u3 + w13)
            h[1][1] += e * (-u2 * u2 + w22)
            h[1][2] = h[2][1] = h[1][2] + e * (-u2 * u3 + w23)
            h[2][2] += e * (-u3 * u3 + w33)
        lam = lam0
        while True:
            hl = [[h[i][j] + (lam if i == j else 0.0) for j in range(3)] for i in range(3)]
            m1 = hl[0][0]
            m2 = hl[0][0] * hl[1][1] - hl[0][1] * hl[1][0]
            det = (
                hl[0][0] * (hl[1][1] * hl[2][2] - hl[1][2] * hl[2][1])
                - hl[0][1] * (hl[1][0] * hl[2][2] - hl[1][2] * hl[2][0])
                + hl[0][2] * (hl[1][0] * hl[2][1] - hl[1][1] * hl[2][0])
            )
            if m1 > 0.0 and m2 > 0.0 and det > 0.0:
                break
            lam *= 2.0
        step = [0.0, 0.0, 0.0]
        for k in range(3):
            a_mat = [[hl[i][j] for j in range(3)] for i in range(3)]
            for i in range(3):
                a_mat[i][k] = -g[i]
            num = (
                a_mat[0][0] * (a_mat[1][1] * a_mat[2][2] - a_mat[1][2] * a_mat[2][1])
                - a_mat[0][1] * (a_mat[1][0] * a_mat[2][2] - a_mat[1][2] * a_mat[2][0])
                + a_mat[0][2] * (a_mat[1][0] * a_mat[2][1] - a_mat[1][1] * a_mat[2][0])
            )
            step[k] = num / det
        s_cur = ndt_score(source, dists, cell_size, ax, ay, ang)
        d0, d1, d2 = step
        while True:
            if ndt_score(source, dists, cell_size, ax + d0, ay + d1, ang + d2) > s_cur:
                break  # strict improvement — accept
            if max(abs(d0), abs(d1), abs(d2)) <= eps:
                return Twist(ax, ay, ang)  # fixed point: no halving level improves
            d0 /= 2.0
            d1 /= 2.0
            d2 /= 2.0
        ax += d0
        ay += d1
        ang += d2
        if max(abs(d0), abs(d1), abs(d2)) <= eps:
            break
    return Twist(ax, ay, ang)


class Ndt(Estimator):
    """Normal Distributions Transform scan matching on the staircase scenario; the pose
    is the declared gauge composed with every recovered twist (odometry never read)."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._theta_deg = params.get_float("theta_deg")
        self._cell_size = params.get_float("cell_size")
        self._lam0 = params.get_float("lam0")
        self._eps = params.get_float("eps")
        self._max_iters = params.get_int("max_iters")
        self._pose: Pose | None = None  # set at t = 0 to the declared gauge pose
        self._prev: tuple[Point, ...] | None = None  # the previous step's scan
        self._poses: list[Pose] = []  # readout history (index == step t)

    @property
    def name(self) -> str:
        return "ndt"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        """t = 0 adopts the declared gauge pose (registration measures RELATIVE
        motion — same convention as Icp). Every later step scores the arriving scan
        against the previous scan's field and composes the recovered twist; Step.odom
        is never read. A step without a scan carries the estimate forward untouched."""
        if self._pose is None:
            self._pose = Pose(self._x0, self._y0, math.radians(self._theta_deg))
            self._prev = step.scan if step.scan is not None else ()
        else:
            twist = ndt_step(
                step.scan if step.scan is not None else (),
                self._prev if self._prev is not None else (),
                self._cell_size, self._lam0, self._eps, self._max_iters,
            )
            self._pose = pose_compose(self._pose, twist)
            if step.scan is not None:
                self._prev = step.scan
        self._poses.append(self._pose)
        if recorder is not None:
            # No covariance — NDT carries no uncertainty model (that is what the
            # filter_based branch adds on top of this).
            recorder.pose_estimated(step.t, self._pose)

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        return EstimateResult(poses=tuple(self._poses))
