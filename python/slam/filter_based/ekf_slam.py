"""ekf_slam — Extended Kalman Filter SLAM (Smith & Cheeseman, IJRR 1986), the filter_based
branch's first member: the map lives INSIDE the estimator. Every earlier branch CONSUMED
(id, bearing, range) observations with association given; this is the page that shows what
an observation DOES once association is given — it augments the state. The state vector is
x = [px, py, θ | l_0x, l_0y, l_1x, l_1y, ...]: the pose plus one (x, y) pair per landmark
discovered so far, and ONE dense covariance Σ over all of it grows as landmarks are born.

THE GAUGE, HONESTLY. The declared start pose IS known exactly (the scenario's first GT pose
IS the gauge — this repo's convention since histogram_filter), so Σ starts as the 3×3 ZERO
matrix: the pose uncertainty is honestly zero at t = 0 and grows ONLY from Q, the odometry
noise the filter declares. A landmark's FIRST observation therefore lands with covariance
B⁻¹RB⁻ᵀ alone (pure measurement noise rotated into world frame — no pose contribution yet)
and cross-covariance exactly ±0; every later re-observation of a KNOWN id then pulls the
pose back through those correlation blocks. That is the whole lesson of the branch:
landmarks do not just get estimated, they ESTIMATE the pose back (what graph_based will
later restate as factor graphs).

THE FIXED OPERATION ORDER (C++ and TS mirror it bit-for-bit; every float op below rounds
where the Python line rounds):

  t = 0            x ← gauge (x0, y0, radians(theta_deg)); Σ ← 3×3 zeros.
  t ≥ 1 predict    x̂ ← x ⊕ u (the SAME pose_compose the simulator used to differentiate
                   ground truth — with σ_xy = σ_θ = 0 the estimate would ride GT bit-for-bit);
                   Σ̄ = FΣFᵀ + Q embedded, F = blockdiag(J, I) with
                   J = [[1, 0, −(s·dx + c·dy)], [0, 1, c·dx − s·dy], [0, 0, 1]] evaluated at
                   the PRIOR mean (c = cos θ̂, s = sin θ̂ BEFORE propagation), Q added to the
                   pose diagonal in slot order. Landmark rows/cols of F are identity — a
                   static landmark needs no motion model; its uncertainty rides along on the
                   pose's shoulders through the cross blocks.

  per observation  (ascending id, exactly as the simulator emitted them):

    KNOWN id → standard EKF update. With dx = lx − px, dy = ly − py evaluated at the
    CURRENT estimate: r̂ = sqrt(dx² + dy²), β = atan2(dy, dx); the predicted measurement is
    (r̂, wrap(β − θ̂)) — the simulator's own bearing formula read back. The innovation wraps
    like the sim wraps bearings: y_r = z_r − r̂ (no wrap: range never wraps), y_b =
    wrap(z_b − wrap(β − θ̂)). H is the n×2 Jacobian h[i][a] = ∂h_a/∂x_i at the current mean,
    S = HΣHᵀ + R (2×2), K = ΣHᵀS⁻¹ with an explicit 2×2 inverse (det, adjugate — no solver
    anywhere), x += Ky componentwise then θ re-wrapped, and Joseph form for the covariance:
    Σ ← AΣAᵀ + KRKᵀ with A = I − KH. Joseph, not the shortcut (I−KH)Σ: the shortcut is only
    ALGEBRAICALLY equal to the Joseph form when Σ̄ was exact — and pinning bit-for-bit
    arithmetic is exactly a case where "algebraically equal" is not equality.

    NEW id → augmentation, NOT an update. The mean becomes robot_to_world((z_r cos z_b,
    z_r sin z_b), pose) — the exact inverse of the measurement model, so this observation's
    innovation would be exactly 0 and no mean correction could exist anyway (that is what
    birth MEANS: an unknown landmark carries no information about the pose). The Jacobian H
    here has ZERO rows for every already-known landmark slot; running it through the SAME
    pipeline as a re-observation (G = ΣH, W = HΣHᵀ + R, new block B⁻¹WB⁻ᵀ, cross columns
    −GB⁻ᵀ) is then exactly the improper-prior limit derived in the page: an unknown
    landmark carries no information about the pose, so the pose block of Σ does not move.

Σ stays EXACTLY symmetric by contract: every product computes the upper triangle in fixed
order and COPIES it below the diagonal (a copy is exact; two independent ulp-different ways
of computing the same covariance entry would not be). Landmarks are appended in OBSERVATION
order (ascending id within a step), so state slots are NOT id-ordered when ids arrive out
of order — the id↔slot bookkeeping is part of the page, not hidden by it. The trace emits
pose_estimated with cov = [σx, σy, σθ] (the Σ diagonal square roots) and landmarks_updated
in slot order with sx/sy.
"""

from __future__ import annotations

import math

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose, robot_to_world, wrap
from ..core.params import ParamSet
from ..core.trace import TraceRecorder
from ..core.types import EstimateResult, LandmarkEstimate, Pose, Step, Twist


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    """Dense matmul with the contract's fixed order: out[i][j] accumulates a[i][k]·b[k][j]
    over k ASCENDING into an accumulator that starts at +0.0. No term is ever skipped —
    the sums themselves are the contract, in all three languages (a multiplied ±0.0 term
    changes no summand's value; sign-of-zero parses equal anyway)."""
    n = len(a)
    m = len(b[0])
    out: list[list[float]] = []
    for i in range(n):
        row_a = a[i]
        row_out: list[float] = []
        for j in range(m):
            acc = 0.0
            for k in range(len(row_a)):
                acc += row_a[k] * b[k][j]
            row_out.append(acc)
        out.append(row_out)
    return out


def _matmul_at_b(a: list[list[float]], h: list[list[float]]) -> list[list[float]]:
    """out[i][j] = Σ_k a[i][k]·h[j][k], k ASCENDING — product with the TRANSPOSED right
    factor (ΣH in both branches, A Fᵀ in predict, −G B⁻ᵀ at birth)."""
    n = len(a)
    m = len(h)
    out: list[list[float]] = []
    for i in range(n):
        row_a = a[i]
        row_out: list[float] = []
        for j in range(m):
            acc = 0.0
            row_h = h[j]
            for k in range(len(row_a)):
                acc += row_a[k] * row_h[k]
            row_out.append(acc)
        out.append(row_out)
    return out


def _invert2(m: list[list[float]]) -> list[list[float]]:
    """Explicit 2×2 inverse — det and adjugate, no solver anywhere:
    det = m00·m11 − m01·m10; inv = [[m11/det, −m01/det], [−m10/det, m00/det]]."""
    det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    return [
        [m[1][1] / det, -m[0][1] / det],
        [-m[1][0] / det, m[0][0] / det],
    ]


class EkfSlam(Estimator):
    """Augmented-state EKF-SLAM: pose + landmark positions in ONE Gaussian; association
    given; birth is covariance-only (improper-prior limit), re-observation corrects."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._theta_deg = params.get_float("theta_deg")
        sigma_xy = params.get_float("sigma_xy")
        sigma_theta = params.get_float("sigma_theta")
        # Q — the motion noise the filter DECLARES (the same values the scenario injects
        # into its simulator; declaring them is how the algorithm "knows" the model).
        self._q_pos = sigma_xy * sigma_xy
        self._q_theta = sigma_theta * sigma_theta
        # R — measurement noise, diagonal in MEASUREMENT space (range m, bearing rad):
        # different units, so a diagonal that is anything but isotropic.
        sigma_range = params.get_float("sigma_range")
        sigma_bearing = params.get_float("sigma_bearing")
        self._r_range = sigma_range * sigma_range
        self._r_bearing = sigma_bearing * sigma_bearing
        # State: x[0..2] is the pose, then one (x, y) pair per DISCOVERED landmark in
        # observation order; _ids maps slot k -> scenario landmark id (slot ≠ id when
        # ids arrive out of order — part of the lesson, not a detail to hide).
        self._x: list[float] = []
        self._p: list[list[float]] = []  # dense covariance, same slot layout
        self._ids: list[int] = []
        self._poses: list[Pose] = []

    @property
    def name(self) -> str:
        return "ekf_slam"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.LANDMARKS}

    # --- predict ------------------------------------------------------------

    def _predict(self, dx_o: float, dy_o: float, dtheta_o: float) -> None:
        """x̂ ← x ⊕ u (core pose_compose — the exact inverse of what the simulator used to
        differentiate ground truth), then Σ̄ = FΣFᵀ + Q with F = blockdiag(J, I). J is
        evaluated at the PRIOR mean; every matmul runs in the contract's ascending order."""
        px, py, th = self._x[0], self._x[1], self._x[2]
        c = math.cos(th)
        s = math.sin(th)
        # J = ∂(x ⊕ u)/∂x at the prior mean — fixed expressions.
        j02 = -((s * dx_o) + (c * dy_o))
        j12 = (c * dx_o) - (s * dy_o)
        # The pose triple propagates through the SAME pose_compose the simulator used on
        # ground truth; landmark slots are STATIC — their mean components carry over by
        # exact copy (the identity block of F applied to the mean, nothing more).
        p_new = pose_compose(Pose(px, py, th), Twist(dx_o, dy_o, dtheta_o))
        new_x = [p_new.x, p_new.y, p_new.theta]
        for i in range(3, len(self._x)):
            new_x.append(self._x[i])
        self._x = new_x
        n = len(self._x)
        f: list[list[float]] = [[0.0] * n for _ in range(n)]
        f[0][0] = 1.0
        f[0][2] = j02
        f[1][1] = 1.0
        f[1][2] = j12
        f[2][2] = 1.0
        for i in range(3, n):
            f[i][i] = 1.0  # a static landmark needs no motion model — identity rows/cols
        a = _matmul(f, self._p)  # A = FΣ (ascending k; multiplied zero terms change nothing)
        p_next: list[list[float]] = [[0.0] * n for _ in range(n)]
        for i in range(n):  # Σ̄ = A·Fᵀ — upper triangle ascending, lower copied (exact copy)
            row_a = a[i]
            for j in range(i, n):
                acc = 0.0
                row_f = f[j]
                for k in range(n):
                    acc += row_a[k] * row_f[k]
                p_next[i][j] = acc
                p_next[j][i] = acc
        p_next[0][0] += self._q_pos  # Q lands on the pose diagonal in slot order
        p_next[1][1] += self._q_pos
        p_next[2][2] += self._q_theta
        self._p = p_next

    # --- update ---------------------------------------------------------------

    def _observe(self, obs_id: int, z_range: float, z_bearing: float) -> None:
        """One observation, ascending id. KNOWN slot → standard EKF update (Joseph form);
        unknown id → augmentation (improper-prior limit — see the module docstring)."""
        px, py, th = self._x[0], self._x[1], self._x[2]
        slot = -1
        for k, known_id in enumerate(self._ids):  # association is GIVEN: linear id lookup
            if known_id == obs_id:
                slot = k
                break

        if slot < 0:
            # ---- BIRTH. Mean: the observation back-projected through the CURRENT pose —
            # the exact inverse of the measurement model, so this observation's innovation
            # is exactly 0 and no mean correction could exist anyway (that is what birth
            # MEANS: an unknown landmark carries no information about the pose).
            zx = z_range * math.cos(z_bearing)
            zy = z_range * math.sin(z_bearing)
            lx, ly = robot_to_world((zx, zy), Pose(px, py, th))
            dx = lx - px
            dy = ly - py
            r_hat = math.sqrt(dx * dx + dy * dy)
            beta = math.atan2(dy, dx)
            cb = math.cos(beta)
            sb = math.sin(beta)
            n = len(self._x)
            # H at the current mean — rows 3.. are ZERO (this landmark is not in the state
            # yet), which is exactly why birth cannot move the pose.
            h: list[list[float]] = [[0.0, 0.0] for _ in range(n)]
            h[0][0] = -cb
            h[1][0] = -sb
            h[2][0] = 0.0
            h[0][1] = sb / r_hat
            h[1][1] = -cb / r_hat
            h[2][1] = -1.0
            g = _matmul(self._p, h)  # G = ΣH — the SAME fixed-order product as an update
            w = [[0.0, 0.0], [0.0, 0.0]]
            w01 = 0.0
            for i in range(n):
                w[0][0] += h[i][0] * g[i][0]
                w01 += h[i][0] * g[i][1]
                w[1][1] += h[i][1] * g[i][1]
            w[0][1] = w01
            w[1][0] = w01  # symmetric by copy (exact)
            w[0][0] += self._r_range
            w[1][1] += self._r_bearing
            # B = ∂h/∂landmark (det = 1/r̂ ≠ 0): row 0 is the range gradient, row 1 the
            # bearing gradient — the SAME entries as the landmark columns of H above.
            b_inv = _invert2([[cb, sb], [-sb / r_hat, cb / r_hat]])
            # New diagonal block B⁻¹WB⁻ᵀ (generic products; upper computed, lower copied);
            # cross columns −GB⁻ᵀ — exactly ±0 at t = 0, where Σ_pp is still zero.
            t_mat = _matmul(b_inv, w)
            nb = _matmul_at_b(t_mat, b_inv)
            nb[1][0] = nb[0][1]  # B⁻¹WB⁻ᵀ is symmetric — lower entry copied (exact)
            cross = _matmul_at_b(g, b_inv)
            # Grow the state: append the mean pair, then grow Σ by one column pair and its
            # mirrored row (the copy keeps the matrix EXACTLY symmetric).
            self._x.append(lx)
            self._x.append(ly)
            grown: list[list[float]] = []
            for i in range(n):
                row = list(self._p[i])
                row.append(-cross[i][0])
                row.append(-cross[i][1])
                grown.append(row)
            col_x: list[float] = []
            col_y: list[float] = []
            for i in range(n):
                col_x.append(grown[i][n])  # the just-appended cross column (slot n)
                col_y.append(grown[i][n + 1])
            grown.append(col_x + [nb[0][0], nb[0][1]])
            grown.append(col_y + [nb[1][0], nb[1][1]])
            self._p = grown
            self._ids.append(obs_id)
            return

        # ---- RE-OBSERVATION: standard EKF update on the existing slot.
        lx = self._x[3 + 2 * slot]
        ly = self._x[4 + 2 * slot]
        dx = lx - px
        dy = ly - py
        r_hat = math.sqrt(dx * dx + dy * dy)
        beta = math.atan2(dy, dx)
        cb = math.cos(beta)
        sb = math.sin(beta)
        y_r = z_range - r_hat                      # range never wraps
        y_b = wrap(z_bearing - wrap(beta - th))    # bearing wraps like the sim's does
        n = len(self._x)
        c1 = 3 + 2 * slot
        h = [[0.0, 0.0] for _ in range(n)]
        h[0][0] = -cb
        h[1][0] = -sb
        h[2][0] = 0.0
        h[0][1] = sb / r_hat
        h[1][1] = -cb / r_hat
        h[2][1] = -1.0
        h[c1][0] = cb
        h[c1][1] = -sb / r_hat
        h[c1 + 1][0] = sb
        h[c1 + 1][1] = cb / r_hat
        g = _matmul(self._p, h)                    # G = ΣH (n×2), ascending order
        s_mat = [[0.0, 0.0], [0.0, 0.0]]
        s01 = 0.0
        for i in range(n):
            s_mat[0][0] += h[i][0] * g[i][0]
            s01 += h[i][0] * g[i][1]
            s_mat[1][1] += h[i][1] * g[i][1]
        s_mat[0][1] = s01
        s_mat[1][0] = s01  # symmetric by copy (exact)
        s_mat[0][0] += self._r_range
        s_mat[1][1] += self._r_bearing
        s_inv = _invert2(s_mat)
        k_gain: list[list[float]] = []
        for i in range(n):
            gi = g[i]
            k_gain.append(
                [
                    gi[0] * s_inv[0][0] + gi[1] * s_inv[1][0],
                    gi[0] * s_inv[0][1] + gi[1] * s_inv[1][1],
                ]
            )
        # Mean update — componentwise in index order; θ re-wraps after its own add.
        self._x[0] += k_gain[0][0] * y_r + k_gain[0][1] * y_b
        self._x[1] += k_gain[1][0] * y_r + k_gain[1][1] * y_b
        self._x[2] = wrap(self._x[2] + (k_gain[2][0] * y_r + k_gain[2][1] * y_b))
        for slot_i in range(3, n):
            self._x[slot_i] += k_gain[slot_i][0] * y_r + k_gain[slot_i][1] * y_b
        # Joseph form: Σ ← AΣAᵀ + KRKᵀ with A = I − KH (fixed ascending matmuls; upper
        # triangle computed, lower copied).
        a_mat: list[list[float]] = []
        for i in range(n):
            row = []
            for j in range(n):
                acc = 0.0
                for k in range(2):
                    acc += k_gain[i][k] * h[j][k]
                row.append((1.0 if i == j else 0.0) - acc)
            a_mat.append(row)
        m_mat = _matmul(a_mat, self._p)  # M = AΣ (ascending k)
        sigma_new: list[list[float]] = [[0.0] * n for _ in range(n)]
        for i in range(n):
            row_m = m_mat[i]
            t_i0 = k_gain[i][0] * self._r_range  # (KR)[i][0], R diagonal — exact products
            t_i1 = k_gain[i][1] * self._r_bearing
            for j in range(i, n):
                acc = 0.0
                row_j = a_mat[j]
                for k in range(n):
                    acc += row_m[k] * row_j[k]
                acc += t_i0 * k_gain[j][0] + t_i1 * k_gain[j][1]
                sigma_new[i][j] = acc
                sigma_new[j][i] = acc
        self._p = sigma_new

    # --- lifecycle ------------------------------------------------------------

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        if not self._poses:
            # t = 0 adopts the declared gauge pose with an EXACTLY zero covariance —
            # the gauge convention of this whole repo: the start pose IS known.
            self._x = [self._x0, self._y0, math.radians(self._theta_deg)]
            self._p = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        else:
            assert step.odom is not None  # contract: t ≥ 1 always carries the command
            self._predict(step.odom.dx, step.odom.dy, step.odom.dtheta)
        if step.obs is not None:
            for obs in step.obs:  # ascending id — fixed observation order
                self._observe(obs.id, obs.range, obs.bearing)
        pose = Pose(self._x[0], self._x[1], self._x[2])
        self._poses.append(pose)
        if recorder is not None:
            cov = [
                math.sqrt(self._p[0][0]),
                math.sqrt(self._p[1][1]),
                math.sqrt(self._p[2][2]),
            ]
            recorder.pose_estimated(step.t, pose, cov)
            estimates: list[LandmarkEstimate] = []
            for k, lm_id in enumerate(self._ids):
                estimates.append(
                    LandmarkEstimate(
                        id=lm_id,
                        x=self._x[3 + 2 * k],
                        y=self._x[4 + 2 * k],
                        sx=math.sqrt(self._p[3 + 2 * k][3 + 2 * k]),
                        sy=math.sqrt(self._p[4 + 2 * k][4 + 2 * k]),
                    )
                )
            recorder.landmarks_updated(step.t, estimates)

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        landmarks = tuple(
            LandmarkEstimate(
                id=lm_id,
                x=self._x[3 + 2 * k],
                y=self._x[4 + 2 * k],
                sx=math.sqrt(self._p[3 + 2 * k][3 + 2 * k]),
                sy=math.sqrt(self._p[4 + 2 * k][4 + 2 * k]),
            )
            for k, lm_id in enumerate(self._ids)
        )
        return EstimateResult(poses=tuple(self._poses), landmarks=landmarks)
