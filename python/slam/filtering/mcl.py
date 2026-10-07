"""mcl — Monte Carlo Localization with KLD-sampling (MCL is Fox, Burgard, Dellaert &
Thrun AAAI 1999; the adaptive sample count is Fox, "Adapting the Sample Size in
Particle Filters through KLD-sampling", IJRR 2003; the textbook form is Thrun,
Burgard & Fox 2005 ch. 4).

The filtering branch's fourth step. The particle_filter page ended on a question:
a fixed N is a lottery — too few samples and the true mode dies, too many wastes
every raycast. MCL keeps the SAME filter (local prior over the declared cell × full
heading, bootstrap motion model, beam-likelihood weighting, weighted-mean readout)
and adds ONE mechanism: at every resample-move step the sample count is not fixed —
samples are drawn one at a time until their bin coverage statistically certifies
the cloud is dense enough.

WHY A SAMPLE COUNT CAN BE CERTIFIED (Fox 2003, derived on the algorithm's page):
discretize belief into bins (here: bin_xy × bin_xy × bin_theta boxes of pose space)
and let p be the true belief discretized over them. Samples drawn i.i.d. from p land
in bins multinomially; the likelihood-ratio statistic for "these n samples came from
p" is λ_n = Π (p̂_i/p_i)^{x_i}, its log is n·KL(p̂‖p) — the KL between the sample MLE
and the true belief, discretized — and Wilks' result gives 2·n·KL → χ²_{k−1} in
distribution (k = bins with support). So P(KL > ε) ≤ δ once n ≥ (1/2ε)·χ²_{k−1,1−δ};
Wilson–Hilferty turns that quantile into the closed form below, whose z_q is
core.stats.inv_norm_cdf(1 − δ). The filter does not know p — it treats the bins its
OWN samples occupy as the support (Fox's own caveat: with the predictive belief
standing in for the unknown posterior, divergence is possible when the bound is too
loose; that honesty is part of the lesson).

    n_chi(k) = 0  (k ≤ 1 — degenerate: one bin needs no guarantee beyond n_min)
             = (ν / 2ε) · t³,  ν = k − 1, q₉ = 2/(9ν), t = (1 − q₉) + z_q·√q₉

THE LOOP (fixed operation order — C++ and TS mirror it bit-for-bit / ulp-for-ulp):
t = 0 draws max_particles particles uniformly over the declared cell × heading,
weights them by the first scan exactly like particle_filter._weight. From t ≥ 1,
every sample is drawn one at a time: ancestor walk over the previous weights (one
uniform draw, strict `>`, guard i < n−1), motion noise drawn and composed
(x ⊕ (u + ε)), the arriving scan scored inline in scan order, the new pose binned
(floor(x/bin_xy), floor(y/bin_xy), floor(θ/bin_θ)) — a bin never seen this step bumps
k, which recomputes n_chi; the loop stops when n ≥ n_chi and n ≥ n_min, or at the
max_particles cap. Weights are then exp(ll − max ll) normalized ascending (the old
weights already acted — they drove the ancestor walk). Readout is unchanged from the
previous page: weighted mean x/y, circular-mean heading, population std devs.

Everything else is the same fixed-order contract as particle_filter: draws in
ascending order; likelihood per scan point in scan order; max-shifted exps
accumulated ascending then divided ascending; sentinel range_max + res on a miss.
"""

from __future__ import annotations

import math

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose, wrap
from ..core.params import ParamSet
from ..core.rng import Rng
from ..core.sim import raycast
from ..core.stats import inv_norm_cdf
from ..core.trace import TraceRecorder
from ..core.types import EstimateResult, Point, Pose, Step, Twist


def kld_bound(k: int, epsilon: float, z_q: float) -> float:
    """Fox 2003 eq. (14): the Wilson–Hilferty closed form of n = χ²_{k−1,1−δ}/2ε —
    the sample count that guarantees KL(p̂_n ‖ p) ≤ ε with probability 1 − δ when the
    support spans k bins. Fixed operation order (t·t·t, left-associative); k ≤ 1 is
    the degenerate case: a one-bin belief needs no guarantee beyond n_min, so the
    bound returns 0.0 and the loop stops at its floor."""
    if k <= 1:
        return 0.0
    nu = float(k - 1)
    q9 = 2.0 / (9.0 * nu)
    t = (1.0 - q9) + z_q * math.sqrt(q9)
    return (nu / (2.0 * epsilon)) * t * t * t


class Mcl(Estimator):
    """Monte Carlo Localization with KLD-adaptive sample count on a known map."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._epsilon = params.get_float("epsilon")
        delta = params.get_float("delta")
        self._bin_xy = params.get_float("bin_xy")
        self._bin_theta = params.get_float("bin_theta")
        self._n_min = params.get_int("n_min")
        self._max_particles = params.get_int("max_particles")
        self._sigma_range = params.get_float("sigma_range")
        self._sigma_xy = params.get_float("sigma_xy")
        self._sigma_theta = params.get_float("sigma_theta")
        self._range_max = params.get_float("range_max")
        seed = params.get_int("seed")  # the demo injected the scenario's seed here
        self._rng = Rng(seed)
        # The bound's quantile is a constant of the config — computed ONCE, here.
        self._z_q = inv_norm_cdf(1.0 - delta)
        self._x: list[float] = []
        self._y: list[float] = []
        self._theta: list[float] = []
        self._w: list[float] = []
        self._poses: list[Pose] = []  # readout history (index == step t)
        self._initialized = False

    @property
    def name(self) -> str:
        return "mcl"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    # --- init ---------------------------------------------------------------

    def _init_particles(self) -> None:
        """Draw max_particles particles in ascending order (the PRIOR is fixed-size —
        adaptivity starts at the first KLD step): the start cell (the one holding
        x0/y0) uniformly in x and y, heading uniform on [−π, π). Three draws per
        particle — x offset, y offset, heading — from the algorithm's own stream;
        weights exactly 1/max_particles."""
        episode = self.episode
        assert episode is not None
        grid = episode.grid
        h = grid.height
        ox, oy = grid.origin
        res = grid.resolution
        row0, col0 = grid.world_to_cell(self._x0, self._y0)
        rng = self._rng
        inv_n = 1.0 / float(self._max_particles)
        for _ in range(self._max_particles):
            x = ox + (float(col0) + rng.uniform01()) * res
            y = oy + (float(h - 1 - row0) + rng.uniform01()) * res
            theta = rng.uniform01() * (2.0 * math.pi) - math.pi
            self._x.append(x)
            self._y.append(y)
            self._theta.append(theta)
        self._w = [inv_n] * self._max_particles  # uniform prior — exactly 1/N each
        self._initialized = True

    def _points(self, scan: tuple[Point, ...]) -> list[tuple[float, float]]:
        """(r, β) per scan point, in scan order — the polar form of each robot-frame
        endpoint. Fixed formulas: sqrt(zx² + zy²), atan2(zy, zx)."""
        points: list[tuple[float, float]] = []
        for z in scan:
            r = math.sqrt(z[0] * z[0] + z[1] * z[1])
            beta = math.atan2(z[1], z[0])
            points.append((r, beta))
        return points

    # --- update -------------------------------------------------------------

    def _weight(self, points: list[tuple[float, float]]) -> None:
        """t = 0 only: multiply every uniform weight by exp(ll − max ll); renormalize.

        Same fixed pattern as particle_filter._weight: ll is the sum over this step's
        scan points IN SCAN ORDER of the beam-model Gaussian −½·((r − expected)/σ)²;
        `expected` is the same DDA raycast the simulator fired, run at each particle's
        own pose along heading + β (a miss keeps the sentinel range_max + res). exp
        never overflows after the max shift and the argmax keeps its weight exactly;
        a total of 0.0 leaves the belief unchanged (documented degenerate case)."""
        episode = self.episode
        assert episode is not None
        grid = episode.grid
        sentinel = self._range_max + grid.resolution
        n = len(self._w)
        m = float("-inf")
        ll: list[float] = []
        for i in range(n):
            acc = 0.0
            for (r, beta) in points:
                e = raycast(grid, self._x[i], self._y[i], self._theta[i] + beta,
                            self._range_max)
                if e is None:
                    e = sentinel
                d = (r - e) / self._sigma_range
                acc += (-0.5 * d) * d
            ll.append(acc)
            if acc > m:  # max is exact — order-independent by construction
                m = acc
        total = 0.0
        weighted: list[float] = []
        for i in range(n):
            wv = self._w[i] * math.exp(ll[i] - m)
            weighted.append(wv)
            total += wv
        if total != 0.0:
            for i in range(n):
                self._w[i] = weighted[i] / total

    def _kld_update(self, u: Twist, scan: tuple[Point, ...] | None) -> None:
        """One KLD-sampling step: resample-move-reweight ONE sample at a time until
        the bound certifies coverage (or the cap). Per sample, fixed order: ancestor
        walk over the previous weights (ONE uniform draw, strict `>`, guard i < n−1 —
        this is plain multinomial resampling; KLD changes WHEN it stops, not HOW it
        picks), then three gaussian draws and x ⊕ (u + ε); then the arriving scan's
        likelihood at the new pose (scan order, sentinel on a miss) and the bin key —
        a first-seen bin bumps k and recomputes n_chi. The loop stops when
        n ≥ n_chi(k) AND n ≥ n_min, or at max_particles."""
        episode = self.episode
        assert episode is not None
        grid = episode.grid
        sentinel = self._range_max + grid.resolution
        points = self._points(scan) if scan else []
        rng = self._rng
        x_old, y_old, th_old, w_old = self._x, self._y, self._theta, self._w
        n_old = len(w_old)
        x_new: list[float] = []
        y_new: list[float] = []
        th_new: list[float] = []
        ll_new: list[float] = []
        seen: set[tuple[int, int, int]] = set()
        k = 0
        n = 0
        m = float("-inf")
        n_chi = float("inf")
        while True:
            u1 = rng.uniform01()
            i = 0
            c = w_old[0]
            while i < n_old - 1 and u1 > c:
                i += 1
                c += w_old[i]
            ex = rng.gaussian(0.0, self._sigma_xy)
            ey = rng.gaussian(0.0, self._sigma_xy)
            et = rng.gaussian(0.0, self._sigma_theta)
            p = pose_compose(Pose(x_old[i], y_old[i], th_old[i]),
                             Twist(u.dx + ex, u.dy + ey, u.dtheta + et))
            acc = 0.0
            for (r, beta) in points:
                e = raycast(grid, p.x, p.y, p.theta + beta, self._range_max)
                if e is None:
                    e = sentinel
                d = (r - e) / self._sigma_range
                acc += (-0.5 * d) * d
            x_new.append(p.x)
            y_new.append(p.y)
            th_new.append(p.theta)
            ll_new.append(acc)
            if acc > m:  # max is exact — order-independent by construction
                m = acc
            key = (math.floor(p.x / self._bin_xy), math.floor(p.y / self._bin_xy),
                   math.floor(p.theta / self._bin_theta))
            if key not in seen:
                seen.add(key)
                k += 1
            n += 1
            if n >= self._n_min:
                n_chi = kld_bound(k, self._epsilon, self._z_q)
            if (n >= n_chi and n >= self._n_min) or n >= self._max_particles:
                break
        total = 0.0
        weighted: list[float] = []
        for i in range(n):
            wv = math.exp(ll_new[i] - m)
            weighted.append(wv)
            total += wv
        self._w = [weighted[i] / total for i in range(n)]
        self._x = x_new
        self._y = y_new
        self._theta = th_new

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        """t = 0 initializes the fixed-size prior cloud and weights it by the first
        scan; every later step with an arriving command runs one KLD resample-move-
        reweight. Then read out — pose first, cloud second (identical to the
        particle_filter page's readout)."""
        first = not self._initialized
        if first:
            self._init_particles()
            if step.scan:
                self._weight(self._points(step.scan))
        elif step.odom is not None:
            self._kld_update(step.odom, step.scan)

        # Readout (fixed order): weighted mean x, y ascending; then the circular
        # heading from the sin/cos sums; then population variances in a second
        # ascending pass (θ's deviation wrapped around the seam).
        x_hat = 0.0
        y_hat = 0.0
        s_sin = 0.0
        s_cos = 0.0
        n = len(self._w)
        for i in range(n):
            wv = self._w[i]
            x_hat += wv * self._x[i]
            y_hat += wv * self._y[i]
            s_sin += wv * math.sin(self._theta[i])
            s_cos += wv * math.cos(self._theta[i])
        theta_hat = math.atan2(s_sin, s_cos)

        var_x = 0.0
        var_y = 0.0
        var_t = 0.0
        for i in range(n):
            wv = self._w[i]
            dx = self._x[i] - x_hat
            dy = self._y[i] - y_hat
            dt = wrap(self._theta[i] - theta_hat)
            var_x += wv * (dx * dx)
            var_y += wv * (dy * dy)
            var_t += wv * (dt * dt)
        cov = [math.sqrt(var_x), math.sqrt(var_y), math.sqrt(var_t)]

        pose = Pose(x_hat, y_hat, theta_hat)
        self._poses.append(pose)
        if recorder is not None:
            recorder.pose_estimated(step.t, pose, cov)
            recorder.particles_updated(
                step.t,
                [
                    (self._x[i], self._y[i], self._theta[i], self._w[i])
                    for i in range(n)
                ],
            )

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        return EstimateResult(poses=tuple(self._poses))
