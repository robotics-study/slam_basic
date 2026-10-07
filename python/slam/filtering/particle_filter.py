"""particle_filter — the bootstrap filter (Gordon, Salmond & Smith 1993; the robot
localization formulation is Bourlak & Schulman 1996 and Fox, Burgard, Dellaert &
Thrun AAAI 1999, textbook form Thrun, Burgard & Fox 2005 ch. 4).

The filtering branch's third step: the histogram filter kept a probability for
every lattice state by brute force; this one keeps N SAMPLES instead — a weighted
particle cloud over the CONTINUOUS pose (x, y, θ) — and pays for that freedom
with sampling. Nothing else about the Bayes filter changed: predict still pushes
every hypothesis through the motion model, update still multiplies in the sensor
likelihood, and a resampling step is what replaces the exact marginalization the
lattice could afford.

- STATE / PRIOR: N particles drawn uniformly over the DECLARED start cell (the
  one containing x0/y0) with heading uniform on [−π, π). Position is known to a
  cell, heading not at all — LOCAL localization; deciding how many samples a
  GLOBAL prior needs is the next page's question (MCL / KLD-sampling).
- PREDICT: every particle rides the arriving command with its own drawn noise,
  x' = x ⊕ (u + ε), ε ~ N(0, diag(σ_xy², σ_θ²)), draws in ascending particle
  order. The scenario commands exact twists (its odom_noise is 0): the drift this
  branch teaches lives entirely inside the filter's model — which is honest,
  because a filter that cannot see a noise source must model it anyway.
- UPDATE: the same beam model as the simulator's forward model, evaluated at
  each particle's own pose. Each scan point z carries r = |z| and β = atan2; the
  expectation is raycast(particle) along heading + β (no wrap needed — cos/sin are
  periodic), a miss keeps the sentinel range_max + res exactly like the sim, and
  the point contributes −½·((r − expected)/σ_range)². Weights multiply by
  exp(ll − max ll) and renormalize; a zero total leaves the belief unchanged
  (documented degenerate case, identical in C++).
- RESAMPLE: systematic (Thrun's pseudocode 2.8) when ESS = 1/Σw² < N/2 — one
  uniform draw u ∈ [0, 1/N), a cumulative-weight walk with a strict `>` tie rule,
  and weights reset to exactly 1/N afterwards. Resampling is what turns weighted
  importance sampling into equal-weight Monte Carlo; without it the cloud would
  collapse onto one ancestor within a few steps (sample impoverishment — visible
  on this page when N is small).
- READOUT: position = weighted mean, heading = atan2(Σ w·sin θ, Σ w·cos θ) (the
  arithmetic mean of angles lies across the ±π seam; this one cannot), spread =
  population std devs — x/y plain, θ through wrap so the seam never explodes.

WHY σ_range IS BIG HERE (0.3, not 0.05): a sharp likelihood has a support only a
few centimetres wide, and uniform samples over a cell × full heading land inside
it with probability well under 1/N — the filter would lock onto whichever single
particle got lucky, seed by seed. That lottery IS the sampling lesson: coarse
sampling needs a blunt likelihood (or more particles / smarter proposals — the
MCL page). The price is honest too: position accuracy floors near σ/√k, and what
the fan cannot see at all (lateral offset in the slit) stays spread in the cloud.

Everything below is a fixed operation order so C++ mirrors it bit-for-bit: draws
in ascending particle order; likelihood per scan point in scan order per particle;
max-shifted exps accumulated ascending into a fresh list, then divided ascending;
resample walk with strict `>` and the loop's final (unused) u += 1/N kept.
"""

from __future__ import annotations

import math

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose, wrap
from ..core.params import ParamSet
from ..core.rng import Rng
from ..core.sim import raycast
from ..core.types import EstimateResult, Pose, Twist


class ParticleFilter(Estimator):
    """Bootstrap particle filter over continuous (x, y, θ) on a known map."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._n = params.get_int("n_particles")
        self._sigma_range = params.get_float("sigma_range")
        self._sigma_xy = params.get_float("sigma_xy")
        self._sigma_theta = params.get_float("sigma_theta")
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._range_max = params.get_float("range_max")
        seed = params.get_int("seed")  # the demo injected the scenario's seed here
        self._rng = Rng(seed)
        self._inv_n = 1.0 / float(self._n)
        self._x: list[float] = []
        self._y: list[float] = []
        self._theta: list[float] = []
        self._w: list[float] = []
        self._poses: list[Pose] = []  # readout history (index == step t)
        self._initialized = False

    @property
    def name(self) -> str:
        return "particle_filter"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    # --- init ---------------------------------------------------------------

    def _init_particles(self) -> None:
        """Draw N particles in ascending order: the start cell (the one holding
        x0/y0) uniformly in x and y, heading uniform on [−π, π). Three draws per
        particle — x offset, y offset, heading — from the algorithm's own stream."""
        episode = self.episode
        assert episode is not None
        grid = episode.grid
        h = grid.height
        ox, oy = grid.origin
        res = grid.resolution
        row0, col0 = grid.world_to_cell(self._x0, self._y0)
        rng = self._rng
        for _ in range(self._n):
            x = ox + (float(col0) + rng.uniform01()) * res
            y = oy + (float(h - 1 - row0) + rng.uniform01()) * res
            theta = rng.uniform01() * (2.0 * math.pi) - math.pi
            self._x.append(x)
            self._y.append(y)
            self._theta.append(theta)
        self._w = [self._inv_n] * self._n  # uniform prior — exactly 1/N each
        self._initialized = True

    # --- update -------------------------------------------------------------

    def _move(self, u: Twist) -> None:
        """x ⊕ (u + ε), per-particle draws in ascending order (three gaussians =
        six uniforms per particle, fixed order ex, ey, eθ)."""
        rng = self._rng
        for i in range(self._n):
            ex = rng.gaussian(0.0, self._sigma_xy)
            ey = rng.gaussian(0.0, self._sigma_xy)
            et = rng.gaussian(0.0, self._sigma_theta)
            p = pose_compose(
                Pose(self._x[i], self._y[i], self._theta[i]),
                Twist(u.dx + ex, u.dy + ey, u.dtheta + et),
            )
            self._x[i] = p.x
            self._y[i] = p.y
            self._theta[i] = p.theta

    def _weight(self, scan: tuple) -> None:
        """Multiply every particle's weight by exp(ll − max ll); renormalize.

        ll is the sum over this step's scan points IN SCAN ORDER of the beam-model
        Gaussian −½·((r − expected)/σ)²; `expected` is the same DDA raycast the
        simulator fired, run at the particle's own pose along heading + β (a miss
        keeps the sentinel range_max + res — a point exists where the particle sees
        nothing, so it pays the full Gaussian penalty). exp(ll − max) never
        overflows and the argmax keeps its weight exactly; a total of 0.0 would
        make normalization impossible, so the belief simply stays unchanged."""
        episode = self.episode
        assert episode is not None
        grid = episode.grid
        res = grid.resolution
        range_max = self._range_max
        sigma = self._sigma_range
        sentinel = range_max + res

        # (r, β) per scan point, in scan order — the polar form of each robot-frame
        # endpoint. Fixed formulas: sqrt(zx² + zy²), atan2(zy, zx).
        points: list[tuple[float, float]] = []
        for z in scan:
            r = math.sqrt(z[0] * z[0] + z[1] * z[1])
            beta = math.atan2(z[1], z[0])
            points.append((r, beta))

        # Per-particle log-likelihood, ascending particle order.
        m = float("-inf")
        ll: list[float] = []
        for i in range(self._n):
            s = self._theta[i]
            acc = 0.0
            for (r, beta) in points:
                e = raycast(grid, self._x[i], self._y[i], s + beta, range_max)
                if e is None:
                    e = sentinel
                d = (r - e) / sigma
                acc += (-0.5 * d) * d
            ll.append(acc)
            if acc > m:  # max is exact — order-independent by construction
                m = acc

        total = 0.0
        weighted: list[float] = []
        for i in range(self._n):
            wv = self._w[i] * math.exp(ll[i] - m)
            weighted.append(wv)
            total += wv
        if total != 0.0:
            for i in range(self._n):
                self._w[i] = weighted[i] / total

    def _resample_if_effective(self) -> None:
        """Systematic resampling when ESS = 1/Σw² drops below N/2 (Thrun's
        pseudocode 2.8): ONE uniform draw scaled into [0, 1/N), then a walk of the
        cumulative weights picking ancestors — strict `>` at every comparison, and
        the loop keeps its final (unused) u += 1/N so all languages step alike."""
        ess_den = 0.0
        for i in range(self._n):
            wv = self._w[i]
            ess_den += wv * wv
        ess = 1.0 / ess_den
        if ess < float(self._n) / 2.0:
            u = self._rng.uniform01() * self._inv_n
            c = self._w[0]
            i = 0
            xs: list[float] = []
            ys: list[float] = []
            ths: list[float] = []
            for _ in range(self._n):
                while i < self._n - 1 and u > c:
                    i += 1
                    c += self._w[i]
                xs.append(self._x[i])
                ys.append(self._y[i])
                ths.append(self._theta[i])
                u += self._inv_n
            self._x, self._y, self._theta = xs, ys, ths
            self._w = [self._inv_n] * self._n  # equal weights after resampling

    def update(self, step, recorder) -> None:
        """Move (if a command arrived), weight by the scan, resample when the cloud
        collapses, then read out — pose first, cloud second."""
        if not self._initialized:
            self._init_particles()
        if step.odom is not None:
            self._move(step.odom)
        if step.scan:
            self._weight(step.scan)
        self._resample_if_effective()

        # Readout (fixed order): weighted mean x, y ascending; then the circular
        # heading from the sin/cos sums; then population variances in a second
        # ascending pass (θ's deviation wrapped around the seam).
        x_hat = 0.0
        y_hat = 0.0
        s_sin = 0.0
        s_cos = 0.0
        for i in range(self._n):
            wv = self._w[i]
            x_hat += wv * self._x[i]
            y_hat += wv * self._y[i]
            s_sin += wv * math.sin(self._theta[i])
            s_cos += wv * math.cos(self._theta[i])
        theta_hat = math.atan2(s_sin, s_cos)

        var_x = 0.0
        var_y = 0.0
        var_t = 0.0
        for i in range(self._n):
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
                    for i in range(self._n)
                ],
            )

    def finalize(self, recorder) -> EstimateResult:
        return EstimateResult(poses=tuple(self._poses))
