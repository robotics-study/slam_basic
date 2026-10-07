"""mcl tests — the KLD-sampling contract, pinned bit-for-bit.

The unit tests pin what the scenario run rests on: kld_bound is Fox's Wilson–Hilferty
closed form at pinned exact doubles (0.0 for the degenerate k ≤ 1); the KLD loop draws
samples ONE AT A TIME — ancestor walk with strict `>` and guard i < n−1, three gaussians,
pose_compose on the OLD pose — until n ≥ n_chi(k) AND n ≥ n_min, or the cap; and the
readout is the particle_filter page's (weighted mean, circular heading, wrapped spread).

The scenario run pins the branch story end-to-end on corridor03_drift: t = 0 draws the
FULL prior budget (800 particles, all distinct — no resampling has happened yet; both
east- and west-facing twins survive); at t = 1 the first KLD step samples from that
weighted cloud until its bin count k justifies stopping (k = 41 → ceil(n_chi) = 319);
once the west blob dies k collapses to 2 and every later step draws exactly
ceil(n_chi(2)) = 33 samples — sample count IS uncertainty, and the U-turn re-widens it
(k spikes to 4/5/3 as the cluster crosses bins and the ±π seam). Everything below is
deterministic: seed 42 pins these numbers bit-for-bit in both languages.
"""

import io
import json
import math
from itertools import pairwise

from conftest import SCENARIO_DIR, config, grid_from

from slam.core.params import ParamDecl, ParamError, ParamSet
from slam.core.rng import Rng
from slam.core.sim import build_episode
from slam.core.stats import inv_norm_cdf
from slam.core.trace import TraceRecorder
from slam.core.types import Episode, Pose, Step, Twist
from slam.filtering.mcl import Mcl, kld_bound
from slam.maps.loader import load_scenario

# Free cells row-major: (0,1) → center (1.5, 1.5), (0,2) → (2.5, 1.5), (1,2) → (2.5, 0.5).
MINI = ["#..", "##."]


def _params(n_min: int = 2, max_particles: int = 8, seed: int = 7,
            sigma_xy: float = 0.25, sigma_theta: float = 0.1, range_max: float = 4.0,
            sigma_range: float = 0.3) -> ParamSet:
    """A tiny config for unit tests (the start cell is the one holding (1.5, 1.5))."""
    return ParamSet(
        "mcl",
        "filtering",
        {
            "seed": ParamDecl("seed", "int", seed),
            "x0": ParamDecl("x0", "float", 1.5),
            "y0": ParamDecl("y0", "float", 1.5),
            "epsilon": ParamDecl("epsilon", "float", 0.1, min=0.001, max=1.0),
            "delta": ParamDecl("delta", "float", 0.01, min=0.0001, max=0.5),
            "bin_xy": ParamDecl("bin_xy", "float", 0.5, min=0.05),
            "bin_theta": ParamDecl("bin_theta", "float", 0.25, min=0.001),
            "n_min": ParamDecl("n_min", "int", n_min, min=1, max=4096),
            # Unit-test config uses a smaller floor than the shipped yaml (min 8) so
            # tiny clouds can pin the mechanics exactly.
            "max_particles": ParamDecl("max_particles", "int", max_particles, min=2, max=4096),
            "range_max": ParamDecl("range_max", "float", range_max, min=0.1),
            "sigma_range": ParamDecl("sigma_range", "float", sigma_range, min=0.001),
            "sigma_xy": ParamDecl("sigma_xy", "float", sigma_xy, min=0.001, max=0.5),
            "sigma_theta": ParamDecl("sigma_theta", "float", sigma_theta,
                                     min=0.001, max=3.14159265358979),
        },
    )


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(config("mcl"))
    assert params.algorithm == "mcl" and params.section == "filtering"
    assert params.get_int("seed") == 42 and params.get_int("max_particles") == 800
    assert params.get_int("n_min") == 10
    # The prior-cell defaults are the scenario start cell center, exact binary floats.
    assert params.get_float("x0") == 4.25 and params.get_float("y0") == 1.75
    assert params.get_float("epsilon") == 0.1 and params.get_float("delta") == 0.01
    assert params.get_float("bin_xy") == 0.5
    # bin_theta is exactly radians(10°) — the paper's 10° heading bin as a binary float.
    assert params.get_float("bin_theta") == math.radians(10) == 0.17453292519943295
    assert params.get_float("range_max") == 2.5 and params.get_float("sigma_range") == 0.3
    assert params.get_float("sigma_xy") == 0.05 and params.get_float("sigma_theta") == 0.05
    # Out-of-range sets raise (the demo injects the scenario sensor through set()).
    try:
        params.set("max_particles", 8192)
        raise AssertionError("expected ParamError for max_particles > max")
    except ParamError:
        pass
    # The bound's quantile is computed ONCE from delta — pinned to the exact double.
    est = Mcl(params)
    assert est._z_q == 2.326347874388028  # inv_norm_cdf(1 − 0.01), pinned in test_stats


def test_kld_bound_is_wilson_hilferty() -> None:
    """n_chi(k) = (k−1)/2ε · ((1 − 2/9ν) + z_q·√(2/9ν))³ at ε = 0.1 and the pinned
    z_q — exact doubles, pinned; k ≤ 1 is the degenerate case (bound 0.0: a one-bin
    belief needs no guarantee beyond n_min), and the bound grows with k."""
    z_q = inv_norm_cdf(0.99)
    assert z_q == 2.326347874388028
    assert kld_bound(1, 0.1, z_q) == 0.0 and kld_bound(0, 0.1, z_q) == 0.0
    assert kld_bound(2, 0.1, z_q) == 32.92886549325934
    assert kld_bound(3, 0.1, z_q) == 46.102526736216774
    assert kld_bound(4, 0.1, z_q) == 56.8452881537776
    assert kld_bound(5, 0.1, z_q) == 66.52863321822623
    # Monotone in k (and ε shrinks the bound: n ∝ 1/2ε at fixed k).
    values = [kld_bound(k, 0.1, z_q) for k in range(2, 40)]
    assert all(a < b for a, b in pairwise(values))
    assert kld_bound(5, 0.2, z_q) == kld_bound(5, 0.1, z_q) / 2.0


def test_init_draws_ascending_and_uniform() -> None:
    """Three particles, seed pinned: x = ox + (col0 + u1)·res, y = oy + (h−1−row0 +
    u2)·res, θ = u3·2π − π — three draws per particle IN ASCENDING PARTICLE ORDER
    from the algorithm's own stream (replayed with a second identical Rng). Weights
    come out exactly 1/N: at t = 0 the cloud is the FIXED prior budget."""
    est = Mcl(_params(max_particles=3, seed=7))
    est.episode = Episode(steps=(), landmarks=None, grid=grid_from(MINI))
    est._init_particles()

    rng = Rng(7)
    xs, ys, ths = [], [], []
    for _ in range(3):  # cell (0,1): col0 = 1 and h − 1 − row0 = 2 − 1 − 0 = 1
        xs.append(0.0 + (1.0 + rng.uniform01()) * 1.0)
        ys.append(0.0 + (2 - 1 - 0 + rng.uniform01()) * 1.0)
        ths.append(rng.uniform01() * (2.0 * math.pi) - math.pi)
    assert est._x == xs and est._y == ys and est._theta == ths  # bit-for-bit
    assert est._w == [1.0 / 3.0] * 3  # uniform prior — exactly 1/N each


def test_kld_update_draws_one_at_a_time_until_the_cap() -> None:
    """With n_min = max_particles the loop draws EXACTLY that many samples regardless
    of bins — so the whole per-sample chain replays bit-for-bit with a second Rng:
    ancestor walk (strict `>`, guard i < n_old − 1), three gaussians, pose_compose on
    the OLD pose. No scan → every ll is 0.0 → weights come out exactly uniform, and
    the stream advanced by exactly 3 × 7 draws (one uniform + three gaussians)."""
    est = Mcl(_params(n_min=8, max_particles=3, seed=11))
    est.episode = Episode(steps=(), landmarks=None, grid=grid_from(MINI))
    est._initialized = True
    old = [Pose(1.5, 1.5, 0.25), Pose(9.0, 0.5, -1.0)]
    est._x = [p.x for p in old]
    est._y = [p.y for p in old]
    est._theta = [p.theta for p in old]
    est._w = [0.25, 0.75]

    rng = Rng(11)
    expect: list[Pose] = []
    for _ in range(3):
        u1 = rng.uniform01()
        i, c = 0, 0.25
        while i < 1 and u1 > c:  # strict `>` over the cumulative weights
            i += 1
            c += 0.75
        ex = rng.gaussian(0.0, 0.25)
        ey = rng.gaussian(0.0, 0.25)
        et = rng.gaussian(0.0, 0.1)
        c_, s_ = math.cos(old[i].theta), math.sin(old[i].theta)
        dx, dy, dth = 0.5 + ex, -0.25 + ey, 0.3 + et
        t = old[i].theta + dth
        expect.append(Pose(
            old[i].x + c_ * dx - s_ * dy,
            old[i].y + s_ * dx + c_ * dy,
            t - 2.0 * math.pi * math.floor((t + math.pi) / (2.0 * math.pi)),
        ))
    est._kld_update(Twist(dx=0.5, dy=-0.25, dtheta=0.3), None)
    assert est._x == [p.x for p in expect]  # bit-for-bit — same operation order
    assert est._y == [p.y for p in expect]
    assert est._theta == [p.theta for p in expect]
    assert est._w == [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0]  # no scan: every ll is 0.0
    # The stream advanced by exactly 3 × 7 draws (one uniform + three gaussians each).
    assert est._rng.uniform01() == rng.uniform01()


def test_kld_update_stops_at_n_min_when_one_bin_is_enough() -> None:
    """k = 1 → kld_bound is 0.0, so the stop rule reduces to n ≥ n_min. Pin the one
    ancestor at a bin CENTER (x, y and θ all mid-bin) with u = 0 and σ pinned to its
    minimum: both draws land in that same single bin, k stays 1, the bound stays 0 —
    the loop stops exactly at n_min = 2 and the cap never fires."""
    est = Mcl(_params(n_min=2, max_particles=4096, seed=5,
                     sigma_xy=0.001, sigma_theta=0.001))
    est.episode = Episode(steps=(), landmarks=None, grid=grid_from(MINI))
    est._initialized = True
    est._x, est._y, est._theta, est._w = [1.75], [3.75], [0.125], [1.0]

    rng = Rng(5)
    expect: list[Pose] = []
    for _ in range(2):  # n_old = 1 → the walk always lands on ancestor 0
        rng.uniform01()  # consumed and discarded — fixed draw order
        ex = rng.gaussian(0.0, 0.001)
        ey = rng.gaussian(0.0, 0.001)
        et = rng.gaussian(0.0, 0.001)
        c_, s_ = math.cos(0.125), math.sin(0.125)
        expect.append(Pose(
            1.75 + c_ * ex - s_ * ey,
            3.75 + s_ * ex + c_ * ey,
            (0.125 + et) - 2.0 * math.pi * math.floor((0.125 + et + math.pi) / (2.0 * math.pi)),
        ))
    est._kld_update(Twist(dx=0.0, dy=0.0, dtheta=0.0), None)
    assert len(est._x) == 2  # stopped at n_min — the bound was 0 (single bin)
    assert est._x == [p.x for p in expect] and est._y == [p.y for p in expect]
    assert est._theta == [p.theta for p in expect]
    assert est._w == [0.5, 0.5]


def test_readout_mean_circular_and_spread() -> None:
    """Readout order (identical to the particle_filter page): weighted mean x/y,
    circular heading atan2(Σ w sinθ, Σ w cosθ) — the arithmetic mean of angles lies
    across the ±π seam; this one cannot — then population variances in a second
    ascending pass with θ's deviation wrapped. Events: pose_estimated first,
    particles_updated second, same t."""
    est = Mcl(_params(seed=7))
    est.episode = Episode(steps=(), landmarks=None, grid=grid_from(MINI))
    est._initialized = True
    # Straddle the seam: an arithmetic mean of angles would land near 0; the circular
    # mean must land near ±π.
    est._x, est._y, est._theta = [1.0, 2.0, 3.0], [3.0, 4.0, 5.0], [3.0, -3.0, 0.0]
    est._w = [0.5, 0.25, 0.25]

    buf = io.StringIO()
    est.update(Step(t=7, gt=Pose(0.0, 0.0, 0.0), odom=None, scan=None, obs=None),
               TraceRecorder(buf))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["pose_estimated", "particles_updated"]
    assert events[0]["t"] == 7 and events[1]["t"] == 7

    # Accumulated EXACTLY like the implementation: ascending sequential += from 0.0
    # (Python's sum() is compensated summation — a different rounding on purpose).
    w = (0.5, 0.25, 0.25)
    x_hat = y_hat = s_sin = s_cos = 0.0
    for wi, xi, yi, ti in zip(w, est._x, est._y, est._theta, strict=True):
        x_hat += wi * xi
        y_hat += wi * yi
        s_sin += wi * math.sin(ti)
        s_cos += wi * math.cos(ti)
    theta_hat = math.atan2(s_sin, s_cos)
    assert abs(theta_hat) > 3.0  # near ±π — the seam did not explode the mean
    var_x = var_y = var_t = 0.0
    for wi, xi, yi, ti in zip(w, est._x, est._y, est._theta, strict=True):
        dx = xi - x_hat
        dy = yi - y_hat
        dt0 = ti - theta_hat
        dt = dt0 - 2.0 * math.pi * math.floor((dt0 + math.pi) / (2.0 * math.pi))
        var_x += wi * (dx * dx)
        var_y += wi * (dy * dy)
        var_t += wi * (dt * dt)
    assert events[0]["pose"] == [x_hat, y_hat, theta_hat]
    assert events[0]["cov"] == [math.sqrt(var_x), math.sqrt(var_y), math.sqrt(var_t)]
    # The cloud payload is [x, y, θ, w] per particle in ascending order.
    assert events[1]["particles"] == [[1.0, 3.0, 3.0, 0.5], [2.0, 4.0, -3.0, 0.25],
                                      [3.0, 5.0, 0.0, 0.25]]


def test_run_on_scenario_sample_count_is_uncertainty() -> None:
    """The branch story end-to-end (the demo's exact inputs — the scenario seed and
    sensor injected through set(), epsilon/delta at their config defaults): t = 0 draws
    the FULL budget (800 particles, ALL distinct — nothing has resampled yet; both
    twins survive: 427 of them face east) and t = 1 is the first KLD step. From there
    every step's sample count IS the bound: n(t) == ceil(n_chi(k(t))) exactly, where
    k(t) is the distinct-bin count recomputed here from the emitted cloud — 33 while
    the belief sits in two bins, spiking to 47/57/67 when the U-turn drags the cluster
    across bins and the ±π seam. The readout tracks ground truth to centimetres."""
    sc = load_scenario(SCENARIO_DIR / "corridor03_drift.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(config("mcl"))
    # The demo's injection contract — only the fields this config declares.
    params.set("seed", int(sc.seed))
    params.set("range_max", float(sc.sensor.range_max))
    params.set("sigma_range", float(sc.sensor.sigma_range))

    buf = io.StringIO()
    est = Mcl(params)
    result = est.run(episode, TraceRecorder(buf))
    assert len(result.poses) == len(episode.steps) == 37

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    names = [e["event"] for e in events]
    assert names == ["step_observed", "pose_estimated", "particles_updated"] * 37

    clouds = [e for e in events if e["event"] == "particles_updated"]
    n_t = [len(e["particles"]) for e in clouds]
    # t = 0: the fixed prior budget, ALL distinct (no resampling has happened yet),
    # and BOTH headings survive the point-symmetric start cell (427 of 800 face east).
    assert n_t[0] == 800
    assert len({(p[0], p[1], p[2]) for p in clouds[0]["particles"]}) == 800
    east = sum(1 for p in clouds[0]["particles"] if math.cos(p[2]) > 0)
    assert east == 427
    pes = [e for e in events if e["event"] == "pose_estimated"]
    assert pes[0]["cov"][2] > 1.0  # heading std dev is radian-wide, not pretend-sharp

    # Every t ≥ 1: the sample count equals ceil(n_chi(k)) EXACTLY — k recomputed here
    # from the emitted cloud (bin_xy = bin_theta come from the config; every value sits
    # above n_min and below the cap, so the bound is what stopped the loop).
    z_q = inv_norm_cdf(1.0 - 0.01)
    for t in range(1, 37):
        k = len({(math.floor(p[0] / 0.5), math.floor(p[1] / 0.5),
                  math.floor(p[2] / math.radians(10))) for p in clouds[t]["particles"]})
        assert n_t[t] == math.ceil(kld_bound(k, 0.1, z_q))

    # The pinned sequence: full budget at t = 0; the first KLD step samples until k = 41
    # bins justify 319; once the west blob dies k = 2 → 33; the U-turn re-widens (k
    # spikes as the cluster crosses bins and the ±π seam) then settles back to 33.
    assert n_t == ([800, 319] + [33] * 13 + [57, 33, 67, 47] + [33] * 18)

    total = 0.0
    for est_pose, step in zip(result.poses, episode.steps, strict=True):
        dx, dy = est_pose.x - step.gt.x, est_pose.y - step.gt.y
        total += dx * dx + dy * dy
    ate = math.sqrt(total / len(result.poses))
    assert 0.01 < ate < 0.1  # measured ≈ 0.0516 — σ/√k honest floor, not a bug
    final, gt_final = result.poses[-1], episode.steps[-1].gt
    assert abs(final.x - gt_final.x) < 0.06 and abs(final.y - gt_final.y) < 0.06
    d_theta = final.theta - gt_final.theta
    assert abs(d_theta - 2.0 * math.pi * math.floor((d_theta + math.pi) / (2.0 * math.pi))) < 0.06
