"""particle_filter tests — the bootstrap filter's fixed-order contract.

The unit tests pin the mechanics every run rests on: initialization draws in
ascending particle order (x offset, y offset, heading — three uniforms each), the
motion step composes pose_compose(p, u + ε) with two-draw gaussians drawn in the
same ascending order, the weight pass is max-shifted exps accumulated ascending and
normalized ascending (a miss keeps the sentinel range_max + res), and systematic
resampling triggers strictly below ESS = 1/Σw² < N/2 — w = [1, 0] has ESS exactly
1.0 on n = 2, so it must NOT resample, bit-for-bit, and the walk consumes exactly
ONE uniform draw scaled into [0, 1/N).

The scenario run pins the branch story end-to-end on corridor03_drift: at t = 0 the
point-symmetric start cell keeps BOTH east- and west-facing particles alive (42
distinct ancestors, radian-wide heading spread); the first eastward move kills the
west blob — by t = 1 a SINGLE ancestor survives, the covariance collapses to float
noise, and from there the readout tracks ground truth to centimetres. The whole run
is deterministic: seed 42 pins these numbers bit-for-bit in both languages.
"""

import io
import json
import math

from conftest import SCENARIO_DIR, config, grid_from

from slam.core.geometry import wrap
from slam.core.params import ParamDecl, ParamError, ParamSet
from slam.core.rng import Rng
from slam.core.sim import build_episode, raycast
from slam.core.trace import TraceRecorder
from slam.core.types import Episode, Pose, Step, Twist
from slam.filtering.particle_filter import ParticleFilter
from slam.maps.loader import load_scenario

# Free cells row-major: (0,1) → center (1.5, 1.5), (0,2) → (2.5, 1.5), (1,2) → (2.5, 0.5).
MINI = ["#..", "##."]


def _params(n: int = 3, seed: int = 7, sigma_xy: float = 0.25,
            sigma_theta: float = 0.1, range_max: float = 4.0,
            sigma_range: float = 0.3) -> ParamSet:
    """A tiny config for unit tests (the start cell is the one holding (1.5, 1.5))."""
    return ParamSet(
        "particle_filter",
        "filtering",
        {
            "seed": ParamDecl("seed", "int", seed),
            "n_particles": ParamDecl("n_particles", "int", n, min=2, max=4096),
            "x0": ParamDecl("x0", "float", 1.5),
            "y0": ParamDecl("y0", "float", 1.5),
            "range_max": ParamDecl("range_max", "float", range_max, min=0.1),
            "sigma_range": ParamDecl("sigma_range", "float", sigma_range, min=0.001),
            "sigma_xy": ParamDecl("sigma_xy", "float", sigma_xy, min=0.001, max=0.5),
            "sigma_theta": ParamDecl("sigma_theta", "float", sigma_theta,
                                     min=0.001, max=3.14159265358979),
        },
    )


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(config("particle_filter"))
    assert params.algorithm == "particle_filter" and params.section == "filtering"
    assert params.get_int("seed") == 42 and params.get_int("n_particles") == 500
    # The prior-cell defaults are the scenario start cell center, exact binary floats.
    assert params.get_float("x0") == 4.25 and params.get_float("y0") == 1.75
    assert params.get_float("range_max") == 2.5 and params.get_float("sigma_range") == 0.3
    assert params.get_float("sigma_xy") == 0.05 and params.get_float("sigma_theta") == 0.05
    # Out-of-range sets raise (the demo injects the scenario sensor through set()).
    try:
        params.set("n_particles", 8192)
        raise AssertionError("expected ParamError for n_particles > max")
    except ParamError:
        pass


def test_init_draws_ascending_and_uniform() -> None:
    """Three particles, seed pinned: x = ox + (col0 + u1)·res, y = oy + (h−1−row0 +
    u2)·res, θ = u3·2π − π — three draws per particle IN ASCENDING PARTICLE ORDER
    from the algorithm's own stream (replayed here with a second identical Rng).
    Weights come out exactly 1/N."""
    est = ParticleFilter(_params(n=3, seed=7))
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


def test_move_composes_twist_with_drawn_noise() -> None:
    """x ⊕ (u + ε): three gaussians per particle (two uniform draws each) drawn in
    ascending order, composed with the arriving twist on the OLD pose."""
    est = ParticleFilter(_params(n=2, seed=11))
    est._initialized = True
    old = [Pose(1.5, 1.5, 0.25), Pose(2.5, 0.5, -1.0)]
    est._x = [p.x for p in old]
    est._y = [p.y for p in old]
    est._theta = [p.theta for p in old]
    est._w = [0.5, 0.5]

    rng = Rng(11)
    expect: list[Pose] = []
    for i in range(2):
        ex = rng.gaussian(0.0, 0.25)
        ey = rng.gaussian(0.0, 0.25)
        et = rng.gaussian(0.0, 0.1)
        c, s = math.cos(old[i].theta), math.sin(old[i].theta)
        dx, dy, dth = 0.5 + ex, -0.25 + ey, 0.3 + et
        expect.append(Pose(
            old[i].x + c * dx - s * dy,
            old[i].y + s * dx + c * dy,
            wrap(old[i].theta + dth),
        ))
    est._move(Twist(dx=0.5, dy=-0.25, dtheta=0.3))
    assert est._x == [p.x for p in expect]  # bit-for-bit — same operation order
    assert est._y == [p.y for p in expect]
    assert est._theta == [p.theta for p in expect]


def test_weight_is_max_shifted_and_normalized() -> None:
    """Each scan point contributes −½·((r − expected)/σ)² at the particle's own pose
    along heading + β (a miss pays the sentinel range_max + res); exps are max-shifted
    (the argmax keeps its weight exactly) and normalized ascending. A step with no
    scan weights nothing, and normalized n = 2 weights can never trigger a resample
    (ESS ≥ 1 is never strictly below N/2 = 1)."""
    grid = grid_from(MINI)
    est = ParticleFilter(_params(n=2, sigma_range=0.3, range_max=4.0))
    est.episode = Episode(steps=(), landmarks=None, grid=grid)
    est._initialized = True
    est._x, est._y, est._theta = [1.5, 2.5], [1.5, 0.5], [0.0, -1.0]
    est._w = [0.25, 0.75]

    scan = ((1.25, 0.3), (2.0, -0.75))
    sentinel = 4.0 + 1.0  # range_max + res — a miss keeps the sentinel exactly
    ll: list[float] = []
    for i in range(2):
        acc = 0.0
        for (r, beta) in scan:
            e = raycast(grid, est._x[i], est._y[i], est._theta[i] + beta, 4.0)
            if e is None:
                e = sentinel
            d = (r - e) / 0.3
            acc += (-0.5 * d) * d
        ll.append(acc)
    m = max(ll)
    weighted = [est._w[i] * math.exp(ll[i] - m) for i in range(2)]
    total = weighted[0] + weighted[1]
    expect = [weighted[0] / total, weighted[1] / total]

    est._weight(scan)
    assert est._w == expect  # bit-for-bit: same exps, same ascending accumulation

    # A step with no scan weights nothing (and cannot resample — see the docstring).
    before = list(est._w)
    est.update(Step(t=0, gt=Pose(0.0, 0.0, 0.0), odom=None, scan=None, obs=None), None)
    assert est._w == before


def test_resample_trigger_and_walk() -> None:
    """ESS = 1/Σw² triggers strictly BELOW N/2: w = [1, 0] on n = 2 has ESS exactly
    1.0 and must NOT resample (bits untouched, and the rng stream consumes NOTHING).
    A real collapse resamples with ONE draw u ∈ [0, 1/N): the walk advances while
    u > c (strict) over the cumulative weights, copies that ancestor, and steps
    u += 1/N after EVERY copy — then every weight is exactly 1/N."""
    # No trigger: ESS = 1/(1² + 0²) = 1.0, threshold N/2 = 1.0, strict < fails.
    est_a = ParticleFilter(_params(n=2, seed=5))
    est_a._initialized = True
    est_a._x, est_a._y, est_a._theta, est_a._w = [1.0, 2.0], [3.0, 4.0], [0.5, -0.5], [1.0, 0.0]
    est_a._resample_if_effective()
    assert (est_a._x, est_a._y, est_a._theta) == ([1.0, 2.0], [3.0, 4.0], [0.5, -0.5])
    assert est_a._w == [1.0, 0.0]  # bit-for-bit untouched
    assert est_a._rng.uniform01() == Rng(5).uniform01()  # the stream never advanced

    # Trigger: ESS = 1/(0.2² + 0.8²) ≈ 1.47 < 1.5. The walk's strict `>` and the
    # post-copy u += 1/N are replayed here with a second identical Rng.
    est_b = ParticleFilter(_params(n=3, seed=5))
    est_b._initialized = True
    xs_old, ys_old, ths_old = [1.0, 2.0, 3.0], [3.0, 4.0, 5.0], [0.5, -0.5, 1.0]
    w_old = [0.2, 0.8, 0.0]
    est_b._x, est_b._y, est_b._theta, est_b._w = xs_old, ys_old, ths_old, list(w_old)

    rng = Rng(5)
    u = rng.uniform01() * (1.0 / 3.0)
    xs, ys, ths = [], [], []
    c, i = w_old[0], 0
    for _ in range(3):
        while i < 2 and u > c:
            i += 1
            c += w_old[i]
        xs.append(xs_old[i])
        ys.append(ys_old[i])
        ths.append(ths_old[i])
        u += 1.0 / 3.0  # including the final, unused increment — fixed loop shape
    est_b._resample_if_effective()
    assert (est_b._x, est_b._y, est_b._theta) == (xs, ys, ths)  # bit-for-bit ancestors
    assert est_b._w == [1.0 / 3.0] * 3  # equal weights after resampling — exactly 1/N
    # Exactly ONE draw was consumed: the stream's next draw is the fresh stream's second.
    assert est_b._rng.uniform01() == rng.uniform01()


def test_readout_mean_circular_and_spread() -> None:
    """Readout order: weighted mean x/y, circular heading atan2(Σ w sinθ, Σ w cosθ)
    (the arithmetic mean of angles lies across the ±π seam; this one cannot), then
    population variances in a second ascending pass with θ's deviation wrapped.
    Events: pose_estimated first, particles_updated second, same t."""
    est = ParticleFilter(_params(n=3, seed=7))
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
        var_x += wi * (xi - x_hat) ** 2
        var_y += wi * (yi - y_hat) ** 2
        var_t += wi * wrap(ti - theta_hat) ** 2
    assert events[0]["pose"] == [x_hat, y_hat, theta_hat]
    assert events[0]["cov"] == [math.sqrt(var_x), math.sqrt(var_y), math.sqrt(var_t)]
    # The cloud payload is [x, y, θ, w] per particle in ascending order.
    assert events[1]["particles"] == [[1.0, 3.0, 3.0, 0.5], [2.0, 4.0, -3.0, 0.25],
                                      [3.0, 5.0, 0.0, 0.25]]


def test_run_on_scenario_converges_from_the_blob() -> None:
    """The branch story end-to-end (the demo's exact inputs — the scenario seed and
    sensor injected through set(), sigma_xy/sigma_theta left at the config defaults):
    t = 0 the uniform cloud keeps BOTH blobs alive (42 distinct ancestors after the
    first resample) and the readout is their blob-mean heading, NOT ground truth;
    the first eastward move kills the west blob — by t = 1 a SINGLE ancestor survives
    and from there the estimate tracks ground truth to centimetres. Everything below
    is deterministic: seed 42 pins these numbers bit-for-bit in both languages."""
    sc = load_scenario(SCENARIO_DIR / "corridor03_drift.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(config("particle_filter"))
    # The demo's injection contract — only the fields this config declares.
    params.set("seed", int(sc.seed))
    params.set("range_max", float(sc.sensor.range_max))
    params.set("sigma_range", float(sc.sensor.sigma_range))

    buf = io.StringIO()
    est = ParticleFilter(params)
    result = est.run(episode, TraceRecorder(buf))
    assert len(result.poses) == len(episode.steps) == 37

    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    names = [e["event"] for e in events]
    assert names == ["step_observed", "pose_estimated", "particles_updated"] * 37

    # t = 0: resampling fired (weights exactly uniform 1/500 again) and 42 distinct
    # ancestors survived — BOTH headings alive: the point-symmetric start cell lets
    # east-facing AND west-facing particles fit the first scan, so the heading spread
    # stays radian-wide while position is already cell-tight.
    clouds = [e for e in events if e["event"] == "particles_updated"]
    assert all(p[3] == 1.0 / 500.0 for p in clouds[0]["particles"])
    assert len({(p[0], p[1], p[2]) for p in clouds[0]["particles"]}) == 42
    east = sum(1 for p in clouds[0]["particles"] if math.cos(p[2]) > 0)
    assert 0 < east < 500  # both directions survive — a real multi-hypothesis cloud
    pes = [e for e in events if e["event"] == "pose_estimated"]
    assert pes[0]["cov"][2] > 1.0  # heading std dev is radian-wide, not pretend-sharp

    # t = 1: the first eastward move kills the west blob — ONE ancestor survives and
    # the covariance collapses to float noise; from there the readout tracks.
    assert len({(p[0], p[1], p[2]) for p in clouds[1]["particles"]}) == 1
    assert all(c < 1e-9 for c in pes[1]["cov"])

    total = 0.0
    for est_pose, step in zip(result.poses, episode.steps, strict=True):
        dx, dy = est_pose.x - step.gt.x, est_pose.y - step.gt.y
        total += dx * dx + dy * dy
    ate = math.sqrt(total / len(result.poses))
    assert 0.01 < ate < 0.1  # measured ≈ 0.0569 — σ/√k honest floor, not a bug
    final, gt_final = result.poses[-1], episode.steps[-1].gt
    assert abs(final.x - gt_final.x) < 0.06 and abs(final.y - gt_final.y) < 0.06
    assert abs(wrap(final.theta - gt_final.theta)) < 0.06
