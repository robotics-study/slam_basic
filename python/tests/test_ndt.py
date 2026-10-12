"""ndt tests — Normal Distributions Transform scan matching, pinned bit-for-bit.

The unit tests pin what the scenario run rests on: an empty side or a degenerate
cluster (fewer than 3 points, or scatter determinant exactly 0.0) carries no field
and returns the identity twist exactly; and a single four-point cluster shifted by
−0.25 does NOT return +0.25 — one cluster's bumps sit closer together than the blob
is wide, the score has several stationary points, and Newton-with-backtracking slides
to the nearest one (pinned to the last bit: dx = 0.4000…, not 0.25). That is NDT's
capture basin — the analog of ICP's aliased lattice pairs, pinned here exactly like
ICP pins its wrong fixed point (+0.1).

The scenario run pins the branch story end-to-end on room02_diagonal: t = 0 emits
the DECLARED gauge pose exactly (1.75, 1.75, 0) — registration measures relative
motion only — and every later step composes one recovered twist. The estimator never
reads Step.odom: rebuilding the episode with σ_xy = 5.0 changes not one bit of the
result. ATE/RPE are pinned to exact doubles (seed 42 pins them in both languages).
"""

import io
import json
import math

from conftest import SCENARIO_DIR, config

from slam.core.geometry import pose_minus
from slam.core.params import ParamError, ParamSet
from slam.core.sim import build_episode
from slam.core.trace import TraceRecorder
from slam.core.types import Pose, Step, Twist
from slam.maps.loader import load_scenario
from slam.registration.ndt import Ndt, ndt_step


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(config("ndt"))
    assert params.algorithm == "ndt" and params.section == "registration"
    assert params.scenarios == ["room02_diagonal"]
    # The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
    assert params.get_float("x0") == 1.75 and params.get_float("y0") == 1.75
    assert params.get_float("theta_deg") == 0.0
    assert params.get_float("cell_size") == 0.5
    assert params.get_float("lam0") == 1e-6 and params.get_float("eps") == 1e-9
    assert params.get_int("max_iters") == 64
    # Out-of-range sets raise (no silent clamping — the contract is a range check).
    for key, bad in (("cell_size", 0.1), ("lam0", 2.0), ("eps", 0.2), ("max_iters", 0)):
        try:
            params.set(key, bad)
            raise AssertionError(f"expected ParamError for {key} = {bad!r}")
        except ParamError:
            pass


def test_degenerate_inputs_are_identity() -> None:
    """Empty source or target returns the identity twist exactly — no information, no
    motion. A cluster of fewer than 3 points carries NO distribution (n < 3 is a
    skip), and three collinear points have scatter determinant EXACTLY 0.0 — also a
    skip; with no cell left holding a field the score is constant, g = 0 exactly, and
    the Cramer step is 0/det = 0 at every iteration."""
    id_ = Twist(dx=0.0, dy=0.0, dtheta=0.0)
    assert ndt_step((), ((1.0, 1.0),), 1.0, 1e-6, 1e-9, 64) == id_
    assert ndt_step(((5.0, 5.0),), (), 1.0, 1e-6, 1e-9, 64) == id_
    # n < 3: a single point can carry no distribution — the score is constant.
    assert ndt_step(((5.0, 5.0),), ((2.0, 1.0),), 1.0, 1e-6, 1e-9, 64) == id_
    # Collinear target: sxx = 0 exactly (all x equal), so det = sxx·syy − sxy² is
    # exactly 0.0 — the cell carries no distribution and the solve stays identity.
    got = ndt_step(((2.0, 1.0), (2.0, 1.5), (2.0, 2.0)), ((3.0, 1.0), (3.0, 1.5), (3.0, 2.0)),
                   1.0, 1e-6, 1e-9, 64)
    assert got == id_


def test_one_cluster_is_a_capture_basin_not_an_exact_shift() -> None:
    """The honest limit, pinned. One cluster of four points (μ = (2.5, 1.5), sxy = 0
    exactly); the source is the same cross shifted −0.25 in x. A point-to-point rule
    would return +0.25 EXACTLY — NDT cannot: its score is a SUM of four Gaussian
    bumps whose centers (μ − q_i) sit at (0.5, 0), (0, 0) and (0.25, ±0.25), closer
    together than the blob is wide (σ_eff = √0.125 ≈ 0.354). The score has several
    stationary points; Newton-with-backtracking slides to the nearest one — pinned at
    dx = 0.4000…, NOT +0.25 — and even a single iteration already lands at 0.3933….
    Capture is soft here, exactly like ICP's aliased lattice pairs, and this test is
    the pin that keeps it honest."""
    tgt = ((2.25, 1.5), (2.75, 1.5), (2.5, 1.25), (2.5, 1.75))
    src = ((2.0, 1.5), (2.5, 1.5), (2.25, 1.25), (2.25, 1.75))
    got = ndt_step(src, tgt, 1.0, 1e-6, 1e-9, 64)
    assert got.dx == 0.4000430730746629
    assert got.dy == -0.20372559558711778
    assert got.dtheta == 0.09359947373033549
    once = ndt_step(src, tgt, 1.0, 1e-6, 1e-9, 1)
    assert once.dx == 0.3933349995237937
    assert once.dy == 1.5137416256439681e-07
    assert once.dtheta == -6.718066401189245e-08


def test_update_emits_gauge_then_poses_without_cov() -> None:
    """t = 0 adopts the declared gauge pose and emits it WITHOUT cov (NDT carries no
    uncertainty model); a scan-less step carries the estimate forward untouched."""
    params = ParamSet.from_yaml(config("ndt"))
    est = Ndt(params)
    buf = io.StringIO()
    with TraceRecorder(buf) as rec:
        est.update(Step(t=0, gt=Pose(0.0, 0.0, 0.0), odom=None, scan=None, obs=None), rec)
        est.update(Step(t=1, gt=Pose(9.0, 9.0, 0.0), odom=Twist(0.25, 0.0, 0.0),
                        scan=None, obs=None), rec)
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["pose_estimated", "pose_estimated"]
    assert events[0]["t"] == 0 and events[1]["t"] == 1
    assert events[0]["pose"] == [1.75, 1.75, 0.0]  # the declared gauge, exactly
    assert events[1]["pose"] == [1.75, 1.75, 0.0]   # scan-less step: identity carried
    assert "cov" not in events[0] and "cov" not in events[1]


def test_run_on_scenario_is_registration_not_odometry() -> None:
    """The branch story end-to-end on room02_diagonal (the demo's exact inputs — no
    sensor params are declared, so the demo injects NOTHING): the gauge pose is
    emitted verbatim at t = 0, every later step composes one recovered twist, and
    Step.odom NEVER matters — rebuilding the episode with σ_xy = 5.0 changes not one
    bit of any estimate. ATE/RPE pinned to exact doubles (seed 42 pins them in both
    languages); the four composed poses pin every bit between them."""
    sc = load_scenario(SCENARIO_DIR / "room02_diagonal.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(config("ndt"))  # no sensor params declared — none injected

    buf = io.StringIO()
    est = Ndt(params)
    result = est.run(episode, TraceRecorder(buf))
    assert len(result.poses) == len(episode.steps) == 5
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    names = [e["event"] for e in events]
    assert names == ["step_observed", "pose_estimated"] * 5
    pes = [e for e in events if e["event"] == "pose_estimated"]
    assert all("cov" not in e for e in pes)
    # t = 0 is the declared gauge, bit-for-bit; odom arrives at t ≥ 1 (σ = 0 here —
    # the EXACT command, which the estimator never reads anyway).
    assert pes[0]["pose"] == [1.75, 1.75, 0.0]
    assert "odom" not in events[0] and events[2]["odom"] == [0.25, 0.0, 0.0]

    # Every composed pose pinned to the exact double (the twists ride inside them).
    pins = [
        (1.75, 1.75, 0.0),
        (1.9938614253006162, 1.7485465123669501, 0.0019442557789731546),
        (2.2148091162733996, 1.8312591245115464, -0.05973331507934694),
        (2.450363965729009, 1.789030019919816, -0.041422709877410674),
        (2.707779875481148, 1.7391449950498663, -0.010721669321791652),
    ]
    for k in range(5):
        assert [pes[k]["pose"][i] for i in range(3)] == list(pins[k])

    total = 0.0
    for est_pose, step in zip(result.poses, episode.steps, strict=True):
        dx, dy = est_pose.x - step.gt.x, est_pose.y - step.gt.y
        total += dx * dx + dy * dy
    ate = math.sqrt(total / len(result.poses))
    assert ate == 0.05247606452360631  # pinned — the staircase demo's exact ATE
    total_r = 0.0
    for t in range(1, len(result.poses)):
        e_hat = pose_minus(result.poses[t - 1], result.poses[t])
        e_gt = pose_minus(episode.steps[t - 1].gt, episode.steps[t].gt)
        dx, dy, dt = e_hat.dx - e_gt.dx, e_hat.dy - e_gt.dy, e_hat.dtheta - e_gt.dtheta
        total_r += dx * dx + dy * dy + dt * dt
    assert math.sqrt(total_r / (len(result.poses) - 1)) == 0.06183302900652463

    # The estimator never reads Step.odom: a σ that would swamp any odometry model
    # changes NOTHING here, bit for bit.
    noisy = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        5.0, 5.0, sc.seed,
    )
    result_noisy = Ndt(params).run(noisy, None)
    assert [(p.x, p.y, p.theta) for p in result_noisy.poses] == \
           [(p.x, p.y, p.theta) for p in result.poses]
