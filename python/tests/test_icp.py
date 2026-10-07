"""icp tests — point-to-point ICP, pinned bit-for-bit.

The unit tests pin what the scenario run rests on: a single kept pair reproduces its
displacement EXACTLY (the closed form is exact for translation); an equidistant
source point keeps the LOWER target index (strict `<` scan); truncation is the whole
story — one outlier pair beyond d_max silently drags the least-squares mean toward
garbage, and a source whose every pair exceeds d_max returns the identity twist
exactly (no information, no motion; so does an empty scan on either side). The
rotation case pins the closed form to the exact doubles of −R(θ)a / θ constructed
from the same formulas, and iteration is pinned by max_iters=1 vs convergence.

The scenario run pins the branch story end-to-end on room01_straight: t = 0 emits
the DECLARED gauge pose exactly (1.25, 1.75, 0) — registration measures relative
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
from slam.registration.icp import Icp, icp_step


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(config("icp"))
    assert params.algorithm == "icp" and params.section == "registration"
    assert params.scenarios == ["room01_straight"]
    # The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
    assert params.get_float("x0") == 1.25 and params.get_float("y0") == 1.75
    assert params.get_float("theta_deg") == 0.0
    assert params.get_float("d_max") == 1.0 and params.get_float("eps") == 1e-9
    assert params.get_int("max_iters") == 64
    # Out-of-range sets raise (no silent clamping — the contract is a range check).
    for key, bad in (("d_max", 0.2), ("eps", 0.2), ("max_iters", 0)):
        try:
            params.set(key, bad)
            raise AssertionError(f"expected ParamError for {key} = {bad!r}")
        except ParamError:
            pass


def test_single_pair_translation_is_exact() -> None:
    """One kept pair whose displacement is inside d_max: the closed form returns that
    displacement EXACTLY (centroids of singletons, atan2(0, ·) = 0 exactly), and one
    iteration already sits at the fixed point — max_iters=1 changes nothing."""
    got = icp_step(((1.0, 0.5),), ((1.5, 0.5),), 1.0, 1e-9, 64)
    assert got == Twist(dx=0.5, dy=0.0, dtheta=0.0)
    assert icp_step(((1.0, 0.5),), ((1.5, 0.5),), 1.0, 1e-9, 1) == got


def test_tie_keeps_the_lower_index() -> None:
    """Source (1, 0) sits EXACTLY between targets (0, 0) and (2, 0): squared distance
    1.0 to both, and the ascending strict-`<` scan keeps index 0 — so the recovered
    twist is −x, not +x. A `<=` would have flipped this result."""
    got = icp_step(((1.0, 0.0),), ((0.0, 0.0), (2.0, 0.0)), 1.0, 1e-9, 64)
    assert got == Twist(dx=-1.0, dy=0.0, dtheta=0.0)


def test_truncation_is_the_whole_story() -> None:
    """The outlier demo: source {(1,.5),(9,.5)} against target {(1.25,.5),(2,.5)}.
    With d_max = 1 the far pair (squared distance 49 > 1) is truncated and the solve
    returns the kept pair's displacement EXACTLY — +0.25. Widen d_max to 20 and the
    wrong pair survives: both pairs are kept, and the least-squares mean of the two
    contradictory displacements lands at −3.375 — silently, confidently wrong. This
    is why truncation exists."""
    src = ((1.0, 0.5), (9.0, 0.5))
    tgt = ((1.25, 0.5), (2.0, 0.5))
    assert icp_step(src, tgt, 1.0, 1e-9, 64) == Twist(dx=0.25, dy=0.0, dtheta=0.0)
    assert icp_step(src, tgt, 20.0, 1e-9, 64) == Twist(dx=-3.375, dy=0.0, dtheta=0.0)


def test_degenerate_inputs_are_identity() -> None:
    """Empty source or target, and a step whose every pair exceeds d_max, all return
    the identity twist exactly — no information, no motion (documented degenerate)."""
    assert icp_step((), ((1.0, 1.0),), 1.0, 1e-9, 64) == Twist(dx=0.0, dy=0.0, dtheta=0.0)
    assert icp_step(((5.0, 5.0),), (), 1.0, 1e-9, 64) == Twist(dx=0.0, dy=0.0, dtheta=0.0)
    assert icp_step(((5.0, 5.0),), ((0.0, 0.0),), 1.0, 1e-9, 64) == \
        Twist(dx=0.0, dy=0.0, dtheta=0.0)


def test_rotation_recovers_and_converges() -> None:
    """Two points rotated by θ = 0.1 (constructed with the same cos/sin doubles the
    implementation uses): the closed form returns the constructing twist — dθ lands on
    0.1 to the last bit and the translation on −R(θ)a exactly. The displacement is
    small against the point spacing, so identity already pairs correctly and ONE
    iteration sits at the fixed point (max_iters=1 pins equal to convergence)."""
    th = 0.1
    c, s = math.cos(th), math.sin(th)
    p1, p2 = (0.0, -5.0), (0.0, 5.0)
    a = (0.25, -0.75)
    q1 = (c * p1[0] + s * p1[1] + a[0], -s * p1[0] + c * p1[1] + a[1])
    q2 = (c * p2[0] + s * p2[1] + a[0], -s * p2[0] + c * p2[1] + a[1])
    got = icp_step((q1, q2), (p1, p2), 5.0, 1e-9, 64)
    assert got.dx == -0.32362610380462753 and got.dy == 0.7212947697968124
    assert got.dtheta == 0.09999999999999999
    # Correct pairing from the first iteration: one pass already IS the fixed point.
    assert icp_step((q1, q2), (p1, p2), 5.0, 1e-9, 1) == got


def test_capture_basin_is_a_fixed_point_too() -> None:
    """The honest limit: points closer than the displacement alias under identity —
    both source points pair to the SAME target point and the wrong fixed point wins.
    Displacement +0.6 on spacing 1.0 converges (exactly, pinned) to +0.1 = −(−0.6+0.4)/2
    — the mean of the two aliased displacements. ICP is a local method; d_max and the
    capture basin are its honest limits, not implementation bugs."""
    got = icp_step(((-0.6, 0.0), (0.4, 0.0)), ((0.0, 0.0), (1.0, 0.0)), 5.0, 1e-9, 64)
    assert got.dx == 0.09999999999999998 and got.dy == 0.0 and got.dtheta == 0.0


def test_update_emits_gauge_then_poses_without_cov() -> None:
    """t = 0 adopts the declared gauge pose and emits it WITHOUT cov (ICP carries no
    uncertainty model); a scan-less step carries the estimate forward untouched."""
    params = ParamSet.from_yaml(config("icp"))
    est = Icp(params)
    buf = io.StringIO()
    with TraceRecorder(buf) as rec:
        est.update(Step(t=0, gt=Pose(0.0, 0.0, 0.0), odom=None, scan=None, obs=None), rec)
        est.update(Step(t=1, gt=Pose(9.0, 9.0, 0.0), odom=Twist(0.25, 0.0, 0.0),
                        scan=None, obs=None), rec)
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["pose_estimated", "pose_estimated"]
    assert events[0]["t"] == 0 and events[1]["t"] == 1
    assert events[0]["pose"] == [1.25, 1.75, 0.0]  # the declared gauge, exactly
    assert events[1]["pose"] == [1.25, 1.75, 0.0]   # scan-less step: identity carried
    assert "cov" not in events[0] and "cov" not in events[1]


def test_run_on_scenario_is_registration_not_odometry() -> None:
    """The branch story end-to-end on room01_straight (the demo's exact inputs): the
    gauge pose is emitted verbatim at t = 0, every later step composes one recovered
    twist, and Step.odom NEVER matters — rebuilding the episode with σ_xy = 5.0
    changes not one bit of any estimate. ATE/RPE pinned to exact doubles (seed 42)."""
    sc = load_scenario(SCENARIO_DIR / "room01_straight.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(config("icp"))  # no sensor params declared — none injected

    buf = io.StringIO()
    est = Icp(params)
    result = est.run(episode, TraceRecorder(buf))
    assert len(result.poses) == len(episode.steps) == 9
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    names = [e["event"] for e in events]
    assert names == ["step_observed", "pose_estimated"] * 9
    pes = [e for e in events if e["event"] == "pose_estimated"]
    assert all("cov" not in e for e in pes)
    # t = 0 is the declared gauge, bit-for-bit; odom arrives at t ≥ 1 (σ = 0 here).
    assert pes[0]["pose"] == [1.25, 1.75, 0.0]
    assert "odom" not in events[0] and events[2]["odom"] == [0.25, 0.0, 0.0]

    total = 0.0
    for est_pose, step in zip(result.poses, episode.steps, strict=True):
        dx, dy = est_pose.x - step.gt.x, est_pose.y - step.gt.y
        total += dx * dx + dy * dy
    ate = math.sqrt(total / len(result.poses))
    assert ate == 0.020964123524742473  # pinned — sub-cell by construction (res 0.5)
    total_r = 0.0
    for t in range(1, len(result.poses)):
        e_hat = pose_minus(result.poses[t - 1], result.poses[t])
        e_gt = pose_minus(episode.steps[t - 1].gt, episode.steps[t].gt)
        dx, dy, dt = e_hat.dx - e_gt.dx, e_hat.dy - e_gt.dy, e_hat.dtheta - e_gt.dtheta
        total_r += dx * dx + dy * dy + dt * dt
    assert math.sqrt(total_r / (len(result.poses) - 1)) == 0.01952285461299684

    # The estimator never reads Step.odom: a σ that would swamp any odometry model
    # changes NOTHING here, bit for bit.
    noisy = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        5.0, 5.0, sc.seed,
    )
    result_noisy = Icp(params).run(noisy, None)
    assert [ (p.x, p.y, p.theta) for p in result_noisy.poses ] == \
           [(p.x, p.y, p.theta) for p in result.poses]
