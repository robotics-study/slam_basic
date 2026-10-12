"""ekf_slam tests — augmented-state EKF-SLAM, pinned bit-for-bit.

The unit tests pin what the algorithm IS: birth is augmentation NOT an update (the
mean is the exact back-projection so the innovation is exactly 0 by construction, and
at t = 0 the pose covariance is still the ZERO matrix — every cross block born with
it is ±0 exactly); landmarks enter in OBSERVATION order (ids [0,2,3,4,5] at t = 0,
id 1 late at t = 3 as SLOT 5 — slot ≠ id is part of the page); and Σ stays exactly
symmetric because every product computes its upper triangle and COPIES it.

The scenario run pins the branch story end-to-end on office01_landmarks: with Q > 0
the landmarks pull the pose back through the correlation blocks (ATE ≈ 0.041 against
a raw odometry drift far larger), and σ_xy = σ_θ = 0 is the honest limit — Σ_pose
stays EXACTLY zero forever, so every estimated pose equals ground truth BIT-FOR-BIT
(noise-free odometry composed through pose_compose) while landmarks alone still chase
the observation noise floor. Everything below is deterministic: seed 42 pins these
numbers bit-for-bit in both languages.
"""

import io
import json

from conftest import SCENARIO_DIR, config

from slam.core.metrics import evaluate
from slam.core.params import ParamError, ParamSet
from slam.core.sim import build_episode
from slam.core.trace import TraceRecorder
from slam.filter_based.ekf_slam import EkfSlam
from slam.maps.loader import load_scenario


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(config("ekf_slam"))
    assert params.algorithm == "ekf_slam" and params.section == "filter_based"
    assert params.scenarios == ["office01_landmarks"]
    # The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
    assert params.get_float("x0") == 0.875 and params.get_float("y0") == 4.625
    assert params.get_float("theta_deg") == 0.0
    # The declared noise IS the scenario's injected noise (declared, not injected).
    assert params.get_float("sigma_xy") == 0.05 and params.get_float("sigma_theta") == 0.035
    assert params.get_float("sigma_range") == 0.05 and params.get_float("sigma_bearing") == 0.03
    # Out-of-range sets raise (no silent clamping — the contract is a range check).
    # sigma_range/sigma_bearing keep min > 0: at t = 0 Σ is the zero matrix, so a
    # zero measurement σ would make the FIRST birth's W = HpΣHᵀ + R singular.
    for key, bad in (("sigma_xy", -0.1), ("sigma_xy", 2.0), ("sigma_range", 0.0)):
        try:
            params.set(key, bad)
            raise AssertionError(f"expected ParamError for {key} = {bad!r}")
        except ParamError:
            pass


def test_birth_at_t0_is_backprojection_with_zero_cross_blocks() -> None:
    """t = 0 is the whole lesson in one step. The gauge pose is KNOWN (cov emitted
    as exactly [0.0, 0.0, 0.0]), so five landmarks are born through the improper-
    prior limit with cross-covariance exactly ±0 — and their slot order IS their
    observation order: ids [0,2,3,4,5] at t = 0 (id 1 is not yet visible), then id
    1 arrives late at t = 3 as SLOT 5. The pinned sx/sy are sqrt(diag(B⁻¹RB⁻ᵀ)) —
    pure measurement noise rotated into world frame, no pose contribution."""
    sc = _scenario()
    params = ParamSet.from_yaml(config("ekf_slam"))
    buf = io.StringIO()
    est = EkfSlam(params)
    est.run(_episode(sc), TraceRecorder(buf))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    names = [e["event"] for e in events]
    assert names == ["step_observed", "pose_estimated", "landmarks_updated"] * 25

    pes = [e for e in events if e["event"] == "pose_estimated"]
    les = [e for e in events if e["event"] == "landmarks_updated"]
    # The gauge: emitted verbatim, covariance EXACTLY zero (all three slots).
    assert pes[0]["pose"] == [0.875, 4.625, 0.0]
    assert pes[0]["cov"] == [0.0, 0.0, 0.0]
    # Slot order = observation order; id 1 is not visible yet at t = 0.
    assert [e["id"] for e in les[0]["estimated"]] == [0, 2, 3, 4, 5]
    t0 = [(e["x"], e["y"], e["sx"], e["sy"]) for e in les[0]["estimated"]]
    assert t0 == [
        (1.415536579483537, 6.910960288238713, 0.06953728280091605, 0.051289190971521355),
        (1.3659884636464192, 6.387327925560553, 0.054546225934396564, 0.05036756581465648),
        (2.3878807675776335, 6.319196650218502, 0.060764973146398255, 0.058743659184490656),
        (6.55599436780117, 5.440077389746368, 0.055204091700766025, 0.17057769960323457),
        (6.5969494533206054, 3.600536505067594, 0.0580251914453736, 0.17188451081075434),
    ]
    # id 1 is born at t = 3 — as SLOT 5 (the sixth entry): slot ≠ id, by contract.
    assert [e["id"] for e in les[3]["estimated"]] == [0, 2, 3, 4, 5, 1]


def test_noise_free_odometry_pins_the_pose_bit_for_bit() -> None:
    """The honest limit, pinned: declare Q = 0 (σ_xy = σ_θ = 0) and feed noise-free
    odometry. Σ_pose starts at zero and every product that could move it multiplies
    those zeros — so the pose rows of K stay exactly ±0 and NO observation can ever
    correct the pose: every estimated pose equals ground truth bit-for-bit, and the
    emitted cov stays [0.0, 0.0, 0.0] at EVERY step. The landmarks still see noise
    (σ_range/σ_bearing unchanged), so they alone chase the observation floor —
    landmark_rmse ≈ 0.031 is that floor; ATE and RPE are EXACTLY zero."""
    sc = _scenario()
    params = ParamSet.from_yaml(config("ekf_slam"))
    params.set("sigma_xy", 0.0)
    params.set("sigma_theta", 0.0)
    episode = _episode(sc, sigma_xy=0.0, sigma_theta=0.0)
    buf = io.StringIO()
    result = EkfSlam(params).run(episode, TraceRecorder(buf))
    assert all(p.x == s.gt.x and p.y == s.gt.y and p.theta == s.gt.theta
               for p, s in zip(result.poses, episode.steps, strict=True))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    pes = [e for e in events if e["event"] == "pose_estimated"]
    assert all(e["cov"] == [0.0, 0.0, 0.0] for e in pes)
    metrics = evaluate(result, episode)
    assert metrics["ate_rmse"] == 0.0 and metrics["rpe_rmse"] == 0.0
    assert metrics["landmark_rmse"] == 0.031009034055468968


def test_run_pinned_metrics_and_final_state() -> None:
    """The branch story end-to-end at the declared defaults: five landmarks at t = 0,
    id 1 late at t = 3, ids 7/9 only after the door (t = 20/23) — and once a landmark
    is re-observed its correlation blocks pull the pose back (ATE ≈ 0.041, RPE ≈
    0.053 against σ_xy = 0.05 raw odometry). The final estimates sit in SLOT order
    ([0,2,3,4,5,1,6,8,7,9] — birth order, not id order); id 0's sx shrinks from its
    birth value (pinned above) to 0.0467… through re-observations."""
    sc = _scenario()
    params = ParamSet.from_yaml(config("ekf_slam"))
    episode = _episode(sc)
    result = EkfSlam(params).run(episode, None)
    assert [le.id for le in result.landmarks] == [0, 2, 3, 4, 5, 1, 6, 8, 7, 9]
    assert [(le.x, le.y) for le in result.landmarks] == [
        (1.4200996828569024, 6.886174368712869),
        (1.3803222685724423, 6.359958406045323),
        (2.3619726977007933, 6.368831565718878),
        (6.618930570332339, 5.37503286465251),
        (6.643192860733659, 3.618988650677379),
        (2.36488622326796, 6.8334875911877235),
        (8.415522255648076, 2.8835850551259687),
        (8.384155139907442, 1.931969036455171),
        (10.432688731991588, 2.9304085077584583),
        (10.465177740635248, 1.8892355645801564),
    ]
    assert result.landmarks[0].sx == 0.0467706341056405
    assert result.landmarks[0].sy == 0.035023119339539416
    metrics = evaluate(result, episode)
    assert metrics["ate_rmse"] == 0.040788530613740936
    assert metrics["rpe_rmse"] == 0.05301454702803877
    assert metrics["landmark_rmse"] == 0.04962713080932249


def _scenario():
    return load_scenario(SCENARIO_DIR / "office01_landmarks.yaml")


def _episode(sc, sigma_xy: float | None = None, sigma_theta: float | None = None):
    return build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, sc.landmarks,
        sc.sigma_xy if sigma_xy is None else sigma_xy,
        sc.sigma_theta if sigma_theta is None else sigma_theta,
        sc.seed,
    )
