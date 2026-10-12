"""dbscan tests — density clustering, pinned bit-for-bit.

The unit tests pin what the algorithm IS: a CLOSED ball (a point exactly eps away
is a neighbor), min_pts counting itself, noise that is sticky by visit order (a
point visited while sparse stays noise even when it sits inside eps of a cluster
seeded later — on static data this equals the paper's pseudocode exactly), and ids
that follow DISCOVERY order (ascending seed index), not spatial order.

The scenario run pins the branch story end-to-end on blobs01_line: three single-cell
blobs become three clusters whose ids are the GT list order, every estimated pose is
ground truth EXACTLY (this branch measures no motion — σ = 0 odometry composed onto
the declared gauge lands bit-for-bit on gt, so ATE/RPE are exactly 0.0), and unlike
the registration branch this estimator DOES consume Step.odom: rebuilding the episode
with σ_xy = σ_θ = 5.0 shatters the world-frame accumulation into 33 scattered clusters
and moves every pinned number (pinned below, honestly). The residual landmark error is
the half-cell itself: a beam stops at a blob's face, so the honest feature centroid
sits on the face (B's estimate lands at x = 5.9999… against the cell CENTER 6.25), and
landmark_rmse ≈ 0.2389 is that half-cell bias plus σ_range noise — measured, pinned.
"""

import io
import json

from conftest import SCENARIO_DIR, config

from slam.core.metrics import evaluate
from slam.core.params import ParamError, ParamSet
from slam.core.sim import build_episode
from slam.core.trace import TraceRecorder
from slam.features.dbscan import Dbscan, dbscan_cluster
from slam.maps.loader import load_scenario


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(config("dbscan"))
    assert params.algorithm == "dbscan" and params.section == "features"
    assert params.scenarios == ["blobs01_line"]
    # The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
    assert params.get_float("x0") == 0.75 and params.get_float("y0") == 1.75
    assert params.get_float("theta_deg") == 0.0
    assert params.get_float("eps") == 0.2
    assert params.get_int("min_pts") == 4
    # Out-of-range sets raise (no silent clamping — the contract is a range check).
    for key, bad in (("eps", 0.04), ("eps", 2.5), ("min_pts", 0)):
        try:
            params.set(key, bad)
            raise AssertionError(f"expected ParamError for {key} = {bad!r}")
        except ParamError:
            pass


def test_closed_ball_and_self_counting() -> None:
    """A point exactly eps away IS a neighbor (dx² + dy² ≤ eps² — closed ball, no
    sqrt anywhere) and the neighborhood counts its center itself. Four collinear
    points one unit apart at eps = 1.0: p0's neighborhood is {p0, p1} — two points,
    below min_pts 3 — so p0 is noise; p1 sees {p0, p1, p2} (distance EXACTLY 1.0
    counts) and seeds the only cluster."""
    pts = ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0))
    assert dbscan_cluster(pts, 1.0, 3) == [[(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]]
    # One point below the threshold: no core anywhere, so NO cluster at all —
    # empty input and sub-threshold input both yield the empty list.
    assert dbscan_cluster(((0.0, 0.0), (1.0, 0.0)), 1.0, 3) == []
    assert dbscan_cluster((), 1.0, 3) == []


def test_noise_is_sticky_by_visit_order() -> None:
    """The algorithm's own arbitrariness, pinned. Same four points, reordered: as
    ((5,0),(0,0),(1,0),(2,0}) the point (0,0) is visited while still sparse (its
    neighborhood holds 2 < 3), so it is labeled NOISE — and it stays noise even
    though it sits exactly eps from the later cluster's seed. In the first ordering
    (0,0) was itself a member; order is the only difference."""
    assert dbscan_cluster(((5.0, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0)), 1.0, 3) == [
        [(1.0, 0.0), (2.0, 0.0)]
    ]


def test_ids_follow_discovery_order_not_space() -> None:
    """Cluster ids are the ascending index of each cluster's first core point — not
    any spatial order. The RIGHT-hand triple comes first in the input list, so it
    earns id 0 and the left-hand pair (whose own seed is visited later) earns id 1;
    each cluster's members keep their append order (seed first)."""
    pts = ((5.0, 0.0), (6.0, 0.0), (7.0, 0.0), (0.0, 0.0), (0.0, 1.0), (0.0, 2.0))
    got = dbscan_cluster(pts, 1.0, 3)
    assert got == [[(6.0, 0.0), (7.0, 0.0)], [(0.0, 1.0), (0.0, 2.0)]]


def test_run_on_scenario_landmarks_come_from_points() -> None:
    """The branch story end-to-end on blobs01_line with the config defaults (no
    sensor params declared — the demo injects NOTHING): t = 0 emits the declared
    gauge pose exactly and already resolves three clusters; every later step composes
    the arriving EXACT twist onto that pose, so every estimated pose equals ground
    truth bit-for-bit and ATE/RPE are exactly 0.0. The final landmarks sit on the
    blobs' faces (B at x = 5.9999… against cell center 6.25) — a beam stops at the
    face, and that half-cell is what landmark_rmse honestly measures."""
    sc = _scenario()
    params = ParamSet.from_yaml(config("dbscan"))

    episode = _episode(sc)
    buf = io.StringIO()
    est = Dbscan(params)
    result = est.run(episode, TraceRecorder(buf))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    names = [e["event"] for e in events]
    assert names == ["step_observed", "pose_estimated", "landmarks_updated"] * 11

    # t = 0: the declared gauge, emitted verbatim — and three clusters already.
    pes = [e for e in events if e["event"] == "pose_estimated"]
    les = [e for e in events if e["event"] == "landmarks_updated"]
    assert pes[0]["pose"] == [0.75, 1.75, 0.0]
    assert "cov" not in pes[0] and "sx" not in les[0]["estimated"][0]
    assert all([e["id"] for e in le["estimated"]] == [0, 1, 2] for le in les)
    t0 = [(e["x"], e["y"]) for e in les[0]["estimated"]]
    assert t0 == [
        (4.493887564731294, 1.2896867167788368),   # A — face x ≈ 4.5, center is 4.75
        (6.005154080104347, 1.795749986472559),     # B — dead-ahead face at x = 6.0
        (4.50441122289002, 2.2452803504816274),     # C — mirror of A across y = 1.75
    ]

    # Final estimates: the accumulated clouds densify and the centroids converge onto
    # the faces; every pinned double is bit-exact in both languages.
    assert [(e.x, e.y) for e in result.landmarks] == [
        (4.518062212080072, 1.2759425774767463),
        (5.9999255335068975, 1.7501295266874315),
        (4.518751688800652, 2.224051537406564),
    ]

    # Pose = declared gauge ⊕ exact odometry — bit-for-bit ground truth, so both pose
    # metrics are EXACTLY zero and the only real number is landmark_rmse.
    assert all(p.x == s.gt.x and p.y == s.gt.y and p.theta == s.gt.theta
               for p, s in zip(result.poses, episode.steps, strict=True))
    metrics = evaluate(result, episode)
    assert metrics["ate_rmse"] == 0.0 and metrics["rpe_rmse"] == 0.0
    assert metrics["landmark_rmse"] == 0.23885456837708618


def test_run_consumes_odometry_unlike_registration() -> None:
    """The branch contrast, pinned: registration NEVER reads Step.odom; this branch
    lives on it. Rebuilding the same episode with σ_xy = σ_θ = 5.0 shatters the
    world-frame accumulation — 33 scattered clusters instead of three, ATE 12.878…,
    and even the surviving first three landmarks move. Every pinned number below is a
    measured consequence of consuming noisy odometry."""
    sc = _scenario()
    params = ParamSet.from_yaml(config("dbscan"))
    episode = _episode(sc, sigma_xy=5.0, sigma_theta=5.0)
    result = Dbscan(params).run(episode, None)
    assert len(result.landmarks) == 33
    metrics = evaluate(result, episode)
    assert metrics["ate_rmse"] == 12.878164352825113
    assert metrics["rpe_rmse"] == 6.941842126318921
    assert metrics["landmark_rmse"] == 0.2513611765791011


def _scenario():
    return load_scenario(SCENARIO_DIR / "blobs01_line.yaml")


def _episode(sc, sigma_xy: float | None = None, sigma_theta: float | None = None):
    return build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, sc.landmarks,
        sc.sigma_xy if sigma_xy is None else sigma_xy,
        sc.sigma_theta if sigma_theta is None else sigma_theta,
        sc.seed,
    )
