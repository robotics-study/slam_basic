"""dbscan — Density-Based Spatial Clustering of Applications with Noise (Ester,
Kriegel, Sander & Xu, KDD 1996), the features branch's first member and the answer
to where this repo's landmark-based branches (filter_based: ekf_slam, fastslam) get
their (id, bearing, range) observations from. They CONSUME association as given;
this is the page that shows an id being BORN: raw scan points density-clustered
into stable candidates, each candidate's id its discovery order, its position the
mean of its members.

WHAT THIS BRANCH MEASURES — and what it does not. dbscan measures NO motion. The
pose that lifts robot-frame scan points into world frame is the declared gauge pose
composed with the scenario's odometry (σ_xy = σ_θ = 0 in this scenario, so the
integrated pose equals ground truth bit-for-bit) — exactly grid_mapping's "pose is
given" convention. ATE/RPE read ~0 by construction; the only real metric here is
landmark_rmse over ids present on both sides. That IS the lesson: feature-based
SLAM stands on odometry it does not question, and this page shows what sits under
the assumption — association without an association algorithm.

THE ALGORITHM (Ester's Algorithm 1, fixed operation order — C++ and TS mirror it
bit-for-bit; every comparison below is a squared-distance compare dx²+dy² ≤ eps²,
no sqrt anywhere): points are visited in ASCENDING INDEX (the accumulation order:
step-major, beam-minor). An unvisited point is core iff its closed eps-neighborhood
holds at least min_pts points INCLUDING ITSELF; a non-core visit labels the point
NOISE and noise is sticky — a point visited while sparse stays noise forever even
when a later cluster grows within eps of it (on static data, re-querying a noise
point returns the same sub-threshold neighborhood, so this equals the paper's
pseudocode exactly). A core seed opens a new cluster with the next id and expands:
a growing list walked by index pointer; every unvisited point reached joins the
cluster, but only a reached point whose OWN neighborhood also reaches min_pts (a
core point) extends the frontier — border points join without expanding. Cluster
id = seed order = ascending index of each cluster's first core point, which is why
ids stay identical across languages AND stable across steps: earlier-indexed seeds
keep their ids as points accumulate.

WHY ACCUMULATED RE-CLUSTERING (not per-scan): the accumulated point list grows in
fixed order (step-major, beam-minor), and clustering re-runs over ALL of it every
step — a cluster seeded by an early index keeps its id forever; a blob whose cloud
is too sparse to core yet has NO id at all until density crosses min_pts, and when
it cores, its seed's index decides where the new id lands. That is what "stable
candidates" honestly means: not a tracker, just order.

THE CENTROID (the landmark): mean of the cluster's members in their append order —
explicit += loops, NOT the builtin sum() (from CPython 3.12, gh-100425, that is
Neumaier compensated summation — a different arithmetic expression than the plain
left-to-right sums the C++ mirror runs). sx/sy are emitted as None: this estimator
carries no uncertainty model; that is what filter_based adds on top.
"""

from __future__ import annotations

import math

from ..core.estimator import Capability, Estimator
from ..core.geometry import pose_compose, robot_to_world
from ..core.params import ParamSet
from ..core.trace import TraceRecorder
from ..core.types import EstimateResult, LandmarkEstimate, Point, Pose, Step

# Label sentinels — the paper's UNVISITED / NOISE labels as ints (cluster ids are k ≥ 0).
UNVISITED = -2
NOISE = -1


def _neighborhood(points: tuple[Point, ...], i: int, eps_sq: float) -> list[int]:
    """Indices j (ASCENDING, self included) with dx² + dy² ≤ eps² — the closed ball,
    compared on squared distance so no sqrt exists anywhere in this module. Self is
    always inside (0 ≤ eps²), which is what makes min_pts count itself."""
    xi, yi = points[i]
    nb: list[int] = []
    for j in range(len(points)):
        xj, yj = points[j]
        dx = xj - xi
        dy = yj - yi
        if dx * dx + dy * dy <= eps_sq:
            nb.append(j)
    return nb


def dbscan_cluster(
    points: tuple[Point, ...], eps: float, min_pts: int
) -> list[list[Point]]:
    """Ester Algorithm 1 over `points` in index order (module docstring for the
    full fixed-order contract). Returns clusters in DISCOVERY order; each cluster's
    members sit in append order (seed first). Points labeled NOISE appear nowhere."""
    n = len(points)
    labels = [UNVISITED] * n
    eps_sq = eps * eps
    clusters: list[list[Point]] = []
    for i in range(n):  # ascending visit order — fixed
        if labels[i] != UNVISITED:
            continue  # already a member or already sticky noise
        nb = _neighborhood(points, i, eps_sq)
        if len(nb) < min_pts:
            labels[i] = NOISE  # sparse at visit time → noise forever (sticky)
            continue
        k = len(clusters)  # discovery order IS the id
        members = [points[i]]
        clusters.append(members)
        labels[i] = k
        j = 0
        while j < len(nb):  # ExpandCluster: growing list, index pointer — fixed
            q = nb[j]
            j += 1
            if labels[q] != UNVISITED:
                continue  # member already (possibly twice in the list) or sticky noise
            qnb = _neighborhood(points, q, eps_sq)
            if len(qnb) >= min_pts:  # q is core → its neighborhood extends the frontier
                for m in qnb:  # ascending append; labels filter at processing time
                    if labels[m] == UNVISITED:
                        nb.append(m)
            labels[q] = k
            members.append(points[q])
    return clusters


class Dbscan(Estimator):
    """Density clustering of accumulated scan points → landmark candidates (the
    features branch: where the landmark-based branches' ids come from)."""

    def __init__(self, params: ParamSet) -> None:
        super().__init__(params)
        self._x0 = params.get_float("x0")
        self._y0 = params.get_float("y0")
        self._theta_deg = params.get_float("theta_deg")
        self._eps = params.get_float("eps")
        self._min_pts = params.get_int("min_pts")
        self._pose: Pose | None = None  # gauge at t = 0, then composed with odom
        self._points: list[Point] = []  # accumulated world points — step-major, beam-minor
        self._poses: list[Pose] = []
        self._landmarks: tuple[LandmarkEstimate, ...] = ()

    @property
    def name(self) -> str:
        return "dbscan"

    def required_capabilities(self) -> set[Capability]:
        return {Capability.BEAM}

    # --- update -------------------------------------------------------------

    def update(self, step: Step, recorder: TraceRecorder | None) -> None:
        """Integrate odometry onto the declared gauge (this branch measures NO
        motion), lift this scan's points into world frame in beam order, append them
        to the accumulation, re-cluster ALL accumulated points, and emit this step's
        events (pose first, landmarks second — same event order as the other
        branches). A step without a scan accumulates nothing and re-emits."""
        if not self._poses:
            pose = Pose(self._x0, self._y0, math.radians(self._theta_deg))
        else:
            assert step.odom is not None  # contract: t ≥ 1 always carries the command
            pose = pose_compose(self._poses[-1], step.odom)
        self._poses.append(pose)

        if step.scan is not None:
            for z in step.scan:  # beam ascending — fixed accumulation order
                self._points.append(robot_to_world(z, pose))
        clusters = dbscan_cluster(tuple(self._points), self._eps, self._min_pts)

        estimates: list[LandmarkEstimate] = []
        for k, members in enumerate(clusters):  # discovery order IS the id
            mx = 0.0
            my = 0.0
            n = float(len(members))
            for (x, y) in members:  # append order — fixed; NOT the builtin sum()
                mx += x
                my += y
            estimates.append(LandmarkEstimate(id=k, x=mx / n, y=my / n))
        self._landmarks = tuple(estimates)

        if recorder is not None:
            recorder.pose_estimated(step.t, pose)
            recorder.landmarks_updated(step.t, self._landmarks)

    def finalize(self, recorder: TraceRecorder | None) -> EstimateResult:
        return EstimateResult(poses=tuple(self._poses), landmarks=self._landmarks)
