"""The simulator — GT trajectory resampling, odometry noise, DDA raycast, landmark
observations. Part of the CONTRACT (core), not an algorithm: every beam scan and
landmark observation an estimator ever sees is produced here, bit-identically in
both languages, from the scenario's waypoints + seed (spec/data_formats.md).

Draw order (determinism contract): at each step t draw the OBSERVATION noise first
(beam: beams 0..beams-1 in order; landmarks: id ascending, range then bearing —
invisible landmarks consume no draws), THEN the odometry noise (ex, ey, etheta) for
the NEXT move. Step 0 has no arriving command so it carries no odom field and the
last step draws no odometry.

Beam raycast = Amanatides & Woo grid DDA on the fixed formulas below; a hit is the
distance to the first occupied cell's closed boundary, strictly below range_max,
else miss (no point). Landmark visibility: the segment GT-pose -> landmark must not
intersect any occupied cell square (geometry.segment_intersects_rect).
"""

from __future__ import annotations

import math

from .geometry import pose_minus, segment_intersects_rect, world_to_robot, wrap
from .rng import Rng
from .types import (
    Episode,
    LandmarkObs,
    Point,
    Pose,
    ScanGrid,
    SensorConfig,
    Step,
    Twist,
)


def resample(path: list[Point], step_meters: float) -> list[Pose]:
    """Equal-arc resampling of the waypoint polyline (contract-exact).

    Points sit at arc lengths 0, s, 2s, ... while k*s < L (strictly less — a final
    point exactly on L is not duplicated), plus one last point at the polyline end
    when leftover length remains. Heading theta_k = atan2 of the segment from p_k to
    p_{k+1} (consecutive RESAMPLED points, so a corner's whole rotation lands in one
    step's dtheta); the final point keeps the previous heading."""
    seg_len: list[float] = []
    total = 0.0
    for k in range(len(path) - 1):
        dx = path[k + 1][0] - path[k][0]
        dy = path[k + 1][1] - path[k][1]
        seg_l = math.sqrt(dx * dx + dy * dy)
        seg_len.append(seg_l)
        total += seg_l

    points: list[Point] = []
    s = 0.0
    while s < total:
        points.append(_point_at_arc(path, seg_len, s))
        s += step_meters
    # The polyline end is always a point (its step may be shorter than the spacing).
    points.append((path[-1][0], path[-1][1]))

    poses: list[Pose] = []
    for k in range(len(points)):
        if k + 1 < len(points):
            theta = math.atan2(
                points[k + 1][1] - points[k][1], points[k + 1][0] - points[k][0]
            )
        else:
            theta = poses[-1].theta if poses else 0.0  # final point keeps the previous heading
        poses.append(Pose(points[k][0], points[k][1], theta))
    return poses


def _point_at_arc(path: list[Point], seg_len: list[float], s: float) -> Point:
    """The polyline point at arc length s (fixed walk: first segment whose remaining
    length covers the remainder, linear interpolation on it)."""
    acc = 0.0
    for k, seg_l in enumerate(seg_len):
        if s - acc <= seg_l:
            frac = (s - acc) / seg_l
            ax, ay = path[k]
            bx, by = path[k + 1]
            return (ax + frac * (bx - ax), ay + frac * (by - ay))
        acc += seg_l
    # s >= total: the polyline end.
    return path[-1]


def raycast(grid: ScanGrid, x: float, y: float, phi: float, range_max: float) -> float | None:
    """DDA ray from (x, y) at angle phi over the grid; returns the distance to the
    first occupied cell's closed boundary (< range_max) or None for a miss.

    Fixed formulas (both languages compute identically): with cdx = cos(phi),
    sdy = sin(phi), the axis step is sign(cdx)/sign(sdy); tMax starts at the ray
    parameter of the next grid-line crossing in that direction and advances by
    res/|cos| (res/|sin|). An axis never steps when its cosine is 0 (tMax = inf).
    A tie (tMaxX == tMaxY) takes the Y step. Starting inside an occupied cell hits
    at distance 0; leaving the grid bounds misses."""
    cdx = math.cos(phi)
    sdy = math.sin(phi)
    r, c = grid.world_to_cell(x, y)
    if not grid.in_bounds(r, c):
        return None
    if grid.occupied(r, c):
        return 0.0
    ox, oy = grid.origin
    res = grid.resolution
    h = grid.height

    inf = float("inf")
    if cdx > 0.0:
        step_x = 1
        t_max_x = (ox + (c + 1) * res - x) / cdx
    elif cdx < 0.0:
        step_x = -1
        t_max_x = (ox + c * res - x) / cdx
    else:
        step_x = 0
        t_max_x = inf
    if sdy > 0.0:
        step_y = -1  # world y grows DOWNWARD in row index
        # Row r's TOP edge is at oy + (h - r) * res (row 0 is the top image row).
        t_max_y = (oy + (h - r) * res - y) / sdy
    elif sdy < 0.0:
        step_y = 1
        # Row r's BOTTOM edge is at oy + (h - 1 - r) * res.
        t_max_y = (oy + (h - 1 - r) * res - y) / sdy
    else:
        step_y = 0
        t_max_y = inf

    delta_x = res / abs(cdx) if cdx != 0.0 else inf
    delta_y = res / abs(sdy) if sdy != 0.0 else inf

    while True:
        if t_max_x < t_max_y:
            hit_t = t_max_x
            if hit_t >= range_max:
                return None
            c += step_x
            t_max_x += delta_x
        else:
            hit_t = t_max_y
            if hit_t >= range_max:
                return None
            r += step_y
            t_max_y += delta_y
        if not grid.in_bounds(r, c):
            return None
        if grid.occupied(r, c):
            return hit_t


def landmark_visible(grid: ScanGrid, from_pt: Point, lm: Point) -> bool:
    """True iff the segment from_pt -> lm crosses NO occupied cell square (the
    contract's visibility predicate; bbox-rejected cells are skipped exactly)."""
    min_x = min(from_pt[0], lm[0])
    max_x = max(from_pt[0], lm[0])
    min_y = min(from_pt[1], lm[1])
    max_y = max(from_pt[1], lm[1])
    ox, oy = grid.origin
    res = grid.resolution
    h = grid.height
    w = grid.width
    for r in range(h):
        y_lo = oy + (h - 1 - r) * res
        y_hi = oy + (h - r) * res
        if y_lo > max_y or y_hi < min_y:
            continue
        for c in range(w):
            x_lo = ox + c * res
            x_hi = ox + (c + 1) * res
            if x_lo > max_x or x_hi < min_x:
                continue
            if grid.occupied(r, c) and segment_intersects_rect(
                from_pt, lm, (x_lo, y_lo, x_hi, y_hi)
            ):
                return False
    return True


def _observe(
    rng: Rng,
    grid: ScanGrid,
    gt: Pose,
    sensor: SensorConfig,
    landmarks: tuple[Point, ...] | None,
) -> tuple[tuple[Point, ...] | None, tuple[LandmarkObs, ...] | None]:
    """One step's observation, drawn in contract order (see module docstring)."""
    if sensor.type == "beam":
        beams = sensor.beams
        assert beams is not None and sensor.fov_deg is not None
        fov = math.radians(sensor.fov_deg)
        half = fov / 2.0
        step_a = fov / (beams - 1)
        scan: list[Point] = []
        for i in range(beams):
            phi = gt.theta - half + float(i) * step_a
            r_hit = raycast(grid, gt.x, gt.y, phi, sensor.range_max)
            if r_hit is None:
                continue  # miss emits no point
            r_noisy = r_hit + rng.gaussian(0.0, sensor.sigma_range)
            e = (gt.x + math.cos(phi) * r_noisy, gt.y + math.sin(phi) * r_noisy)
            scan.append(world_to_robot(e, gt))
        return tuple(scan), None
    # landmarks: id ascending; range noise then bearing noise per visible landmark.
    assert sensor.type == "landmarks" and landmarks is not None
    assert sensor.sigma_bearing is not None
    obs: list[LandmarkObs] = []
    for lm_id, lm in enumerate(landmarks):
        dx = lm[0] - gt.x
        dy = lm[1] - gt.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > sensor.range_max or not landmark_visible(grid, (gt.x, gt.y), lm):
            continue
        r_noisy = dist + rng.gaussian(0.0, sensor.sigma_range)
        bearing_exact = wrap(math.atan2(dy, dx) - gt.theta)
        b_noisy = wrap(bearing_exact + rng.gaussian(0.0, sensor.sigma_bearing))
        obs.append(LandmarkObs(id=lm_id, bearing=b_noisy, range=r_noisy))
    return None, tuple(obs)


def build_episode(
    grid: ScanGrid,
    path: list[Point],
    step_meters: float,
    sensor: SensorConfig,
    landmarks: tuple[Point, ...] | None,
    sigma_xy: float,
    sigma_theta: float,
    seed: int,
) -> Episode:
    """Assemble the full Step stream from a scenario (see maps/loader.Scenario):
    resample GT poses, then per step draw observation noise first and odometry
    noise for the NEXT move second; Step(t).odom is the command that arrived at t."""
    poses = resample(path, step_meters)
    rng = Rng(seed)
    steps: list[Step] = []
    u_arriving: Twist | None = None
    last = len(poses) - 1
    for t in range(len(poses)):
        scan, obs = _observe(rng, grid, poses[t], sensor, landmarks)
        steps.append(Step(t=t, gt=poses[t], odom=u_arriving, scan=scan, obs=obs))
        if t < last:
            u_exact = pose_minus(poses[t], poses[t + 1])
            ex = rng.gaussian(0.0, sigma_xy)
            ey = rng.gaussian(0.0, sigma_xy)
            etheta = rng.gaussian(0.0, sigma_theta)
            u_arriving = Twist(u_exact.dx + ex, u_exact.dy + ey, wrap(u_exact.dtheta + etheta))
    return Episode(steps=tuple(steps), landmarks=landmarks, grid=grid)
