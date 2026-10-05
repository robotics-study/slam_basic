"""Simulator contract tests: resample / DDA raycast / visibility / draw order.

The golden floats below pin the cross-language contract (the C++ tests assert the
same numbers): they were produced by this exact formula chain once and are frozen."""

import math

import numpy as np

from slam.core.sim import build_episode, landmark_visible, raycast, resample
from slam.core.types import SensorConfig
from slam.maps.occupancy_grid import OccupancyGrid2D


def _grid(rows: list[str], resolution: float = 1.0) -> OccupancyGrid2D:
    pixels = np.array([[255 if ch == "." else 0 for ch in row] for row in rows], dtype=np.uint16)
    return OccupancyGrid2D(pixels=pixels, resolution=resolution, origin=(0.0, 0.0))


def test_resample_straight_and_corner() -> None:
    poses = resample([(0.0, 0.0), (2.0, 0.0)], 1.0)
    assert [(p.x, p.y, p.theta) for p in poses] == [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
    ]
    # Corner: heading flips on the step whose segment crosses the corner; the final
    # point keeps the previous heading.
    poses = resample([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 1.0)
    assert len(poses) == 3
    assert poses[0].theta == 0.0
    assert poses[1].theta == math.pi / 2 and poses[2].theta == math.pi / 2
    assert (poses[1].x, poses[1].y) == (1.0, 0.0) and (poses[2].x, poses[2].y) == (1.0, 1.0)


def test_raycast_goldens() -> None:
    # 3x5 grid, res 1, origin 0: only cell (row 1, col 2) is occupied — its rect is
    # x in [2,3], y in [1,2].
    g = _grid([".....", "..#..", "....."])
    assert raycast(g, 0.5, 1.5, 0.0, 10.0) == 1.5  # wall face at x=2 from x=0.5
    assert raycast(g, 0.5, 1.5, math.pi / 2, 10.0) is None  # leaves the map: miss
    assert raycast(g, 0.5, 1.5, 0.0, 1.4) is None  # first crossing beyond range_max
    assert raycast(g, 2.5, 1.5, 0.0, 10.0) == 0.0  # starts inside an occupied cell


def test_landmark_visibility() -> None:
    g = _grid([".....", "..#..", "....."])
    assert not landmark_visible(g, (0.5, 1.5), (2.5, 1.5))  # wall cell in between
    assert landmark_visible(g, (0.5, 0.5), (4.5, 0.5))


def test_episode_draw_order_golden() -> None:
    """Step 0 carries no odom; observation draws come first (range pair then bearing
    pair for the single landmark); the odometry triple follows for the next move."""
    g = _grid(["...", "...", "..."])
    sensor = SensorConfig(type="landmarks", range_max=10.0, sigma_range=0.05, sigma_bearing=0.03)
    ep = build_episode(g, [(0.5, 1.5), (1.5, 1.5)], 1.0, sensor, ((2.5, 1.5),), 0.05, 0.035, 42)

    s0 = ep.steps[0]
    assert s0.t == 0 and s0.odom is None
    assert (s0.gt.x, s0.gt.y, s0.gt.theta) == (0.5, 1.5, 0.0)
    # Golden obs: range = 2 + 0.05*N(0,1)(draws 1-2), bearing = wrap(0 + 0.03*N(0,1)(draws 3-4)).
    assert s0.obs is not None and len(s0.obs) == 1
    assert s0.obs[0].id == 0
    assert s0.obs[0].range == 2.0441124453111135
    assert s0.obs[0].bearing == -0.013525496271565803

    # Determinism: rebuilding from the same seed reproduces every field exactly.
    ep2 = build_episode(g, [(0.5, 1.5), (1.5, 1.5)], 1.0, sensor, ((2.5, 1.5),), 0.05, 0.035, 42)
    assert ep.steps == ep2.steps

    # Step 1: odom = exact twist (dx=1, dtheta=pi/2... here straight so dtheta=0) + noise.
    s1 = ep.steps[1]
    assert s1.odom is not None
    assert abs(s1.odom.dx - 1.0) < 0.5 and abs(s1.odom.dy) < 0.5 and abs(s1.odom.dtheta) < 0.5


def test_episode_beam_scan_is_robot_frame() -> None:
    # Wall at x in [2,3] seen from (0.5,1.5) heading east: the beam fan is centered
    # on theta=0; with sigma_range=0 the hit points are exact.
    g = _grid([".....", "..#..", "....."])
    sensor = SensorConfig(type="beam", range_max=10.0, sigma_range=0.0, beams=3, fov_deg=180.0)
    ep = build_episode(g, [(0.5, 1.5)], 1.0, sensor, None, 0.05, 0.035, 42)
    scan = ep.steps[0].scan
    assert scan is not None
    # beams=3: phi in {-pi/2, 0, +pi/2}. Center beam hits the wall face at dx=1.5;
    # the ±90° beams leave the map (miss) -> exactly one point, robot frame x=1.5.
    assert len(scan) == 1
    assert scan[0][0] == 1.5 and abs(scan[0][1]) < 1e-12
