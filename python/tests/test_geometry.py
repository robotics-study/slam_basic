"""Fixed-formula geometry: wrap / pose compose+minus / frame conversion / segment-rect."""

import math

from slam.core.geometry import (
    point_in_rect,
    pose_compose,
    pose_minus,
    robot_to_world,
    segment_intersects_rect,
    segments_intersect,
    wrap,
)
from slam.core.types import Pose, Twist


def test_wrap_formula_contract() -> None:
    # The FORMULA defines the range: at a = pi it yields -pi (range [-pi, pi)).
    two_pi = 2.0 * math.pi
    assert wrap(math.pi) == math.pi - two_pi * math.floor((math.pi + math.pi) / two_pi)
    assert wrap(math.pi) == -math.pi
    assert wrap(0.0) == 0.0
    assert wrap(-math.pi) == -math.pi
    assert abs(wrap(3.0 * math.pi) - (-math.pi)) < 1e-12 or wrap(3.0 * math.pi) == -math.pi
    assert wrap(math.pi / 2) == math.pi / 2
    # Wrapping is idempotent inside the range.
    for a in (-2.5, -0.3, 0.0, 1.2):
        assert wrap(wrap(a)) == wrap(a)


def test_pose_compose_minus_roundtrip() -> None:
    a = Pose(1.0, -2.0, 0.7)
    u = Twist(0.5, -0.25, 0.3)
    b = pose_compose(a, u)
    # pose_minus inverts composition exactly (float-exact formula pair).
    back = pose_minus(a, b)
    assert back.dx == round(u.dx, 12) or abs(back.dx - u.dx) < 1e-15
    assert abs(back.dy - u.dy) < 1e-15
    assert abs(wrap(back.dtheta - u.dtheta)) < 1e-15


def test_pose_compose_fixed_formula() -> None:
    # Fixed operation order check at theta = pi/2 (cos ~ 0): x' = x + c*dx - s*dy.
    p = Pose(1.0, 2.0, math.pi / 2)
    q = pose_compose(p, Twist(3.0, 4.0, 0.0))
    assert abs(q.x - (1.0 + math.cos(math.pi / 2) * 3.0 - math.sin(math.pi / 2) * 4.0)) < 1e-15
    assert abs(q.y - (2.0 + math.sin(math.pi / 2) * 3.0 + math.cos(math.pi / 2) * 4.0)) < 1e-15


def test_frame_conversion_roundtrip() -> None:
    p = Pose(-1.5, 3.25, -1.1)
    e = (2.0, 0.5)
    z = robot_to_world(e, p)  # world -> robot -> world must round-trip
    from slam.core.geometry import world_to_robot

    back = world_to_robot(z, p)
    assert abs(back[0] - e[0]) < 1e-12 and abs(back[1] - e[1]) < 1e-12


def test_segments_intersect() -> None:
    assert segments_intersect((0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0))
    # Touching at an endpoint counts (closed segments).
    assert segments_intersect((0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 1.0))
    # Collinear overlap counts.
    assert segments_intersect((0.0, 0.0), (2.0, 0.0), (1.0, 0.0), (3.0, 0.0))
    # Parallel disjoint does not.
    assert not segments_intersect((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0))


def test_point_in_rect_and_segment_rect() -> None:
    rect = (0.0, 0.0, 1.0, 1.0)
    assert point_in_rect((0.5, 0.5), rect)
    assert point_in_rect((1.0, 1.0), rect)  # closed
    assert not point_in_rect((1.0 + 1e-9, 2.0), rect)
    # Segment crossing the square / touching its corner / missing it.
    assert segment_intersects_rect((-1.0, 0.5), (2.0, 0.5), rect)
    assert segment_intersects_rect((2.0, 2.0), (1.0, 1.0), rect)  # touches corner
    assert not segment_intersects_rect((2.0, 0.0), (2.0, 1.0), rect)
