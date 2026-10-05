"""Exact floating-point pose / point geometry shared by simulator and algorithms.

Every function here is a fixed IEEE-754 double expression: identical operation order
in Python and C++ means bit-identical results, so every boundary case resolves on
identical bits across engines (the repo's parse-equality contract covers the wire;
this module makes the DECISIONS equal). The formulas are copied verbatim from
spec/data_formats.md — do not "simplify" one without updating the spec and C++.
"""

from __future__ import annotations

import math

from .types import Point, Pose, Twist


def wrap(a: float) -> float:
    """Fold an angle to [-pi, pi): a - 2*pi*floor((a + pi) / (2*pi)).

    The FORMULA is the contract (both languages compute it identically), and at
    a = pi exactly it yields -pi — so the reachable range is [-pi, pi)."""
    return a - 2.0 * math.pi * math.floor((a + math.pi) / (2.0 * math.pi))


def pose_compose(p: Pose, t: Twist) -> Pose:
    """p (+) t — apply a robot-frame twist to a world pose (fixed operation order)."""
    c = math.cos(p.theta)
    s = math.sin(p.theta)
    return Pose(
        p.x + c * t.dx - s * t.dy,
        p.y + s * t.dx + c * t.dy,
        wrap(p.theta + t.dtheta),
    )


def pose_minus(a: Pose, b: Pose) -> Twist:
    """a^-1 (+) b — the robot-frame twist that carries a onto b (the exact inverse
    composition the simulator differentiates ground truth with)."""
    c = math.cos(a.theta)
    s = math.sin(a.theta)
    dx = b.x - a.x
    dy = b.y - a.y
    return Twist(c * dx + s * dy, -s * dx + c * dy, wrap(b.theta - a.theta))


def world_to_robot(e: Point, p: Pose) -> Point:
    """World point e into p's robot frame: z = R(-theta)(e - p)."""
    ex = e[0] - p.x
    ey = e[1] - p.y
    c = math.cos(p.theta)
    s = math.sin(p.theta)
    return (c * ex + s * ey, -s * ex + c * ey)


def robot_to_world(z: Point, p: Pose) -> Point:
    """Robot-frame point z into world frame: e = R(theta) z + (p.x, p.y).

    The exact inverse of world_to_robot — algorithms reconstruct scan endpoints
    from their OWN estimated pose with this."""
    c = math.cos(p.theta)
    s = math.sin(p.theta)
    return (p.x + c * z[0] - s * z[1], p.y + s * z[0] + c * z[1])


def _orient(a: Point, b: Point, c: Point) -> float:
    """Cross product (b-a) x (c-a); its sign is the orientation of a->b->c."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, p: Point) -> bool:
    """p is collinear with a->b (caller checked orient == 0): is it on the segment?"""
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])) and (
        min(a[1], b[1]) <= p[1] <= max(a[1], b[1])
    )


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Closed-segment intersection test (touching counts). Exact float sign tests;
    collinear overlap falls through to the on-segment cases."""
    o1 = _orient(a, b, c)
    o2 = _orient(a, b, d)
    o3 = _orient(c, d, a)
    o4 = _orient(c, d, b)
    if o1 == 0.0 and _on_segment(a, b, c):
        return True
    if o2 == 0.0 and _on_segment(a, b, d):
        return True
    if o3 == 0.0 and _on_segment(c, d, a):
        return True
    if o4 == 0.0 and _on_segment(c, d, b):
        return True
    return (o1 > 0.0) != (o2 > 0.0) and (o3 > 0.0) != (o4 > 0.0)


def point_in_rect(p: Point, rect: tuple[float, float, float, float]) -> bool:
    """Closed-rectangle containment test on the fixed rect (x_lo, y_lo, x_hi, y_hi)."""
    x_lo, y_lo, x_hi, y_hi = rect
    return x_lo <= p[0] <= x_hi and y_lo <= p[1] <= y_hi


def segment_intersects_rect(
    a: Point, b: Point, rect: tuple[float, float, float, float]
) -> bool:
    """True iff the CLOSED segment a->b meets the CLOSED cell square (an endpoint
    inside counts). Landmark visibility is defined as "no occupied cell intersects
    the segment", so this predicate IS the visibility test's inner loop."""
    x_lo, y_lo, x_hi, y_hi = rect
    if point_in_rect(a, rect) or point_in_rect(b, rect):
        return True
    c0: Point = (x_lo, y_lo)
    c1: Point = (x_hi, y_lo)
    c2: Point = (x_hi, y_hi)
    c3: Point = (x_lo, y_hi)
    for e0, e1 in ((c0, c1), (c1, c2), (c2, c3), (c3, c0)):
        if segments_intersect(a, b, e0, e1):
            return True
    return False
