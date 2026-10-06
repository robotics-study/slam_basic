#pragma once

#include "slam/core/types.hpp"

namespace slam::core {

// A closed cell square (x_lo, y_lo, x_hi, y_hi) in world coordinates.
struct Rect {
  double x_lo = 0.0;
  double y_lo = 0.0;
  double x_hi = 0.0;
  double y_hi = 0.0;
};

// Exact floating-point pose / point geometry shared by simulator and algorithms.
// Every function is a fixed IEEE-754 double expression: identical operation order
// in Python and C++ means bit-identical results (the repo's parse-equality contract
// covers the wire; these make the DECISIONS equal). The formulas are copied verbatim
// from spec/data_formats.md — do not "simplify" one without updating the spec.
// sin/cos here route through core/libm.hpp (dlsym-resolved scalar libSystem sin/
// cos): Apple clang folds a same-argument (sin, cos) pair into __sincos_stret,
// whose SIMD result differs from Python's math.sin/math.cos by 1 ulp on rare
// inputs — routing through function pointers makes the fold impossible.

// Fold an angle to [-pi, pi): a - 2*pi*floor((a + pi) / (2*pi)). The FORMULA is the
// contract; at a = pi exactly it yields -pi (reachable range is [-pi, pi)).
double wrap(double a);

// p (+) t — apply a robot-frame twist to a world pose (fixed operation order).
Pose pose_compose(const Pose& p, const Twist& t);

// a^-1 (+) b — the robot-frame twist that carries a onto b (the exact inverse
// composition the simulator differentiates ground truth with).
Twist pose_minus(const Pose& a, const Pose& b);

// World point e into p's robot frame: z = R(-theta)(e - p).
Point world_to_robot(const Point& e, const Pose& p);

// Robot-frame point z into world frame: e = R(theta) z + (p.x, p.y). The exact
// inverse of world_to_robot — algorithms reconstruct scan endpoints from their OWN
// estimated pose with this.
Point robot_to_world(const Point& z, const Pose& p);

// Closed-segment intersection test (touching counts). Exact float sign tests;
// collinear overlap falls through to the on-segment cases.
bool segments_intersect(const Point& a, const Point& b, const Point& c, const Point& d);

// Closed-rectangle containment on the fixed rect (x_lo, y_lo, x_hi, y_hi).
bool point_in_rect(const Point& p, const Rect& rect);

// True iff the CLOSED segment a->b meets the CLOSED cell square (an endpoint inside
// counts). Landmark visibility is "no occupied cell intersects the segment".
bool segment_intersects_rect(const Point& a, const Point& b, const Rect& rect);

}  // namespace slam::core
