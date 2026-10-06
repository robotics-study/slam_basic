// Fixed-formula geometry: wrap / pose compose+minus / frame conversion /
// segment-rect. The formulas are the contract — identical operation order in both
// languages means bit-identical results.

#include <cmath>

#include <gtest/gtest.h>

#include "slam/core/geometry.hpp"
#include "slam/core/types.hpp"

using slam::core::kPi;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Rect;
using slam::core::Twist;
using namespace slam::core;  // free functions: wrap, pose_compose, ...

TEST(Geometry, WrapFormulaContract) {
  // The FORMULA defines the range: at a = pi it yields -pi (range [-pi, pi)).
  double two_pi = 2.0 * kPi;
  EXPECT_DOUBLE_EQ(wrap(kPi), kPi - two_pi * std::floor((kPi + kPi) / two_pi));
  EXPECT_DOUBLE_EQ(wrap(kPi), -kPi);
  EXPECT_DOUBLE_EQ(wrap(0.0), 0.0);
  EXPECT_DOUBLE_EQ(wrap(-kPi), -kPi);
  EXPECT_NEAR(wrap(3.0 * kPi), -kPi, 1e-12);
  EXPECT_DOUBLE_EQ(wrap(kPi / 2), kPi / 2);
  // Wrapping is idempotent inside the range.
  for (double a : {-2.5, -0.3, 0.0, 1.2}) {
    EXPECT_DOUBLE_EQ(wrap(wrap(a)), wrap(a));
  }
}

TEST(Geometry, PoseComposeMinusRoundtrip) {
  Pose a{1.0, -2.0, 0.7};
  Twist u{0.5, -0.25, 0.3};
  Pose b = pose_compose(a, u);
  // pose_minus inverts composition exactly (float-exact formula pair).
  Twist back = pose_minus(a, b);
  EXPECT_NEAR(back.dx - u.dx, 0.0, 1e-15);
  EXPECT_NEAR(back.dy - u.dy, 0.0, 1e-15);
  EXPECT_NEAR(std::abs(wrap(back.dtheta - u.dtheta)), 0.0, 1e-15);
}

TEST(Geometry, PoseComposeFixedFormula) {
  // Fixed operation order check at theta = pi/2 (cos ~ 0): x' = x + c*dx - s*dy.
  Pose p{1.0, 2.0, kPi / 2};
  Pose q = pose_compose(p, Twist{3.0, 4.0, 0.0});
  EXPECT_NEAR(q.x - (1.0 + std::cos(kPi / 2) * 3.0 - std::sin(kPi / 2) * 4.0), 0.0, 1e-15);
  EXPECT_NEAR(q.y - (2.0 + std::sin(kPi / 2) * 3.0 + std::cos(kPi / 2) * 4.0), 0.0, 1e-15);
}

TEST(Geometry, FrameConversionRoundtrip) {
  Pose p{-1.5, 3.25, -1.1};
  Point e{2.0, 0.5};
  // world -> robot -> world must round-trip (robot_to_world treats e as a
  // robot-frame point; world_to_robot brings it back).
  Point z = robot_to_world(e, p);
  Point back = world_to_robot(z, p);
  EXPECT_NEAR(back.x - e.x, 0.0, 1e-12);
  EXPECT_NEAR(back.y - e.y, 0.0, 1e-12);
}

TEST(Geometry, SegmentsIntersect) {
  ASSERT_TRUE(segments_intersect(Point{0.0, 0.0}, Point{1.0, 1.0}, Point{0.0, 1.0}, Point{1.0, 0.0}));
  // Touching at an endpoint counts (closed segments).
  ASSERT_TRUE(segments_intersect(Point{0.0, 0.0}, Point{1.0, 0.0}, Point{1.0, 0.0}, Point{1.0, 1.0}));
  // Collinear overlap counts.
  ASSERT_TRUE(segments_intersect(Point{0.0, 0.0}, Point{2.0, 0.0}, Point{1.0, 0.0}, Point{3.0, 0.0}));
  // Parallel disjoint does not.
  ASSERT_FALSE(
      segments_intersect(Point{0.0, 0.0}, Point{1.0, 0.0}, Point{0.0, 1.0}, Point{1.0, 1.0}));
}

TEST(Geometry, PointInRectAndSegmentRect) {
  Rect rect{0.0, 0.0, 1.0, 1.0};
  ASSERT_TRUE(point_in_rect(Point{0.5, 0.5}, rect));
  ASSERT_TRUE(point_in_rect(Point{1.0, 1.0}, rect));  // closed
  ASSERT_FALSE(point_in_rect(Point{1.0 + 1e-9, 2.0}, rect));
  // Segment crossing the square / touching its corner / missing it.
  ASSERT_TRUE(segment_intersects_rect(Point{-1.0, 0.5}, Point{2.0, 0.5}, rect));
  ASSERT_TRUE(segment_intersects_rect(Point{2.0, 2.0}, Point{1.0, 1.0}, rect));  // touches corner
  ASSERT_FALSE(segment_intersects_rect(Point{2.0, 0.0}, Point{2.0, 1.0}, rect));
}
