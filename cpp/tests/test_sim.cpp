// Simulator contract tests: resample / DDA raycast / visibility / draw order.
// The golden floats below pin the cross-language contract (identical to the Python
// test goldens); they were produced by this exact formula chain once and are frozen.

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/sim.hpp"
#include "slam/core/types.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::kPi;
using slam::core::Point;
using slam::core::SensorConfig;
using slam::core::Step;

namespace {

std::vector<Point> path2() { return {Point{0.5, 1.5}, Point{1.5, 1.5}}; }

}  // namespace

TEST(Sim, ResampleStraightAndCorner) {
  auto poses = slam::core::resample({Point{0.0, 0.0}, Point{2.0, 0.0}}, 1.0);
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_DOUBLE_EQ(poses[0].x, 0.0);
  EXPECT_DOUBLE_EQ(poses[0].y, 0.0);
  EXPECT_DOUBLE_EQ(poses[0].theta, 0.0);
  EXPECT_DOUBLE_EQ(poses[1].x, 1.0);
  EXPECT_DOUBLE_EQ(poses[2].x, 2.0);
  EXPECT_DOUBLE_EQ(poses[2].theta, 0.0);

  // Corner: heading flips on the step whose segment crosses the corner; the final
  // point keeps the previous heading.
  auto corners = slam::core::resample({Point{0.0, 0.0}, Point{1.0, 0.0}, Point{1.0, 1.0}}, 1.0);
  ASSERT_EQ(corners.size(), 3u);
  EXPECT_DOUBLE_EQ(corners[0].theta, 0.0);
  EXPECT_DOUBLE_EQ(corners[1].theta, kPi / 2);
  EXPECT_DOUBLE_EQ(corners[2].theta, kPi / 2);
  EXPECT_DOUBLE_EQ(corners[1].x, 1.0);
  EXPECT_DOUBLE_EQ(corners[1].y, 0.0);
  EXPECT_DOUBLE_EQ(corners[2].x, 1.0);
  EXPECT_DOUBLE_EQ(corners[2].y, 1.0);
}

TEST(Sim, RaycastGoldens) {
  // 3x5 grid, res 1, origin 0: only cell (row 1, col 2) is occupied — its rect is
  // x in [2,3], y in [1,2].
  auto g = slam::test::make_grid({".....", "..#..", "....."});
  EXPECT_DOUBLE_EQ(*slam::core::raycast(g, 0.5, 1.5, 0.0, 10.0), 1.5);  // wall face at x=2
  ASSERT_FALSE(slam::core::raycast(g, 0.5, 1.5, kPi / 2, 10.0).has_value());  // leaves the map
  ASSERT_FALSE(slam::core::raycast(g, 0.5, 1.5, 0.0, 1.4).has_value());  // beyond range_max
  EXPECT_DOUBLE_EQ(*slam::core::raycast(g, 2.5, 1.5, 0.0, 10.0), 0.0);   // starts inside
}

TEST(Sim, LandmarkVisibility) {
  auto g = slam::test::make_grid({".....", "..#..", "....."});
  ASSERT_FALSE(slam::core::landmark_visible(g, Point{0.5, 1.5}, Point{2.5, 1.5}));  // wall between
  ASSERT_TRUE(slam::core::landmark_visible(g, Point{0.5, 0.5}, Point{4.5, 0.5}));
}

TEST(Sim, EpisodeDrawOrderGolden) {
  /* Step 0 carries no odom; observation draws come first (range pair then bearing
     pair for the single landmark); the odometry triple follows for the next move. */
  auto g = slam::test::make_grid({"...", "...", "..."});
  SensorConfig sensor;
  sensor.type = "landmarks";
  sensor.range_max = 10.0;
  sensor.sigma_range = 0.05;
  sensor.has_sigma_bearing = true;
  sensor.sigma_bearing = 0.03;
  std::vector<Point> landmarks{Point{2.5, 1.5}};

  Episode ep = slam::core::build_episode(g, path2(), 1.0, sensor, &landmarks, 0.05, 0.035, 42);
  ASSERT_EQ(ep.steps.size(), 2u);

  const Step& s0 = ep.steps[0];
  EXPECT_EQ(s0.t, 0);
  ASSERT_FALSE(s0.has_odom);
  EXPECT_DOUBLE_EQ(s0.gt.x, 0.5);
  EXPECT_DOUBLE_EQ(s0.gt.y, 1.5);
  EXPECT_DOUBLE_EQ(s0.gt.theta, 0.0);
  // Golden obs: range = 2 + 0.05*N(0,1)(draws 1-2), bearing = wrap(0 + 0.03*N(0,1)
  // (draws 3-4)) — the same bits the Python golden pins.
  ASSERT_TRUE(s0.has_obs);
  ASSERT_EQ(s0.obs.size(), 1u);
  EXPECT_EQ(s0.obs[0].id, 0);
  EXPECT_DOUBLE_EQ(s0.obs[0].range, 2.0441124453111135);
  EXPECT_DOUBLE_EQ(s0.obs[0].bearing, -0.013525496271565803);

  // Determinism: rebuilding from the same seed reproduces every field exactly.
  Episode ep2 = slam::core::build_episode(g, path2(), 1.0, sensor, &landmarks, 0.05, 0.035, 42);
  ASSERT_EQ(ep2.steps.size(), ep.steps.size());
  for (size_t t = 0; t < ep.steps.size(); ++t) {
    EXPECT_DOUBLE_EQ(ep2.steps[t].gt.x, ep.steps[t].gt.x);
    EXPECT_DOUBLE_EQ(ep2.steps[t].gt.y, ep.steps[t].gt.y);
    EXPECT_DOUBLE_EQ(ep2.steps[t].gt.theta, ep.steps[t].gt.theta);
  }
  EXPECT_DOUBLE_EQ(ep2.steps[0].obs[0].range, s0.obs[0].range);
  EXPECT_DOUBLE_EQ(ep2.steps[0].obs[0].bearing, s0.obs[0].bearing);

  // Step 1: odom = exact twist (straight here: dx=1, dtheta=0) + noise.
  const Step& s1 = ep.steps[1];
  ASSERT_TRUE(s1.has_odom);
  EXPECT_LT(std::abs(s1.odom.dx - 1.0), 0.5);
  EXPECT_LT(std::abs(s1.odom.dy), 0.5);
  EXPECT_LT(std::abs(s1.odom.dtheta), 0.5);
}

TEST(Sim, EpisodeBeamScanIsRobotFrame) {
  // Wall at x in [2,3] seen from (0.5,1.5) heading east: the beam fan is centered
  // on theta=0; with sigma_range=0 the hit points are exact.
  auto g = slam::test::make_grid({".....", "..#..", "....."});
  SensorConfig sensor;
  sensor.type = "beam";
  sensor.range_max = 10.0;
  sensor.sigma_range = 0.0;
  sensor.has_beams = true;
  sensor.beams = 3;
  sensor.has_fov_deg = true;
  sensor.fov_deg = 180.0;

  Episode ep = slam::core::build_episode(g, {Point{0.5, 1.5}}, 1.0, sensor, nullptr, 0.05, 0.035,
                                         42);
  const Step& s0 = ep.steps[0];
  ASSERT_TRUE(s0.has_scan);
  // beams=3: phi in {-pi/2, 0, +pi/2}. Center beam hits the wall face at dx=1.5;
  // the ±90° beams leave the map (miss) -> exactly one point, robot frame x=1.5.
  ASSERT_EQ(s0.scan.size(), 1u);
  EXPECT_DOUBLE_EQ(s0.scan[0].x, 1.5);
  EXPECT_LT(std::abs(s0.scan[0].y), 1e-12);
}
