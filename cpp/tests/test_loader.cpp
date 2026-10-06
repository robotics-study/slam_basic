// Scenario / map loaders against the real repo yaml files — the C++ mirror of
// python/tests/test_loader.py.

#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::maps::load_map;
using slam::maps::load_scenario;
using slam::maps::OccupancyGrid2D;
using slam::maps::Scenario;

TEST(Loader, LoadsRealGrid) {
  OccupancyGrid2D g = load_map(slam::test::repo_path("maps/grid/corridor01.yaml"));
  EXPECT_EQ(g.height(), 7);
  EXPECT_EQ(g.width(), 25);
  EXPECT_DOUBLE_EQ(g.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(g.origin()[0], 0.0);
  EXPECT_DOUBLE_EQ(g.origin()[1], 0.0);
  // Border walls are occupied, corridor center is free.
  ASSERT_TRUE(g.occupied(0, 0));
  ASSERT_FALSE(g.occupied(3, 1));
}

TEST(Loader, LoadsBeamScenario) {
  Scenario sc = load_scenario(slam::test::repo_path("maps/scenarios/corridor01_back_and_forth.yaml"));
  EXPECT_EQ(sc.sensor.type, "beam");
  ASSERT_TRUE(sc.sensor.has_beams);
  EXPECT_EQ(sc.sensor.beams, 361);
  ASSERT_TRUE(sc.sensor.has_fov_deg);
  EXPECT_DOUBLE_EQ(sc.sensor.fov_deg, 360.0);
  ASSERT_FALSE(sc.sensor.has_sigma_bearing);
  ASSERT_FALSE(sc.has_landmarks);
  // Lattice-matched odometry: sub-quantum position noise, ZERO heading noise (the
  // histogram lattice cannot represent a rotation finer than its own bin).
  EXPECT_DOUBLE_EQ(sc.sigma_xy, 0.05);
  EXPECT_DOUBLE_EQ(sc.sigma_theta, 0.0);
  EXPECT_DOUBLE_EQ(sc.step_meters, 0.5);
  EXPECT_EQ(sc.seed, 42);
  EXPECT_EQ(sc.waypoints.size(), 3u);
}

TEST(Loader, LoadsLandmarksScenario) {
  Scenario sc = load_scenario(slam::test::repo_path("maps/scenarios/office01_landmarks.yaml"));
  EXPECT_EQ(sc.sensor.type, "landmarks");
  ASSERT_FALSE(sc.sensor.has_beams);
  ASSERT_FALSE(sc.sensor.has_fov_deg);
  ASSERT_TRUE(sc.sensor.has_sigma_bearing);
  EXPECT_DOUBLE_EQ(sc.sensor.sigma_bearing, 0.03);
  ASSERT_TRUE(sc.has_landmarks);
  EXPECT_EQ(sc.landmarks.size(), 10u);
}

TEST(Loader, MapPathResolvesAbsolute) {
  Scenario sc = load_scenario(slam::test::repo_path("maps/scenarios/corridor01_back_and_forth.yaml"));
  std::filesystem::path expected =
      std::filesystem::weakly_canonical(slam::test::repo_path("maps/grid/corridor01.yaml"));
  EXPECT_EQ(std::filesystem::path(sc.map_path), expected);
}
