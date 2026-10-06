// PGM reader (P2 ascii + P5 binary) and the occupancy grid contract — the C++
// mirror of python/tests/test_grid.py.

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/maps/occupancy_grid.hpp"
#include "slam/maps/pgm.hpp"
#include "test_util.hpp"

using slam::maps::OccupancyGrid2D;
using slam::maps::PgmImage;

TEST(Pgm, P2AsciiWithCommentAndBlockLayout) {
  std::string p = slam::test::write_temp("a.pgm", "P2\n# comment\n2 2\n255\n0 255\n128 42\n");
  PgmImage img = slam::maps::load_pgm(p);
  EXPECT_EQ(img.width, 2);
  EXPECT_EQ(img.height, 2);
  EXPECT_EQ(img.pixels, (std::vector<std::uint16_t>{0, 255, 128, 42}));
}

TEST(Pgm, P5BinaryMatchesAsciiValues) {
  std::string content = "P5\n2 2\n255\n";
  for (int v : {0, 255, 128, 42}) content += static_cast<char>(static_cast<unsigned char>(v));
  std::string p = slam::test::write_temp("b.pgm", content);
  PgmImage img = slam::maps::load_pgm(p);
  EXPECT_EQ(img.width, 2);
  EXPECT_EQ(img.height, 2);
  EXPECT_EQ(img.pixels, (std::vector<std::uint16_t>{0, 255, 128, 42}));
}

TEST(Pgm, RejectsOtherMagic) {
  std::string p = slam::test::write_temp("c.pgm", "P6\n1 1\n255\n0");
  EXPECT_THROW(slam::maps::load_pgm(p), std::runtime_error);
}

TEST(Grid, ThresholdAndFrames) {
  // 2x1 grid, resolution 0.5, origin (1, 2). Row 0 = TOP image row.
  std::vector<std::uint16_t> pixels{0, 255};
  OccupancyGrid2D g(pixels, 1, 2, 0.5, {1.0, 2.0});
  EXPECT_EQ(g.height(), 1);
  EXPECT_EQ(g.width(), 2);
  ASSERT_TRUE(g.occupied(0, 0));
  ASSERT_FALSE(g.occupied(0, 1));
  // cell_to_world fixed formula: x = ox + (c+0.5)r ; y = oy + ((H-1-r)+0.5)r
  auto c0 = g.cell_to_world(0, 0);
  EXPECT_DOUBLE_EQ(c0.x, 1.0 + 0.5 * 0.5);
  EXPECT_DOUBLE_EQ(c0.y, 2.0 + 0.5 * 0.5);
  // world_to_cell inverts it exactly at the center.
  auto c1 = g.cell_to_world(0, 1);
  auto back = g.world_to_cell(c1.x, c1.y);
  EXPECT_EQ(back.row, 0);
  EXPECT_EQ(back.col, 1);
  ASSERT_FALSE(g.in_bounds(1, 0));
  ASSERT_FALSE(g.in_bounds(0, 2));
  // Out of bounds is never occupied.
  ASSERT_FALSE(g.occupied(5, 5));
}
