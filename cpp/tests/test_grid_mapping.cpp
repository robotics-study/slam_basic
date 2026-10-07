// grid_mapping contract tests — the C++ mirror of python/tests/test_grid_mapping.py.
// The same contracts, the same asymmetric mini-grid (a single scan must fill the
// occupied column and double-charge the shared free cell), and bit-identical
// expectations: where Python asserts exact float equality, EXPECT_EQ pins the same
// bits here.

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/metrics.hpp"
#include "slam/core/params.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "slam/filtering/grid_mapping.hpp"
#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::core::Cell;
using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::filtering::cells_along;
using slam::filtering::GridMapping;

namespace {

// Free column 0, occupied column 1 — the mini raster of the update tests.
const std::vector<std::string> kMini = {".#", ".#"};

ParamSet mini_params() {
  // A tiny config for unit tests (anchor at the centre of the 2x2 mini raster) —
  // the mirror of Python's _params(): p_hit 0.9, anchor (0.5, 0.5), heading 0°.
  return ParamSet::from_yaml(slam::test::write_temp(
      "grid_mapping_mini.yaml",
      "algorithm: grid_mapping\nsection: filtering\nscenarios: []\nparams:\n"
      "  - name: p_hit\n    type: float\n    default: 0.9\n    description: symmetric inverse model\n"
      "  - name: x0\n    type: float\n    default: 0.5\n    description: anchor x\n"
      "  - name: y0\n    type: float\n    default: 0.5\n    description: anchor y\n"
      "  - name: theta_deg\n    type: float\n    default: 0.0\n    description: anchor heading (deg)\n"));
}

// Probe subclass: injects an episode and reads the log-odds field directly (the
// mirror of the Python tests reaching est._l / est.episode).
class Probe final : public GridMapping {
 public:
  explicit Probe(ParamSet params) : GridMapping(std::move(params)) {}
  void set_episode(const Episode& ep) { episode_ = &ep; }
  const std::vector<double>& field() const { return field_; }
};

// Extract the event names in wire order from a JSONL buffer (crude substring scan —
// the exact bytes are pinned by test_trace).
std::vector<std::string> event_names(const std::string& jsonl) {
  std::vector<std::string> out;
  const std::string key = "\"event\":\"";
  size_t pos = 0;
  while ((pos = jsonl.find(key, pos)) != std::string::npos) {
    size_t begin = pos + key.size();
    size_t end = jsonl.find('"', begin);
    out.push_back(jsonl.substr(begin, end - begin));
    pos = end;
  }
  return out;
}

// Parse the [row, col, l] triples of the map_updated line (crude scan — the wire
// format itself is pinned by test_trace).
std::vector<std::array<double, 3>> parse_cells(const std::string& jsonl) {
  size_t pos = jsonl.find("\"cells\":[");
  if (pos == std::string::npos) return {};
  pos += std::strlen("\"cells\":[");
  std::vector<std::array<double, 3>> out;
  while (pos < jsonl.size() && jsonl[pos] == '[') {
    std::array<double, 3> triple{};
    for (int k = 0; k < 3; ++k) {
      ++pos;  // skip '[' or ','
      char* end = nullptr;
      double v = std::strtod(jsonl.c_str() + pos, &end);
      EXPECT_NE(end, jsonl.c_str() + pos) << "unparsable cell value at " << pos;
      triple[k] = v;
      pos = static_cast<size_t>(end - jsonl.c_str());
    }
    EXPECT_EQ(jsonl[pos], ']');
    ++pos;  // consume ']'
    out.push_back(triple);
    if (pos < jsonl.size() && jsonl[pos] == ',') ++pos;
  }
  return out;
}

// Assert a walk result equals the expected (row, col) list, element by element.
void expect_cells(const std::vector<Cell>& got, const std::vector<std::array<int, 2>>& want) {
  ASSERT_EQ(got.size(), want.size());
  for (size_t i = 0; i < want.size(); ++i) {
    EXPECT_EQ(got[i].row, want[i][0]) << "cell " << i;
    EXPECT_EQ(got[i].col, want[i][1]) << "cell " << i;
  }
}

}  // namespace

TEST(GridMapping, ConfigDefaultsAndRangeValidation) {
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filtering/grid_mapping.yaml"));
  EXPECT_EQ(params.algorithm(), "grid_mapping");
  EXPECT_EQ(params.section(), "filtering");
  EXPECT_DOUBLE_EQ(params.get_float("p_hit"), 0.9);
  // The anchor defaults are the tour's start cell center, exact binary floats.
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 5.875);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 4.375);
  EXPECT_DOUBLE_EQ(params.get_float("theta_deg"), 180.0);
  // p_hit <= 0.5 makes the inverse model uninformative — out of range by contract.
  EXPECT_THROW(params.set("p_hit", 0.4), std::runtime_error);
}

TEST(GridMapping, CellsAlongWalkRules) {
  // The walk mirrors raycast's DDA on the raw segment vector: an axis with zero
  // delta never steps, a tMax tie takes Y, and a degenerate segment is just its own
  // cell (the start IS the hit). All-free 2x2 — this pins pure walk geometry.
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid({"..", ".."});

  // Due east from (0.5, 0.5): dy == 0 → y never steps; stops ON the target cell.
  expect_cells(cells_along(grid, Point{0.5, 0.5}, Point{1.5, 0.5}), {{1, 0}, {1, 1}});
  // Due north: dx == 0 → x never steps.
  expect_cells(cells_along(grid, Point{0.5, 0.5}, Point{0.5, 1.5}), {{1, 0}, {0, 0}});
  // Exact diagonal from the cell center: tMaxX == tMaxY at the shared corner and
  // the tie takes Y — the middle cell is (0, 0) (row index grows downward), NOT
  // (1, 1); a wrong tie rule would pin [(1,0),(1,1),(0,1)] here.
  expect_cells(cells_along(grid, Point{0.5, 0.5}, Point{1.5, 1.5}), {{1, 0}, {0, 0}, {0, 1}});
  // Degenerate: the endpoint sits in the start cell — that cell IS the hit.
  expect_cells(cells_along(grid, Point{0.5, 0.5}, Point{0.75, 0.25}), {{1, 0}});
  // A segment starting off-raster updates nothing (impossible with an exact pose,
  // but the walk's contract is explicit about it).
  expect_cells(cells_along(grid, Point{5.0, 5.0}, Point{0.5, 0.5}), {});
}

TEST(GridMapping, UpdateHitsPassesAndUnknowns) {
  // One hit +logit(p) on the endpoint cell; every passed-through cell −logit(p) per
  // pass (exact floats — 0.0 − L − L is exactly −2L); a never-touched cell keeps its
  // prior 0.0 bit-for-bit.
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params());
  Episode episode;
  episode.grid = &grid;
  est.set_episode(episode);

  const double L = std::log(9.0);  // logit(0.9)
  Step step;
  step.t = 0;
  step.gt = Pose{0.5, 0.5, 0.0};
  step.has_scan = true;
  step.scan = {Point{1.0, 0.0}, Point{0.0, 1.0}};
  est.update(step, nullptr);

  const std::vector<double>& field = est.field();
  // Beam 1 due east ends inside the occupied cell (1, 1); beam 2 due north ends
  // inside the occupied cell (0, 0). Both pass through the robot's own cell.
  EXPECT_EQ(field[1 * 2 + 1], L);
  EXPECT_EQ(field[0 * 2 + 0], L);
  EXPECT_EQ(field[1 * 2 + 0], -2.0 * L);  // passed through twice — exactly −2L
  EXPECT_EQ(field[0 * 2 + 1], 0.0);       // never touched: the prior stays, bit-for-bit

  // A point inside the robot's own cell makes that cell itself the hit.
  Step step2;
  step2.t = 1;
  step2.gt = Pose{0.5, 0.5, 0.0};
  step2.has_scan = true;
  step2.scan = {Point{0.25, 0.0}};
  est.update(step2, nullptr);
  EXPECT_EQ(field[1 * 2 + 0], -L);  // −2L + L
}

TEST(GridMapping, TraceEventsAndRowMajorSorted) {
  // pose_estimated precedes map_updated at the same t, and the diff lists cells
  // row-major sorted (the byte order std::set<Cell> ordered by (row, col) emits).
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Episode episode;
  episode.grid = &grid;

  std::ostringstream os;
  TraceRecorder rec(os);
  Probe est(mini_params());
  est.set_episode(episode);
  Step step;
  step.t = 0;
  step.gt = Pose{0.5, 0.5, 0.0};
  step.has_scan = true;
  step.scan = {Point{1.0, 0.0}, Point{0.0, 1.0}};
  est.update(step, &rec);
  std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "pose_estimated");
  EXPECT_EQ(names[1], "map_updated");

  // The pose is the declared anchor (no odom arrived at t = 0): [x, y, theta].
  std::vector<std::array<double, 3>> cells = parse_cells(jsonl);
  ASSERT_EQ(cells.size(), 3u);
  // Only TOUCHED cells are emitted, row-major sorted: row 0 before row 1, col
  // ascending inside a row.
  EXPECT_EQ(cells[0][0], 0.0); EXPECT_EQ(cells[0][1], 0.0);
  EXPECT_EQ(cells[1][0], 1.0); EXPECT_EQ(cells[1][1], 0.0);
  EXPECT_EQ(cells[2][0], 1.0); EXPECT_EQ(cells[2][1], 1.0);
  const double L = std::log(9.0);
  EXPECT_EQ(cells[0][2], L);
  EXPECT_EQ(cells[1][2], -2.0 * L);
  EXPECT_EQ(cells[2][2], L);

  // The pose payload itself: [0.5, 0.5, 0.0] — anchor defaults, exact binary floats.
  EXPECT_NE(jsonl.find("\"pose\":[0.5,0.5,0]"), std::string::npos);
}

TEST(GridMapping, RunOnScenarioIsExactAndHonest) {
  // The branch premise end-to-end: noiseless odometry anchored to the declared
  // start pose reproduces ground truth bit-for-bit (pose[0] is exact, so ATE/RPE
  // read float-noise ~0), while the map keeps its honest ceiling — the sealed block
  // interiors keep prior 0.0 and IoU sits near but below 1.
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/office01_tour.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                             nullptr, sc.sigma_xy, sc.sigma_theta, sc.seed);
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filtering/grid_mapping.yaml"));
  GridMapping est(params);
  EstimateResult result = est.run(episode, nullptr);
  ASSERT_EQ(result.poses.size(), episode.steps.size());

  // Pose is GIVEN: the anchor equals ground truth bit-for-bit and noiseless odometry
  // keeps it there (ATE/RPE are float noise, not drift).
  EXPECT_EQ(result.poses[0].x, episode.steps[0].gt.x);
  EXPECT_EQ(result.poses[0].y, episode.steps[0].gt.y);
  EXPECT_EQ(result.poses[0].theta, episode.steps[0].gt.theta);

  std::map<std::string, double> metrics = slam::core::evaluate(result, episode);
  EXPECT_LT(metrics["ate_rmse"], 1e-9);
  EXPECT_LT(metrics["rpe_rmse"], 1e-9);

  // Sealed interiors (never reachable by any beam): prior stays EXACTLY 0.0.
  EXPECT_EQ(result.grid.at(26, 37), 0.0);
  EXPECT_EQ(result.grid.at(9, 7), 0.0);
  EXPECT_EQ(result.grid.at(0, 47), 0.0);
  // Everything observable got occupied evidence: IoU lands in the honest band
  // (structural blind spots + a handful of noise-flipped cells keep it below 1).
  EXPECT_GT(metrics["map_iou"], 0.85);
  EXPECT_LT(metrics["map_iou"], 0.95);
}
