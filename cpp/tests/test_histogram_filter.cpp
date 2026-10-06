// histogram_filter contract tests — the C++ mirror of python/tests/test_histogram_filter.py.
// The same five contracts, the same asymmetric mini-grid (a single scan must exclude
// every wrong state), and bit-identical expectations: where Python asserts exact
// float equality, EXPECT_EQ pins the same bits here.

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/params.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "slam/filtering/histogram_filter.hpp"
#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Point;
using slam::core::Pose;
using slam::core::SensorConfig;
using slam::core::TraceRecorder;
using slam::core::Twist;
using slam::filtering::HistogramFilter;

namespace {

// Free cells row-major: (0,1) → center (1.5, 1.5), (0,2) → (2.5, 1.5), (1,2) → (2.5, 0.5).
const std::vector<std::string> kMini = {"#..", "##."};

ParamSet mini_params() {
  // A tiny-lattice config for unit tests (5 beams over 360° → lattice step π/2,
  // B = 4 heading bins, off = 2) — the mirror of Python's _params().
  return ParamSet::from_yaml(slam::test::write_temp(
      "histogram_filter_mini.yaml",
      "algorithm: histogram_filter\nsection: filtering\nscenarios: []\nparams:\n"
      "  - name: p_slip\n    type: float\n    default: 0.1\n    description: slip mix\n"
      "  - name: beams\n    type: int\n    default: 5\n    description: beam count\n"
      "  - name: fov_deg\n    type: float\n    default: 360.0\n    description: field of view\n"
      "  - name: range_max\n    type: float\n    default: 4.0\n    description: max range\n"
      "  - name: sigma_range\n    type: float\n    default: 0.05\n    description: beam noise\n"));
}

SensorConfig mini_sensor() {
  SensorConfig s;
  s.type = "beam";
  s.range_max = 4.0;
  s.sigma_range = 0.05;
  s.has_beams = true;
  s.beams = 5;
  s.has_fov_deg = true;
  s.fov_deg = 360.0;
  return s;
}

// Probe subclass: drives init_state/predict and reads the belief directly (the
// mirror of the Python tests reaching _init_state/_predict/_belief).
class Probe final : public HistogramFilter {
 public:
  explicit Probe(ParamSet params) : HistogramFilter(std::move(params)) {}
  using HistogramFilter::init_state;
  using HistogramFilter::predict;
  void set_episode(const Episode& ep) { episode_ = &ep; }
  std::vector<double>& belief() { return belief_; }
  int bins() const { return bins_; }
  int off() const { return off_; }
  double w_bin() const { return w_bin_; }
  size_t n_states() const { return static_cast<size_t>(n_states_); }
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

// Parse the [row, col, p] triples of the belief_updated line (crude scan — the wire
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

}  // namespace

TEST(HistogramFilter, ConfigDefaultsAndRangeValidation) {
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filtering/histogram_filter.yaml"));
  EXPECT_EQ(params.algorithm(), "histogram_filter");
  EXPECT_EQ(params.section(), "filtering");
  EXPECT_DOUBLE_EQ(params.get_float("p_slip"), 0.1);
  EXPECT_EQ(params.get_int("beams"), 361);
  EXPECT_DOUBLE_EQ(params.get_float("fov_deg"), 360.0);
  EXPECT_DOUBLE_EQ(params.get_float("range_max"), 6.0);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_range"), 0.05);
  // Out-of-range sets raise (the demo injects the scenario sensor through set()).
  EXPECT_THROW(params.set("beams", 362), std::runtime_error);
}

TEST(HistogramFilter, UniformPriorCollapsesToTrueState) {
  // One noise-free scan collapses the uniform prior onto exactly one state and the
  // readout pose equals the ground-truth pose bit-for-bit (the lattice makes the
  // true pose exactly representable).
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Point gt{1.5, 1.5};  // center of cell (0,1), the first free cell
  Episode episode =
      slam::core::build_episode(grid, {gt}, 0.5, mini_sensor(), nullptr, 0.0, 0.0, 7);
  ASSERT_EQ(episode.steps.size(), 1u);

  HistogramFilter est(mini_params());
  EstimateResult result = est.run(episode, nullptr);
  ASSERT_EQ(result.poses.size(), 1u);
  // Position exact to the bit; heading is the lattice bin for θ = 0 (exact 0.0).
  EXPECT_EQ(result.poses[0].x, gt.x);
  EXPECT_EQ(result.poses[0].y, gt.y);
  EXPECT_EQ(result.poses[0].theta, 0.0);
}

TEST(HistogramFilter, TraceEventsAndCellMarginals) {
  // run() echoes step_observed before the estimate events; belief_updated carries
  // EVERY cell row-major (occupied cells emit exactly 0.0 — they are not states).
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Episode episode =
      slam::core::build_episode(grid, {Point{1.5, 1.5}}, 0.5, mini_sensor(), nullptr, 0.0, 0.0, 7);

  std::ostringstream os;
  TraceRecorder rec(os);
  HistogramFilter est(mini_params());
  est.run(episode, &rec);
  std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "step_observed");
  EXPECT_EQ(names[1], "pose_estimated");
  EXPECT_EQ(names[2], "belief_updated");

  // Every cell of the 2x3 raster, row-major.
  std::vector<std::array<double, 3>> cells = parse_cells(jsonl);
  ASSERT_EQ(cells.size(), 6u);
  for (size_t i = 0; i < cells.size(); ++i) {
    EXPECT_EQ(cells[i][0], static_cast<double>(i / 3));
    EXPECT_EQ(cells[i][1], static_cast<double>(i % 3));
  }
  // Occupied cells (0,0), (1,0), (1,1) are not states: exactly 0.0.
  EXPECT_EQ(cells[0][2], 0.0);
  EXPECT_EQ(cells[3][2], 0.0);
  EXPECT_EQ(cells[4][2], 0.0);
  // The free cell the robot sits on carries the whole mass after one scan.
  EXPECT_NEAR(cells[1][2], 1.0, 1e-12);
}

TEST(HistogramFilter, PredictSlipsAndRotatesOnTheLattice) {
  // Child-of-movement: (1−p_slip) rides the twist onto the child state, p_slip stays
  // put; a move into an occupied cell does not move (slide); a dtheta of one lattice
  // step rotates the heading bin by exactly one (self-heal).
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  // free row-major: (0,1),(0,2),(1,2) → centers (1.5,1.5), (2.5,1.5), (2.5,0.5)
  Episode episode;
  episode.grid = &grid;

  Probe est(mini_params());
  est.set_episode(episode);
  est.init_state();  // private on purpose — this unit-tests the predict step
  ASSERT_EQ(est.bins(), 4);
  EXPECT_EQ(est.off(), 2);
  double w_bin = est.w_bin();
  size_t fi_01 = 0, fi_02 = 1;  // free cells (0,1) → (1.5,1.5) and (0,2) → (2.5,1.5)

  // δ at (cell (0,1), heading bin 2 ↔ θ = 0). Move +x into free cell (0,2): mass
  // splits 0.9 onto the child state, 0.1 stays — exact floats.
  est.belief().assign(est.n_states(), 0.0);
  est.belief()[fi_01 * 4 + 2] = 1.0;
  est.predict(Twist{1.0, 0.0, 0.0});
  EXPECT_EQ(est.belief()[fi_02 * 4 + 2], 0.9);
  EXPECT_EQ(est.belief()[fi_01 * 4 + 2], 0.1);

  // Slide: from (0,2) a further +x move leaves the grid — the hypothesis does not
  // move; both terms land on the SAME state and sum back to exactly 1.
  est.belief().assign(est.n_states(), 0.0);
  est.belief()[fi_02 * 4 + 2] = 1.0;
  est.predict(Twist{1.0, 0.0, 0.0});
  EXPECT_EQ(est.belief()[fi_02 * 4 + 2], 1.0);

  // Heading rotation: a twist of exactly one lattice step rotates bin 2 → 3
  // (0.9 child / 0.1 stay — the heading axis self-heals by construction).
  est.predict(Twist{0.0, 0.0, w_bin});
  EXPECT_EQ(est.belief()[fi_02 * 4 + 3], 0.9);
  EXPECT_EQ(est.belief()[fi_02 * 4 + 2], 0.1);
}

TEST(HistogramFilter, RunOnScenarioIsExact) {
  // The showcase run: lattice-matched corridor, the belief collapses onto the true
  // state at t=0 and rides it — ATE/RPE are zero to float rounding.
  slam::maps::Scenario sc =
      slam::maps::load_scenario(slam::test::repo_path("maps/scenarios/corridor01_back_and_forth.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                             nullptr, sc.sigma_xy, sc.sigma_theta, sc.seed);
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filtering/histogram_filter.yaml"));
  HistogramFilter est(params);
  EstimateResult result = est.run(episode, nullptr);
  ASSERT_EQ(result.poses.size(), episode.steps.size());

  double total = 0.0;
  for (size_t i = 0; i < result.poses.size(); ++i) {
    double dx = result.poses[i].x - episode.steps[i].gt.x;
    double dy = result.poses[i].y - episode.steps[i].gt.y;
    total += dx * dx + dy * dy;
  }
  double ate = std::sqrt(total / static_cast<double>(result.poses.size()));
  EXPECT_LT(ate, 1e-9);  // exact to float rounding (the demo reports ~1.3e-16)
}
