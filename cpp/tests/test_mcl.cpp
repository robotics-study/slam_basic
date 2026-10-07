// mcl contract tests — the C++ mirror of python/tests/test_mcl.py. The same
// contracts, the same asymmetric mini-grid, and bit-identical expectations: the
// KLD loop draws samples ONE AT A TIME — ancestor walk with strict `>` and guard
// i < n−1, three gaussians, pose_compose on the OLD pose — until n ≥ n_chi(k) AND
// n ≥ n_min, or the cap; kld_bound is Fox's Wilson–Hilferty closed form at pinned
// exact doubles (0.0 for the degenerate k ≤ 1); and the readout is the
// particle_filter page's (weighted mean, circular heading, wrapped spread).
//
// The scenario run pins the branch story end-to-end on corridor03_drift: t = 0
// draws the FULL prior budget (800 particles, all distinct — no resampling has
// happened yet; both east- and west-facing twins survive); at t = 1 the first KLD
// step samples until k = 41 bins justify 319; once the west blob dies k collapses
// to 2 and every later step draws exactly ceil(n_chi(2)) = 33 samples — sample
// count IS uncertainty, and the U-turn re-widens it (k spikes as the cluster
// crosses bins and the ±π seam). Everything below is deterministic: seed 42 pins
// these numbers bit-for-bit in both languages.

#include <cmath>
#include <array>
#include <cstring>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/geometry.hpp"
#include "slam/core/params.hpp"
#include "slam/core/rng.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/stats.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "slam/filtering/mcl.hpp"
#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Pose;
using slam::core::Rng;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;
using slam::filtering::kld_bound;
using slam::filtering::Mcl;

namespace {

// Free cells row-major: (0,1) → center (1.5, 1.5), (0,2) → (2.5, 1.5), (1,2) → (2.5, 0.5).
const std::vector<std::string> kMini = {"#..", "##."};

ParamSet mini_params(int n_min = 2, int max_particles = 8, int seed = 7,
                     double sigma_xy = 0.25, double sigma_theta = 0.1,
                     double range_max = 4.0, double sigma_range = 0.3) {
  // A tiny config for unit tests (the start cell is the one holding (1.5, 1.5)) —
  // the mirror of Python's _params(). The max_particles floor here (2) is smaller
  // than the shipped yaml's (8) so tiny clouds can pin the mechanics exactly.
  return ParamSet::from_yaml(slam::test::write_temp(
      "mcl_mini.yaml",
      ("algorithm: mcl\nsection: filtering\nscenarios: []\nparams:\n"
       "  - name: seed\n    type: int\n    default: " +
       std::to_string(seed) +
       "\n    description: rng seed\n"
       "  - name: x0\n    type: float\n    default: 1.5\n    description: prior cell center x\n"
       "  - name: y0\n    type: float\n    default: 1.5\n    description: prior cell center y\n"
       "  - name: epsilon\n    type: float\n    default: 0.1\n    min: 0.001\n    max: 1.0\n"
       "    description: KL bound\n"
       "  - name: delta\n    type: float\n    default: 0.01\n    min: 0.0001\n    max: 0.5\n"
       "    description: failure probability\n"
       "  - name: bin_xy\n    type: float\n    default: 0.5\n    min: 0.05\n"
       "    description: position bin edge\n"
       "  - name: bin_theta\n    type: float\n    default: 0.25\n    min: 0.001\n"
       "    description: heading bin width\n"
       "  - name: n_min\n    type: int\n    default: " +
       std::to_string(n_min) +
       "\n    min: 1\n    max: 4096\n    description: sample floor\n"
       "  - name: max_particles\n    type: int\n    default: " +
       std::to_string(max_particles) +
       "\n    min: 2\n    max: 4096\n    description: prior budget / hard cap\n"
       "  - name: range_max\n    type: float\n    default: " +
       std::to_string(range_max) +
       "\n    min: 0.1\n    description: max range\n"
       "  - name: sigma_range\n    type: float\n    default: " +
       std::to_string(sigma_range) +
       "\n    min: 0.001\n    description: beam noise\n"
       "  - name: sigma_xy\n    type: float\n    default: " +
       std::to_string(sigma_xy) +
       "\n    min: 0.001\n    max: 0.5\n    description: motion position noise\n"
       "  - name: sigma_theta\n    type: float\n    default: " +
       std::to_string(sigma_theta) +
       "\n    min: 0.001\n    max: 3.14159265358979\n    description: motion heading noise\n")
          .c_str()));
}

// Probe subclass: drives the private stages and reads/writes the cloud directly
// (the mirror of the Python tests reaching _x/_y/_theta/_w/_rng/_z_q/_initialized).
class Probe final : public Mcl {
 public:
  explicit Probe(ParamSet params) : Mcl(std::move(params)) {}
  using Mcl::init_particles;
  using Mcl::kld_update;
  void set_episode(const Episode& ep) { episode_ = &ep; }
  void mark_initialized() { initialized_ = true; }
  double z_q() const { return z_q_; }
  std::vector<double>& x() { return x_; }
  std::vector<double>& y() { return y_; }
  std::vector<double>& theta() { return theta_; }
  std::vector<double>& w() { return w_; }
  double draw() { return rng_.uniform01(); }  // the algorithm's own stream, advanced once
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

// Parse the [x, y, θ, w] quads of one particles_updated line (crude scan — parse a
// single LINE so each event's payload stays separate).
std::vector<std::array<double, 4>> parse_particles(const std::string& line) {
  size_t pos = line.find("\"particles\":[");
  if (pos == std::string::npos) return {};
  pos += std::strlen("\"particles\":[");
  std::vector<std::array<double, 4>> out;
  while (pos < line.size() && line[pos] == '[') {
    std::array<double, 4> quad{};
    for (int k = 0; k < 4; ++k) {
      ++pos;  // skip '[' or ','
      char* end = nullptr;
      double v = std::strtod(line.c_str() + pos, &end);
      EXPECT_NE(end, line.c_str() + pos) << "unparsable particle value at " << pos;
      quad[k] = v;
      pos = static_cast<size_t>(end - line.c_str());
    }
    EXPECT_EQ(line[pos], ']');
    ++pos;  // consume ']'
    out.push_back(quad);
    if (pos < line.size() && line[pos] == ',') ++pos;
  }
  return out;
}

// Parse the [x, y, θ] pose and [sx, sy, sθ] cov of one pose_estimated line.
std::array<double, 3> parse_triple(const std::string& line, const char* key) {
  size_t pos = line.find(key);
  if (pos == std::string::npos) return {};
  pos += std::strlen(key);
  std::array<double, 3> out{};
  for (int k = 0; k < 3; ++k) {
    ++pos;  // skip '[' or ','
    char* end = nullptr;
    double v = std::strtod(line.c_str() + pos, &end);
    EXPECT_NE(end, line.c_str() + pos) << "unparsable triple value";
    out[k] = v;
    pos = static_cast<size_t>(end - line.c_str());
  }
  return out;
}

std::vector<std::string> split_lines(const std::string& jsonl) {
  std::vector<std::string> lines;
  size_t start = 0;
  while (start < jsonl.size()) {
    size_t nl = jsonl.find('\n', start);
    if (nl == std::string::npos) nl = jsonl.size();
    if (nl > start) lines.push_back(jsonl.substr(start, nl - start));
    start = nl + 1;
  }
  return lines;
}

}  // namespace

TEST(Mcl, ConfigDefaultsAndRangeValidation) {
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/filtering/mcl.yaml"));
  EXPECT_EQ(params.algorithm(), "mcl");
  EXPECT_EQ(params.section(), "filtering");
  EXPECT_EQ(params.get_int("seed"), 42);
  EXPECT_EQ(params.get_int("max_particles"), 800);
  EXPECT_EQ(params.get_int("n_min"), 10);
  // The prior-cell defaults are the scenario start cell center, exact binary floats.
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 4.25);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 1.75);
  EXPECT_DOUBLE_EQ(params.get_float("epsilon"), 0.1);
  EXPECT_DOUBLE_EQ(params.get_float("delta"), 0.01);
  EXPECT_DOUBLE_EQ(params.get_float("bin_xy"), 0.5);
  // bin_theta is exactly radians(10°) — the paper's 10° heading bin as a binary float.
  EXPECT_EQ(params.get_float("bin_theta"), 0.17453292519943295);
  EXPECT_DOUBLE_EQ(params.get_float("range_max"), 2.5);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_range"), 0.3);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_xy"), 0.05);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_theta"), 0.05);
  // Out-of-range sets raise (the demo injects the scenario sensor through set()).
  EXPECT_THROW(params.set("max_particles", 8192), std::runtime_error);
  // The bound's quantile is computed ONCE from delta — pinned to the exact double.
  Probe est(params);
  EXPECT_EQ(est.z_q(), 2.326347874388028);  // inv_norm_cdf(1 − 0.01), pinned in test_stats
}

TEST(Mcl, KldBoundIsWilsonHilferty) {
  // n_chi(k) = (k−1)/2ε · ((1 − 2/9ν) + z_q·√(2/9ν))³ at ε = 0.1 and the pinned
  // z_q — exact doubles, pinned; k ≤ 1 is the degenerate case (bound 0.0: a one-bin
  // belief needs no guarantee beyond n_min), and the bound grows with k.
  const double z_q = slam::core::inv_norm_cdf(0.99);
  EXPECT_EQ(z_q, 2.326347874388028);
  EXPECT_EQ(kld_bound(1, 0.1, z_q), 0.0);
  EXPECT_EQ(kld_bound(0, 0.1, z_q), 0.0);
  EXPECT_EQ(kld_bound(2, 0.1, z_q), 32.92886549325934);
  EXPECT_EQ(kld_bound(3, 0.1, z_q), 46.102526736216774);
  EXPECT_EQ(kld_bound(4, 0.1, z_q), 56.8452881537776);
  EXPECT_EQ(kld_bound(5, 0.1, z_q), 66.52863321822623);
  // Monotone in k (and ε shrinks the bound: n ∝ 1/2ε at fixed k).
  std::vector<double> values;
  for (int k = 2; k < 40; ++k) values.push_back(kld_bound(k, 0.1, z_q));
  for (size_t i = 0; i + 1 < values.size(); ++i) {
    EXPECT_LT(values[i], values[i + 1]);
  }
  EXPECT_EQ(kld_bound(5, 0.2, z_q), kld_bound(5, 0.1, z_q) / 2.0);
}

TEST(Mcl, InitDrawsAscendingAndUniform) {
  // Three particles, seed pinned: x = ox + (col0 + u1)·res, y = oy + (h−1−row0 +
  // u2)·res, θ = u3·2π − π — three draws per particle IN ASCENDING PARTICLE ORDER
  // from the algorithm's own stream (replayed with a second identical Rng). Weights
  // come out exactly 1/N: at t = 0 the cloud is the FIXED prior budget.
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params(2, 3, 7));
  Episode episode;
  episode.grid = &grid;
  est.set_episode(episode);
  est.init_particles();

  Rng rng(7);
  std::vector<double> xs, ys, ths;
  for (int i = 0; i < 3; ++i) {  // cell (0,1): col0 = 1 and h − 1 − row0 = 2 − 1 − 0 = 1
    xs.push_back(0.0 + (1.0 + rng.uniform01()) * 1.0);
    ys.push_back(0.0 + (static_cast<double>(2 - 1 - 0) + rng.uniform01()) * 1.0);
    ths.push_back(rng.uniform01() * (2.0 * slam::core::kPi) - slam::core::kPi);
  }
  EXPECT_EQ(est.x(), xs);  // bit-for-bit
  EXPECT_EQ(est.y(), ys);
  EXPECT_EQ(est.theta(), ths);
  EXPECT_EQ(est.w(), (std::vector<double>{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}));  // exactly 1/N
}

TEST(Mcl, KldUpdateDrawsOneAtATimeUntilTheCap) {
  // With n_min = max_particles the loop draws EXACTLY that many samples regardless
  // of bins — so the whole per-sample chain replays bit-for-bit with a second Rng:
  // ancestor walk (strict `>`, guard i < n_old − 1), three gaussians, pose_compose
  // on the OLD pose. No scan → every ll is 0.0 → weights come out exactly uniform,
  // and the stream advanced by exactly 3 × 7 draws (one uniform + three gaussians).
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params(8, 3, 11));
  Episode episode;
  episode.grid = &grid;
  est.set_episode(episode);
  est.mark_initialized();
  std::vector<Pose> old{Pose{1.5, 1.5, 0.25}, Pose{9.0, 0.5, -1.0}};
  for (const Pose& p : old) {
    est.x().push_back(p.x);
    est.y().push_back(p.y);
    est.theta().push_back(p.theta);
  }
  est.w() = {0.25, 0.75};

  Rng rng(11);
  std::vector<Pose> expect;
  for (int i = 0; i < 3; ++i) {
    double u1 = rng.uniform01();
    int j = 0;
    double c = 0.25;
    while (j < 1 && u1 > c) {  // strict `>` over the cumulative weights
      j += 1;
      c += 0.75;
    }
    double ex = rng.gaussian(0.0, 0.25);
    double ey = rng.gaussian(0.0, 0.25);
    double et = rng.gaussian(0.0, 0.1);
    expect.push_back(slam::core::pose_compose(old[static_cast<size_t>(j)],
                                              Twist{0.5 + ex, -0.25 + ey, 0.3 + et}));
  }
  est.kld_update(Twist{0.5, -0.25, 0.3}, {});
  for (int i = 0; i < 3; ++i) {  // bit-for-bit — same operation order
    const size_t k = static_cast<size_t>(i);
    EXPECT_EQ(est.x()[k], expect[static_cast<size_t>(i)].x);
    EXPECT_EQ(est.y()[k], expect[static_cast<size_t>(i)].y);
    EXPECT_EQ(est.theta()[k], expect[static_cast<size_t>(i)].theta);
  }
  EXPECT_EQ(est.w(), (std::vector<double>{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}));  // no scan
  // The stream advanced by exactly 3 × 7 draws (one uniform + three gaussians each).
  EXPECT_EQ(est.draw(), rng.uniform01());
}

TEST(Mcl, KldUpdateStopsAtNMinWhenOneBinIsEnough) {
  // k = 1 → kld_bound is 0.0, so the stop rule reduces to n ≥ n_min. Pin the one
  // ancestor at a bin CENTER (x, y and θ all mid-bin) with u = 0 and σ pinned to
  // its minimum: both draws land in that same single bin, k stays 1, the bound
  // stays 0 — the loop stops exactly at n_min = 2 and the cap never fires.
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params(2, 4096, 5, 0.001, 0.001));
  Episode episode;
  episode.grid = &grid;
  est.set_episode(episode);
  est.mark_initialized();
  est.x() = {1.75};
  est.y() = {3.75};
  est.theta() = {0.125};
  est.w() = {1.0};

  Rng rng(5);
  std::vector<Pose> expect;
  for (int i = 0; i < 2; ++i) {  // n_old = 1 → the walk always lands on ancestor 0
    rng.uniform01();            // consumed and discarded — fixed draw order
    double ex = rng.gaussian(0.0, 0.001);
    double ey = rng.gaussian(0.0, 0.001);
    double et = rng.gaussian(0.0, 0.001);
    expect.push_back(slam::core::pose_compose(Pose{1.75, 3.75, 0.125}, Twist{ex, ey, et}));
  }
  est.kld_update(Twist{0.0, 0.0, 0.0}, {});
  EXPECT_EQ(est.x().size(), 2u);  // stopped at n_min — the bound was 0 (single bin)
  for (int i = 0; i < 2; ++i) {
    const size_t k = static_cast<size_t>(i);
    EXPECT_EQ(est.x()[k], expect[static_cast<size_t>(i)].x);
    EXPECT_EQ(est.y()[k], expect[static_cast<size_t>(i)].y);
    EXPECT_EQ(est.theta()[k], expect[static_cast<size_t>(i)].theta);
  }
  EXPECT_EQ(est.w(), (std::vector<double>{0.5, 0.5}));
}

TEST(Mcl, ReadoutMeanCircularAndSpread) {
  // Readout order (identical to the particle_filter page): weighted mean x/y; then
  // the circular heading atan2(Σ w sinθ, Σ w cosθ) — the arithmetic mean of angles
  // lies across the ±π seam, this one cannot; then population variances in a second
  // ascending pass with θ's deviation wrapped. Events: pose_estimated first,
  // particles_updated second, same t. (No odom on this step → no KLD draw.)
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params());
  Episode episode;
  episode.grid = &grid;
  est.set_episode(episode);
  est.mark_initialized();
  // Straddle the seam: an arithmetic mean of angles would land near 0; the circular
  // mean must land near ±π.
  est.x() = {1.0, 2.0, 3.0};
  est.y() = {3.0, 4.0, 5.0};
  est.theta() = {3.0, -3.0, 0.0};
  est.w() = {0.5, 0.25, 0.25};

  std::ostringstream os;
  TraceRecorder rec(os);
  Step step;
  step.t = 7;
  est.update(step, &rec);
  std::vector<std::string> names = event_names(os.str());
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "pose_estimated");
  EXPECT_EQ(names[1], "particles_updated");

  std::vector<std::string> lines = split_lines(os.str());
  ASSERT_EQ(lines.size(), 2u);
  std::array<double, 3> pose = parse_triple(lines[0], "\"pose\":");
  std::array<double, 3> cov = parse_triple(lines[0], "\"cov\":");

  // Accumulated EXACTLY like the implementation: ascending sequential += from 0.0.
  const double w[3] = {0.5, 0.25, 0.25};
  double x_hat = 0.0, y_hat = 0.0, s_sin = 0.0, s_cos = 0.0;
  for (int i = 0; i < 3; ++i) {
    const size_t k = static_cast<size_t>(i);
    x_hat += w[i] * est.x()[k];
    y_hat += w[i] * est.y()[k];
    s_sin += w[i] * slam::core::libm_sin(est.theta()[k]);
    s_cos += w[i] * slam::core::libm_cos(est.theta()[k]);
  }
  const double theta_hat = std::atan2(s_sin, s_cos);
  EXPECT_GT(std::abs(theta_hat), 3.0);  // near ±π — the seam did not explode the mean
  double var_x = 0.0, var_y = 0.0, var_t = 0.0;
  for (int i = 0; i < 3; ++i) {
    const size_t k = static_cast<size_t>(i);
    const double dx = est.x()[k] - x_hat;
    const double dy = est.y()[k] - y_hat;
    const double dt = slam::core::wrap(est.theta()[k] - theta_hat);
    var_x += w[i] * (dx * dx);
    var_y += w[i] * (dy * dy);
    var_t += w[i] * (dt * dt);
  }
  EXPECT_EQ(pose[0], x_hat);
  EXPECT_EQ(pose[1], y_hat);
  EXPECT_EQ(pose[2], theta_hat);
  EXPECT_EQ(cov[0], std::sqrt(var_x));
  EXPECT_EQ(cov[1], std::sqrt(var_y));
  EXPECT_EQ(cov[2], std::sqrt(var_t));

  // The cloud payload is [x, y, θ, w] per particle in ascending order.
  std::vector<std::array<double, 4>> cloud = parse_particles(lines[1]);
  ASSERT_EQ(cloud.size(), 3u);
  for (int i = 0; i < 3; ++i) {
    const size_t k = static_cast<size_t>(i);
    EXPECT_EQ(cloud[k][0], est.x()[k]);
    EXPECT_EQ(cloud[k][1], est.y()[k]);
    EXPECT_EQ(cloud[k][2], est.theta()[k]);
    EXPECT_EQ(cloud[k][3], w[i]);
  }
}

TEST(Mcl, RunOnScenarioSampleCountIsUncertainty) {
  // The branch story end-to-end (the demo's exact inputs — the scenario seed and
  // sensor injected through set(), epsilon/delta at their config defaults): t = 0
  // draws the FULL budget (800 particles, ALL distinct — nothing has resampled yet;
  // both twins survive: 427 of them face east) and t = 1 is the first KLD step.
  // From there every step's sample count IS the bound: n(t) == ceil(n_chi(k(t)))
  // exactly, where k(t) is the distinct-bin count recomputed here from the emitted
  // cloud — 33 while the belief sits in two bins, spiking when the U-turn drags the
  // cluster across bins and the ±π seam. The readout tracks ground truth to
  // centimetres. Seed 42 pins these numbers bit-for-bit in both languages.
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/corridor03_drift.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              nullptr, sc.sigma_xy, sc.sigma_theta, sc.seed);
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/filtering/mcl.yaml"));
  // The demo's injection contract — only the fields this config declares.
  params.set("seed", static_cast<int>(sc.seed));
  params.set("range_max", sc.sensor.range_max);
  params.set("sigma_range", sc.sensor.sigma_range);

  std::ostringstream os;
  TraceRecorder rec(os);
  Mcl est(params);
  EstimateResult result = est.run(episode, &rec);
  ASSERT_EQ(result.poses.size(), episode.steps.size());
  ASSERT_EQ(episode.steps.size(), 37u);

  const std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 111u);
  for (size_t k = 0; k < 37; ++k) {
    EXPECT_EQ(names[3 * k], "step_observed");
    EXPECT_EQ(names[3 * k + 1], "pose_estimated");
    EXPECT_EQ(names[3 * k + 2], "particles_updated");
  }

  std::vector<std::string> lines = split_lines(jsonl);
  ASSERT_EQ(lines.size(), 111u);
  std::vector<std::array<double, 3>> poses;
  std::vector<std::array<double, 3>> covs;
  std::vector<std::vector<std::array<double, 4>>> clouds;
  for (const std::string& line : lines) {
    if (line.find("\"pose_estimated\"") != std::string::npos) {
      poses.push_back(parse_triple(line, "\"pose\":"));
      covs.push_back(parse_triple(line, "\"cov\":"));
    } else if (line.find("\"particles_updated\"") != std::string::npos) {
      clouds.push_back(parse_particles(line));
    }
  }
  ASSERT_EQ(clouds.size(), 37u);

  // t = 0: the fixed prior budget, ALL distinct (no resampling has happened yet),
  // and BOTH headings survive the point-symmetric start cell (427 of 800 face east
  // — cos here is the same scalar libSystem cos CPython calls, on bit-identical θ).
  std::vector<int> n_t;
  for (const auto& cloud : clouds) n_t.push_back(static_cast<int>(cloud.size()));
  EXPECT_EQ(n_t[0], 800);
  std::set<std::array<double, 3>> distinct0;
  int east = 0;
  for (const std::array<double, 4>& p : clouds[0]) {
    distinct0.insert({p[0], p[1], p[2]});
    if (std::cos(p[2]) > 0.0) ++east;
  }
  EXPECT_EQ(distinct0.size(), 800u);
  EXPECT_EQ(east, 427);
  EXPECT_GT(covs[0][2], 1.0);  // heading std dev is radian-wide, not pretend-sharp

  // Every t ≥ 1: the sample count equals ceil(n_chi(k)) EXACTLY — k recomputed here
  // from the emitted cloud (bin_xy = bin_theta come from the config; every value
  // sits above n_min and below the cap, so the bound is what stopped the loop).
  const double z_q = slam::core::inv_norm_cdf(1.0 - 0.01);
  for (int t = 1; t < 37; ++t) {
    std::set<std::tuple<int, int, int>> seen;
    for (const std::array<double, 4>& p : clouds[static_cast<size_t>(t)]) {
      seen.insert({static_cast<int>(std::floor(p[0] / 0.5)),
                   static_cast<int>(std::floor(p[1] / 0.5)),
                   static_cast<int>(std::floor(p[2] / 0.17453292519943295))});
    }
    const int k = static_cast<int>(seen.size());
    EXPECT_EQ(n_t[static_cast<size_t>(t)],
              static_cast<int>(std::ceil(kld_bound(k, 0.1, z_q))));
  }

  // The pinned sequence: full budget at t = 0; the first KLD step samples until k = 41
  // bins justify 319; once the west blob dies k = 2 → 33; the U-turn re-widens (k
  // spikes as the cluster crosses bins and the ±π seam) then settles back to 33.
  std::vector<int> pinned{800, 319};
  for (int i = 0; i < 13; ++i) pinned.push_back(33);
  pinned.insert(pinned.end(), {57, 33, 67, 47});
  for (int i = 0; i < 18; ++i) pinned.push_back(33);
  EXPECT_EQ(n_t, pinned);

  double total = 0.0;
  for (size_t i = 0; i < result.poses.size(); ++i) {
    const double dx = result.poses[i].x - episode.steps[i].gt.x;
    const double dy = result.poses[i].y - episode.steps[i].gt.y;
    total += dx * dx + dy * dy;
  }
  const double ate = std::sqrt(total / static_cast<double>(result.poses.size()));
  EXPECT_GT(ate, 0.01);
  EXPECT_LT(ate, 0.1);  // measured ≈ 0.0516 — σ/√k honest floor, not a bug
  const Pose& final_pose = result.poses[result.poses.size() - 1];
  const Pose gt_final = episode.steps[episode.steps.size() - 1].gt;
  EXPECT_LT(std::abs(final_pose.x - gt_final.x), 0.06);
  EXPECT_LT(std::abs(final_pose.y - gt_final.y), 0.06);
  EXPECT_LT(std::abs(slam::core::wrap(final_pose.theta - gt_final.theta)), 0.06);
}
