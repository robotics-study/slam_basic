// particle_filter contract tests — the C++ mirror of
// python/tests/test_particle_filter.py. The same contracts, the same asymmetric
// mini-grid, and bit-identical expectations: where Python asserts exact float
// equality, EXPECT_EQ pins the same bits here.

#include <cmath>
#include <array>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/geometry.hpp"
#include "slam/core/metrics.hpp"
#include "slam/core/params.hpp"
#include "slam/core/rng.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "slam/filtering/particle_filter.hpp"
#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Rng;
using slam::core::SensorConfig;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;
using slam::filtering::ParticleFilter;

namespace {

// Free cells row-major: (0,1) → center (1.5, 1.5), (0,2) → (2.5, 1.5), (1,2) → (2.5, 0.5).
const std::vector<std::string> kMini = {"#..", "##."};

ParamSet mini_params(int n = 3, int seed = 7, double sigma_xy = 0.25, double sigma_theta = 0.1,
                     double range_max = 4.0, double sigma_range = 0.3) {
  // A tiny config for unit tests (the start cell is the one holding (1.5, 1.5)) —
  // the mirror of Python's _params().
  return ParamSet::from_yaml(slam::test::write_temp(
      "particle_filter_mini.yaml",
      ("algorithm: particle_filter\nsection: filtering\nscenarios: []\nparams:\n"
       "  - name: seed\n    type: int\n    default: " +
       std::to_string(seed) +
       "\n    description: rng seed\n"
       "  - name: n_particles\n    type: int\n    default: " +
       std::to_string(n) +
       "\n    min: 2\n    max: 4096\n    description: particle count\n"
       "  - name: x0\n    type: float\n    default: 1.5\n    description: prior cell center x\n"
       "  - name: y0\n    type: float\n    default: 1.5\n    description: prior cell center y\n"
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
// (the mirror of the Python tests reaching _x/_y/_theta/_w/_rng).
class Probe final : public ParticleFilter {
 public:
  explicit Probe(ParamSet params) : ParticleFilter(std::move(params)) {}
  using ParticleFilter::init_particles;
  using ParticleFilter::move;
  using ParticleFilter::resample_if_effective;
  using ParticleFilter::weight;
  void set_episode(const Episode& ep) { episode_ = &ep; }
  void mark_initialized() { initialized_ = true; }
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

// Parse the [x, y, θ] pose and [sx, sy, sθ] cov of one pose_estimated line. The
// key ENDS at ':' (no bracket): the loop's first ++pos consumes the array's '[',
// later ones the separators — the flat-array counterpart of parse_particles.
std::array<double, 3> parse_triple(const std::string& line, const char* key) {
  size_t pos = line.find(key);
  if (pos == std::string::npos) return {};
  pos += std::strlen(key);
  std::array<double, 3> out{};
  for (int k = 0; k < 3; ++k) {
    ++pos;  // skip '[' (first value) or ',' (later ones)
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

TEST(ParticleFilter, ConfigDefaultsAndRangeValidation) {
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filtering/particle_filter.yaml"));
  EXPECT_EQ(params.algorithm(), "particle_filter");
  EXPECT_EQ(params.section(), "filtering");
  EXPECT_EQ(params.get_int("seed"), 42);
  EXPECT_EQ(params.get_int("n_particles"), 500);
  // The prior-cell defaults are the scenario start cell center, exact binary floats.
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 4.25);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 1.75);
  EXPECT_DOUBLE_EQ(params.get_float("range_max"), 2.5);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_range"), 0.3);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_xy"), 0.05);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_theta"), 0.05);
  // Out-of-range sets raise (the demo injects the scenario sensor through set()).
  EXPECT_THROW(params.set("n_particles", 8192), std::runtime_error);
}

TEST(ParticleFilter, InitDrawsAscendingAndUniform) {
  // Three particles, seed pinned: x = ox + (col0 + u1)·res, y = oy + (h−1−row0 +
  // u2)·res, θ = u3·2π − π — three draws per particle IN ASCENDING PARTICLE ORDER
  // from the algorithm's own stream. Weights come out exactly 1/N.
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params());
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
  EXPECT_EQ(est.x(), xs);      // bit-for-bit
  EXPECT_EQ(est.y(), ys);
  EXPECT_EQ(est.theta(), ths);
  EXPECT_EQ(est.w(), (std::vector<double>{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}));  // exactly 1/N
}

TEST(ParticleFilter, MoveComposesTwistWithDrawnNoise) {
  // x ⊕ (u + ε): three gaussians per particle (two uniform draws each) drawn in
  // ascending order, composed with the arriving twist on the OLD pose.
  Probe est(mini_params(2, 11));
  est.mark_initialized();
  std::vector<Pose> old{Pose{1.5, 1.5, 0.25}, Pose{2.5, 0.5, -1.0}};
  for (const Pose& p : old) {
    est.x().push_back(p.x);
    est.y().push_back(p.y);
    est.theta().push_back(p.theta);
  }
  est.w() = {0.5, 0.5};

  Rng rng(11);
  std::vector<Pose> expect;
  for (int i = 0; i < 2; ++i) {
    double ex = rng.gaussian(0.0, 0.25);
    double ey = rng.gaussian(0.0, 0.25);
    double et = rng.gaussian(0.0, 0.1);
    expect.push_back(slam::core::pose_compose(old[i], Twist{0.5 + ex, -0.25 + ey, 0.3 + et}));
  }
  est.move(Twist{0.5, -0.25, 0.3});
  for (int i = 0; i < 2; ++i) {  // bit-for-bit — same operation order
    EXPECT_EQ(est.x()[static_cast<size_t>(i)], expect[static_cast<size_t>(i)].x);
    EXPECT_EQ(est.y()[static_cast<size_t>(i)], expect[static_cast<size_t>(i)].y);
    EXPECT_EQ(est.theta()[static_cast<size_t>(i)], expect[static_cast<size_t>(i)].theta);
  }
}

TEST(ParticleFilter, WeightIsMaxShiftedAndNormalized) {
  // Each scan point contributes −½·((r − expected)/σ)² at the particle's own pose
  // along heading + β (a miss pays the sentinel range_max + res); exps are max-shifted
  // and normalized ascending. A step with no scan weights nothing, and normalized
  // n = 2 weights can never trigger a resample (ESS ≥ 1 is never < N/2 = 1).
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid(kMini);
  Probe est(mini_params(2, 7, 0.25, 0.1, 4.0, 0.3));
  Episode episode;
  episode.grid = &grid;
  est.set_episode(episode);
  est.mark_initialized();
  est.x() = {1.5, 2.5};
  est.y() = {1.5, 0.5};
  est.theta() = {0.0, -1.0};
  est.w() = {0.25, 0.75};

  const std::vector<Point> scan{Point{1.25, 0.3}, Point{2.0, -0.75}};
  const double sentinel = 4.0 + 1.0;  // range_max + res — a miss keeps the sentinel exactly
  std::vector<double> ll;
  for (int i = 0; i < 2; ++i) {
    double acc = 0.0;
    for (const Point& z : scan) {
      double r = std::sqrt(z.x * z.x + z.y * z.y);
      double beta = std::atan2(z.y, z.x);
      std::optional<double> hit = slam::core::raycast(grid, est.x()[static_cast<size_t>(i)],
                                                      est.y()[static_cast<size_t>(i)],
                                                      est.theta()[static_cast<size_t>(i)] + beta, 4.0);
      double e = hit.has_value() ? *hit : sentinel;
      double d = (r - e) / 0.3;
      acc += (-0.5 * d) * d;
    }
    ll.push_back(acc);
  }
  double m = std::max(ll[0], ll[1]);
  double w0 = 0.25 * std::exp(ll[0] - m);
  double w1 = 0.75 * std::exp(ll[1] - m);
  double total = w0 + w1;

  est.weight(scan);
  EXPECT_EQ(est.w()[0], w0 / total);  // bit-for-bit: same exps, same ascending accumulation
  EXPECT_EQ(est.w()[1], w1 / total);

  // A step with no scan weights nothing (and cannot resample — see the comment).
  std::vector<double> before = est.w();
  Step step;
  step.t = 0;
  est.update(step, nullptr);
  EXPECT_EQ(est.w(), before);
}

TEST(ParticleFilter, ResampleTriggerAndWalk) {
  // ESS = 1/Σw² triggers strictly BELOW N/2: w = [1, 0] on n = 2 has ESS exactly
  // 1.0 and must NOT resample (bits untouched, and the rng stream consumes NOTHING).
  // A real collapse resamples with ONE draw u ∈ [0, 1/N): the walk advances while
  // u > c (strict) over the cumulative weights, copies that ancestor, and steps
  // u += 1/N after EVERY copy — then every weight is exactly 1/N.
  {
    Probe est_a(mini_params(2, 5));
    est_a.mark_initialized();
    est_a.x() = {1.0, 2.0};
    est_a.y() = {3.0, 4.0};
    est_a.theta() = {0.5, -0.5};
    est_a.w() = {1.0, 0.0};
    est_a.resample_if_effective();
    EXPECT_EQ(est_a.x(), (std::vector<double>{1.0, 2.0}));  // bit-for-bit untouched
    EXPECT_EQ(est_a.y(), (std::vector<double>{3.0, 4.0}));
    EXPECT_EQ(est_a.theta(), (std::vector<double>{0.5, -0.5}));
    EXPECT_EQ(est_a.w(), (std::vector<double>{1.0, 0.0}));
    EXPECT_EQ(est_a.draw(), Rng(5).uniform01());  // the stream never advanced
  }

  // Trigger: ESS = 1/(0.2² + 0.8²) ≈ 1.47 < 1.5. The walk's strict `>` and the
  // post-copy u += 1/N are replayed here with a second identical Rng.
  Probe est_b(mini_params(3, 5));
  est_b.mark_initialized();
  std::vector<double> xs_old{1.0, 2.0, 3.0}, ys_old{3.0, 4.0, 5.0}, ths_old{0.5, -0.5, 1.0};
  const std::vector<double> w_old{0.2, 0.8, 0.0};
  est_b.x() = xs_old;
  est_b.y() = ys_old;
  est_b.theta() = ths_old;
  est_b.w() = w_old;

  Rng rng(5);
  double u = rng.uniform01() * (1.0 / 3.0);
  std::vector<double> xs, ys, ths;
  double c = w_old[0];
  int i = 0;
  for (int j = 0; j < 3; ++j) {
    while (i < 2 && u > c) {
      i += 1;
      c += w_old[static_cast<size_t>(i)];
    }
    xs.push_back(xs_old[static_cast<size_t>(i)]);
    ys.push_back(ys_old[static_cast<size_t>(i)]);
    ths.push_back(ths_old[static_cast<size_t>(i)]);
    u += 1.0 / 3.0;  // including the final, unused increment — fixed loop shape
  }
  est_b.resample_if_effective();
  EXPECT_EQ(est_b.x(), xs);  // bit-for-bit ancestor selection
  EXPECT_EQ(est_b.y(), ys);
  EXPECT_EQ(est_b.theta(), ths);
  EXPECT_EQ(est_b.w(), (std::vector<double>{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}));
  // Exactly ONE draw was consumed: the stream's next draw is the fresh stream's second.
  EXPECT_EQ(est_b.draw(), rng.uniform01());
}

TEST(ParticleFilter, RunOnScenarioConvergesFromTheBlob) {
  // The branch story end-to-end (the demo's exact inputs — the scenario seed and
  // sensor injected through set()): t = 0 keeps BOTH headings alive (42 distinct
  // ancestors, radian-wide heading spread); the first eastward move kills the west
  // blob — by t = 1 a SINGLE ancestor survives, the covariance collapses to float
  // noise, and from there the estimate tracks ground truth to centimetres. Seed 42
  // pins these numbers bit-for-bit across languages.
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/corridor03_drift.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              nullptr, sc.sigma_xy, sc.sigma_theta, sc.seed);
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filtering/particle_filter.yaml"));
  // The demo's injection contract — only the fields this config declares.
  params.set("seed", static_cast<int>(sc.seed));
  params.set("range_max", sc.sensor.range_max);
  params.set("sigma_range", sc.sensor.sigma_range);

  std::ostringstream os;
  TraceRecorder rec(os);
  ParticleFilter est(params);
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

  // Parse the per-step payloads line by line.
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

  // t = 0: resampling fired (weights exactly uniform 1/500 again) and 42 distinct
  // ancestors survived — BOTH headings alive.
  for (const std::array<double, 4>& p : clouds[0]) EXPECT_EQ(p[3], 1.0 / 500.0);
  std::set<std::array<double, 3>> distinct0;
  int east = 0;
  for (const std::array<double, 4>& p : clouds[0]) {
    distinct0.insert({p[0], p[1], p[2]});
    if (std::cos(p[2]) > 0.0) ++east;
  }
  EXPECT_EQ(distinct0.size(), 42u);
  EXPECT_GT(east, 0);
  EXPECT_LT(east, 500);
  EXPECT_GT(covs[0][2], 1.0);  // heading std dev is radian-wide, not pretend-sharp

  // t = 1: one ancestor survives (the west blob is dead); the covariance collapses.
  for (const std::array<double, 4>& p : clouds[1]) {
    EXPECT_EQ(p[0], clouds[1][0][0]);
    EXPECT_EQ(p[1], clouds[1][0][1]);
    EXPECT_EQ(p[2], clouds[1][0][2]);
  }
  for (int k = 0; k < 3; ++k) EXPECT_LT(covs[1][k], 1e-9);

  double total = 0.0;
  for (size_t i = 0; i < result.poses.size(); ++i) {
    double dx = result.poses[i].x - episode.steps[i].gt.x;
    double dy = result.poses[i].y - episode.steps[i].gt.y;
    total += dx * dx + dy * dy;
  }
  double ate = std::sqrt(total / static_cast<double>(result.poses.size()));
  EXPECT_GT(ate, 0.01);
  EXPECT_LT(ate, 0.1);  // measured ≈ 0.0569 — σ/√k honest floor, not a bug
  const Pose& final_pose = result.poses[result.poses.size() - 1];
  const Pose gt_final = episode.steps[episode.steps.size() - 1].gt;
  EXPECT_LT(std::abs(final_pose.x - gt_final.x), 0.06);
  EXPECT_LT(std::abs(final_pose.y - gt_final.y), 0.06);
  EXPECT_LT(std::abs(slam::core::wrap(final_pose.theta - gt_final.theta)), 0.06);
}
