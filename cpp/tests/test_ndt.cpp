// ndt contract tests — the C++ mirror of python/tests/test_ndt.py. The same
// contracts and bit-identical expectations: an empty side or a degenerate cluster
// (n < 3, scatter determinant exactly 0.0) carries no field and returns the identity
// twist exactly; one four-point cluster shifted by −0.25 does NOT return +0.25 — its
// bumps sit closer together than the blob is wide, the score has several stationary
// points, and Newton-with-backtracking slides to the nearest one (pinned to the last
// bit: dx = 0.4000…, not 0.25) — NDT's capture basin, pinned like ICP pins its wrong
// fixed point. The scenario run pins the branch story end-to-end on room02_diagonal:
// t = 0 emits the DECLARED gauge pose exactly (1.75, 1.75, 0) and every later step
// composes one recovered twist; Step.odom NEVER matters — rebuilding the episode with
// sigma_xy = 5.0 changes not one bit of the result. ATE/RPE pinned to exact doubles.

#include <cmath>
#include <array>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/geometry.hpp"
#include "slam/core/params.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "slam/maps/loader.hpp"
#include "slam/registration/ndt.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;
using slam::registration::Ndt;

namespace {

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

// Parse the [x, y, θ] triple of one pose_estimated / odom field.
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

TEST(Ndt, ConfigDefaultsAndRangeValidation) {
  // The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/registration/ndt.yaml"));
  EXPECT_EQ(params.algorithm(), "ndt");
  EXPECT_EQ(params.section(), "registration");
  EXPECT_EQ(params.scenarios(), (std::vector<std::string>{"room02_diagonal"}));
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 1.75);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 1.75);
  EXPECT_DOUBLE_EQ(params.get_float("theta_deg"), 0.0);
  EXPECT_DOUBLE_EQ(params.get_float("cell_size"), 0.5);
  EXPECT_DOUBLE_EQ(params.get_float("lam0"), 1e-6);
  EXPECT_DOUBLE_EQ(params.get_float("eps"), 1e-9);
  EXPECT_EQ(params.get_int("max_iters"), 64);
  // Out-of-range sets throw (no silent clamping — the contract is a range check).
  EXPECT_THROW(params.set("cell_size", 0.1), std::runtime_error);  // below min 0.25
  EXPECT_THROW(params.set("lam0", 2.0), std::runtime_error);       // above max 1.0
  EXPECT_THROW(params.set("eps", 0.2), std::runtime_error);        // above max 0.1
  EXPECT_THROW(params.set("max_iters", 0), std::runtime_error);    // below min 1
}

TEST(Ndt, DegenerateInputsAreIdentity) {
  // Empty source or target returns the identity twist exactly — no information, no
  // motion. n < 3 carries NO distribution and collinear points have scatter det
  // EXACTLY 0.0 — with no cell holding a field the score is constant, g = 0 exactly,
  // and the Cramer step is 0/det = 0 at every iteration.
  Twist id{0.0, 0.0, 0.0};
  Twist empty_source = slam::registration::ndt_step({}, {Point{1.0, 1.0}}, 1.0, 1e-6, 1e-9, 64);
  EXPECT_EQ(empty_source.dx, id.dx);
  EXPECT_EQ(empty_source.dy, id.dy);
  EXPECT_EQ(empty_source.dtheta, id.dtheta);
  Twist empty_target = slam::registration::ndt_step({Point{5.0, 5.0}}, {}, 1.0, 1e-6, 1e-9, 64);
  EXPECT_EQ(empty_target.dx, id.dx);
  EXPECT_EQ(empty_target.dy, id.dy);
  EXPECT_EQ(empty_target.dtheta, id.dtheta);
  // n < 3: a single point can carry no distribution — the score is constant.
  Twist few = slam::registration::ndt_step({Point{5.0, 5.0}}, {Point{2.0, 1.0}}, 1.0, 1e-6, 1e-9,
                                           64);
  EXPECT_EQ(few.dx, id.dx);
  EXPECT_EQ(few.dy, id.dy);
  EXPECT_EQ(few.dtheta, id.dtheta);
  // Collinear target: sxx = 0 exactly (all x equal), so det is exactly 0.0 — the
  // cell carries no distribution and the solve stays identity.
  Twist collinear = slam::registration::ndt_step({Point{2.0, 1.0}, Point{2.0, 1.5}, Point{2.0, 2.0}},
                                                 {Point{3.0, 1.0}, Point{3.0, 1.5}, Point{3.0, 2.0}},
                                                 1.0, 1e-6, 1e-9, 64);
  EXPECT_EQ(collinear.dx, id.dx);
  EXPECT_EQ(collinear.dy, id.dy);
  EXPECT_EQ(collinear.dtheta, id.dtheta);
}

TEST(Ndt, OneClusterIsACaptureBasinNotAnExactShift) {
  // The honest limit, pinned. One cluster of four points (μ = (2.5, 1.5), sxy = 0
  // exactly); the source is the same cross shifted −0.25 in x. A point-to-point rule
  // would return +0.25 EXACTLY — NDT cannot: its score is a SUM of four Gaussian
  // bumps whose centers (μ − q_i) sit at (0.5, 0), (0, 0) and (0.25, ±0.25), closer
  // together than the blob is wide (σ_eff = √0.125 ≈ 0.354). The score has several
  // stationary points; Newton-with-backtracking slides to the nearest one — pinned at
  // dx = 0.4000…, NOT +0.25 — and even a single iteration already lands at 0.3933….
  const std::vector<Point> tgt{Point{2.25, 1.5}, Point{2.75, 1.5}, Point{2.5, 1.25},
                               Point{2.5, 1.75}};
  const std::vector<Point> src{Point{2.0, 1.5}, Point{2.5, 1.5}, Point{2.25, 1.25},
                               Point{2.25, 1.75}};
  Twist got = slam::registration::ndt_step(src, tgt, 1.0, 1e-6, 1e-9, 64);
  EXPECT_EQ(got.dx, 0.4000430730746629);
  EXPECT_EQ(got.dy, -0.20372559558711778);
  EXPECT_EQ(got.dtheta, 0.09359947373033549);
  Twist once = slam::registration::ndt_step(src, tgt, 1.0, 1e-6, 1e-9, 1);
  EXPECT_EQ(once.dx, 0.3933349995237937);
  EXPECT_EQ(once.dy, 1.5137416256439681e-07);
  EXPECT_EQ(once.dtheta, -6.718066401189245e-08);
}

TEST(Ndt, UpdateEmitsGaugeThenPosesWithoutCov) {
  // t = 0 adopts the declared gauge pose and emits it WITHOUT cov (NDT carries no
  // uncertainty model); a scan-less step carries the estimate forward untouched.
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/registration/ndt.yaml"));
  Ndt est(params);
  std::ostringstream os;
  TraceRecorder rec(os);
  Step s0;
  s0.t = 0;
  s0.gt = Pose{0.0, 0.0, 0.0};
  est.update(s0, &rec);
  Step s1;
  s1.t = 1;
  s1.gt = Pose{9.0, 9.0, 0.0};
  s1.has_odom = true;
  s1.odom = Twist{0.25, 0.0, 0.0};
  est.update(s1, &rec);

  std::vector<std::string> lines = split_lines(os.str());
  ASSERT_EQ(lines.size(), 2u);
  std::vector<std::string> names = event_names(os.str());
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "pose_estimated");
  EXPECT_EQ(names[1], "pose_estimated");
  // The declared gauge, bit-for-bit — and no cov field anywhere (no uncertainty model).
  std::array<double, 3> p0 = parse_triple(lines[0], "\"pose\":");
  std::array<double, 3> p1 = parse_triple(lines[1], "\"pose\":");
  EXPECT_EQ(p0[0], 1.75);
  EXPECT_EQ(p0[1], 1.75);
  EXPECT_EQ(p0[2], 0.0);
  EXPECT_EQ(p1[0], 1.75);  // scan-less step: identity carried
  EXPECT_EQ(p1[1], 1.75);
  EXPECT_EQ(p1[2], 0.0);
  EXPECT_EQ(lines[0].find("\"cov\""), std::string::npos);
  EXPECT_EQ(lines[1].find("\"cov\""), std::string::npos);
}

TEST(Ndt, RunOnScenarioIsRegistrationNotOdometry) {
  // The branch story end-to-end on room02_diagonal (the demo's exact inputs — no
  // sensor params are declared, so the demo injects NOTHING): the gauge pose is
  // emitted verbatim at t = 0, every later step composes one recovered twist, and
  // Step.odom NEVER matters — rebuilding the episode with sigma_xy = 5.0 changes not
  // one bit of any estimate. ATE/RPE pinned to exact doubles (seed 42 pins them in
  // both languages); the four composed poses pin every bit between them.
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/room02_diagonal.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              nullptr, sc.sigma_xy, sc.sigma_theta, sc.seed);
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/registration/ndt.yaml"));

  std::ostringstream os;
  TraceRecorder rec(os);
  Ndt est(params);
  EstimateResult result = est.run(episode, &rec);
  ASSERT_EQ(result.poses.size(), episode.steps.size());
  ASSERT_EQ(episode.steps.size(), 5u);

  const std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 10u);
  for (size_t k = 0; k < 5; ++k) {
    EXPECT_EQ(names[2 * k], "step_observed");
    EXPECT_EQ(names[2 * k + 1], "pose_estimated");
  }
  std::vector<std::string> lines = split_lines(jsonl);
  ASSERT_EQ(lines.size(), 10u);

  // t = 0 is the declared gauge, bit-for-bit; odom arrives at t >= 1 (σ = 0 here —
  // the EXACT command, which the estimator never reads anyway).
  EXPECT_EQ(lines[0].find("\"odom\""), std::string::npos);
  std::array<double, 3> p0 = parse_triple(lines[1], "\"pose\":");
  EXPECT_EQ(p0[0], 1.75);
  EXPECT_EQ(p0[1], 1.75);
  EXPECT_EQ(p0[2], 0.0);
  std::array<double, 3> odom1 = parse_triple(lines[2], "\"odom\":");
  EXPECT_EQ(odom1[0], 0.25);
  EXPECT_EQ(odom1[1], 0.0);
  EXPECT_EQ(odom1[2], 0.0);

  // Every composed pose pinned to the exact double (the twists ride inside them).
  const std::array<std::array<double, 3>, 5> pins{
      {{1.75, 1.75, 0.0},
       {1.9938614253006162, 1.7485465123669501, 0.0019442557789731546},
       {2.2148091162733996, 1.8312591245115464, -0.05973331507934694},
       {2.450363965729009, 1.789030019919816, -0.041422709877410674},
       {2.707779875481148, 1.7391449950498663, -0.010721669321791652}}};
  for (size_t k = 0; k < 5; ++k) {
    std::array<double, 3> p = parse_triple(lines[2 * k + 1], "\"pose\":");
    EXPECT_EQ(p[0], pins[k][0]);
    EXPECT_EQ(p[1], pins[k][1]);
    EXPECT_EQ(p[2], pins[k][2]);
  }

  // ATE over the whole run, pinned to the exact double (the staircase demo's ATE).
  double total = 0.0;
  for (size_t i = 0; i < result.poses.size(); ++i) {
    const double dx = result.poses[i].x - episode.steps[i].gt.x;
    const double dy = result.poses[i].y - episode.steps[i].gt.y;
    total += dx * dx + dy * dy;
  }
  const double ate = std::sqrt(total / static_cast<double>(result.poses.size()));
  EXPECT_EQ(ate, 0.05247606452360631);

  // RPE: per-step pose_minus errors (componentwise — dθ is NOT wrapped), pinned.
  double total_r = 0.0;
  for (size_t t = 1; t < result.poses.size(); ++t) {
    const Twist e_hat = slam::core::pose_minus(result.poses[t - 1], result.poses[t]);
    const Twist e_gt = slam::core::pose_minus(episode.steps[t - 1].gt, episode.steps[t].gt);
    const double dx = e_hat.dx - e_gt.dx;
    const double dy = e_hat.dy - e_gt.dy;
    const double dt = e_hat.dtheta - e_gt.dtheta;
    total_r += dx * dx + dy * dy + dt * dt;
  }
  EXPECT_EQ(std::sqrt(total_r / static_cast<double>(result.poses.size() - 1)),
            0.06183302900652463);

  // The estimator never reads Step.odom: a σ that would swamp any odometry model
  // changes NOTHING here, bit for bit.
  Episode noisy = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                            nullptr, 5.0, 5.0, sc.seed);
  EstimateResult result_noisy = Ndt(params).run(noisy, nullptr);
  ASSERT_EQ(result_noisy.poses.size(), result.poses.size());
  for (size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_EQ(result_noisy.poses[i].x, result.poses[i].x);
    EXPECT_EQ(result_noisy.poses[i].y, result.poses[i].y);
    EXPECT_EQ(result_noisy.poses[i].theta, result.poses[i].theta);
  }
}
