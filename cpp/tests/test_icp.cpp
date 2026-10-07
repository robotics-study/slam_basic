// icp contract tests — the C++ mirror of python/tests/test_icp.py. The same
// contracts and bit-identical expectations: a single kept pair reproduces its
// displacement EXACTLY (the closed form is exact for translation); an equidistant
// source point keeps the LOWER target index (strict `<` scan); truncation is the
// whole story — one outlier pair beyond d_max silently drags the least-squares
// mean toward garbage; and a step whose every pair exceeds d_max returns the
// identity twist exactly. The rotation case pins the closed form to the exact
// doubles of −R(θ)a / θ constructed from the same formulas, and iteration is
// pinned by max_iters=1 vs convergence.
//
// The scenario run pins the branch story end-to-end on room01_straight: t = 0
// emits the DECLARED gauge pose exactly (1.25, 1.75, 0) — registration measures
// relative motion only — and every later step composes one recovered twist. The
// estimator never reads Step.odom: rebuilding the episode with sigma_xy = 5.0
// changes not one bit of the result. ATE/RPE are pinned to exact doubles (seed 42
// pins them in both languages).

#include <cmath>
#include <array>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"
#include "slam/core/params.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "slam/maps/loader.hpp"
#include "slam/registration/icp.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;
using slam::registration::Icp;

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

TEST(Icp, ConfigDefaultsAndRangeValidation) {
  // The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/registration/icp.yaml"));
  EXPECT_EQ(params.algorithm(), "icp");
  EXPECT_EQ(params.section(), "registration");
  EXPECT_EQ(params.scenarios(), (std::vector<std::string>{"room01_straight"}));
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 1.25);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 1.75);
  EXPECT_DOUBLE_EQ(params.get_float("theta_deg"), 0.0);
  EXPECT_DOUBLE_EQ(params.get_float("d_max"), 1.0);
  EXPECT_DOUBLE_EQ(params.get_float("eps"), 1e-9);
  EXPECT_EQ(params.get_int("max_iters"), 64);
  // Out-of-range sets throw (no silent clamping — the contract is a range check).
  EXPECT_THROW(params.set("d_max", 0.2), std::runtime_error);   // below min 0.25
  EXPECT_THROW(params.set("eps", 0.2), std::runtime_error);     // above max 0.1
  EXPECT_THROW(params.set("max_iters", 0), std::runtime_error);  // below min 1
}

TEST(Icp, SinglePairTranslationIsExact) {
  // One kept pair whose displacement is inside d_max: the closed form returns that
  // displacement EXACTLY (centroids of singletons, atan2(0, ·) = 0 exactly), and
  // one iteration already sits at the fixed point — max_iters=1 changes nothing.
  Twist got = slam::registration::icp_step({Point{1.0, 0.5}}, {Point{1.5, 0.5}}, 1.0, 1e-9, 64);
  EXPECT_EQ(got.dx, 0.5);
  EXPECT_EQ(got.dy, 0.0);
  EXPECT_EQ(got.dtheta, 0.0);
  Twist once = slam::registration::icp_step({Point{1.0, 0.5}}, {Point{1.5, 0.5}}, 1.0, 1e-9, 1);
  EXPECT_EQ(once.dx, got.dx);
  EXPECT_EQ(once.dy, got.dy);
  EXPECT_EQ(once.dtheta, got.dtheta);
}

TEST(Icp, TieKeepsTheLowerIndex) {
  // Source (1, 0) sits EXACTLY between targets (0, 0) and (2, 0): squared distance
  // 1.0 to both, and the ascending strict-`<` scan keeps index 0 — so the recovered
  // twist is −x, not +x. A `<=` would have flipped this result.
  Twist got = slam::registration::icp_step({Point{1.0, 0.0}}, {Point{0.0, 0.0}, Point{2.0, 0.0}},
                                           1.0, 1e-9, 64);
  EXPECT_EQ(got.dx, -1.0);
  EXPECT_EQ(got.dy, 0.0);
  EXPECT_EQ(got.dtheta, 0.0);
}

TEST(Icp, TruncationIsTheWholeStory) {
  // The outlier demo: source {(1,.5),(9,.5)} against target {(1.25,.5),(2,.5)}.
  // With d_max = 1 the far pair (squared distance 49 > 1) is truncated and the
  // solve returns the kept pair's displacement EXACTLY — +0.25. Widen d_max to 20
  // and the wrong pair survives: both pairs are kept, and the least-squares mean
  // of the two contradictory displacements lands at −3.375 — silently, confidently
  // wrong. This is why truncation exists.
  const std::vector<Point> src{Point{1.0, 0.5}, Point{9.0, 0.5}};
  const std::vector<Point> tgt{Point{1.25, 0.5}, Point{2.0, 0.5}};
  Twist kept = slam::registration::icp_step(src, tgt, 1.0, 1e-9, 64);
  EXPECT_EQ(kept.dx, 0.25);
  EXPECT_EQ(kept.dy, 0.0);
  EXPECT_EQ(kept.dtheta, 0.0);
  Twist both = slam::registration::icp_step(src, tgt, 20.0, 1e-9, 64);
  EXPECT_EQ(both.dx, -3.375);
  EXPECT_EQ(both.dy, 0.0);
  EXPECT_EQ(both.dtheta, 0.0);
}

TEST(Icp, DegenerateInputsAreIdentity) {
  // Empty source or target, and a step whose every pair exceeds d_max, all return
  // the identity twist exactly — no information, no motion (documented degenerate).
  Twist id{0.0, 0.0, 0.0};
  Twist empty_source = slam::registration::icp_step({}, {Point{5.0, 5.0}}, 1.0, 1e-9, 64);
  EXPECT_EQ(empty_source.dx, id.dx);
  EXPECT_EQ(empty_source.dy, id.dy);
  EXPECT_EQ(empty_source.dtheta, id.dtheta);
  Twist empty_target = slam::registration::icp_step({Point{5.0, 5.0}}, {}, 1.0, 1e-9, 64);
  EXPECT_EQ(empty_target.dx, id.dx);
  EXPECT_EQ(empty_target.dy, id.dy);
  EXPECT_EQ(empty_target.dtheta, id.dtheta);
  Twist truncated = slam::registration::icp_step({Point{5.0, 5.0}}, {Point{0.0, 0.0}}, 1.0, 1e-9,
                                                 64);
  EXPECT_EQ(truncated.dx, id.dx);
  EXPECT_EQ(truncated.dy, id.dy);
  EXPECT_EQ(truncated.dtheta, id.dtheta);
}

TEST(Icp, RotationRecoversAndConverges) {
  // Two points rotated by θ = 0.1 (constructed with the same cos/sin doubles the
  // implementation uses — libm-routed scalar sin/cos, the CPython pair): the closed
  // form returns the constructing twist — dθ lands on 0.1 to the last bit and the
  // translation on −R(θ)a exactly. The displacement is small against the point
  // spacing, so identity already pairs correctly and ONE iteration sits at the
  // fixed point (max_iters=1 pins equal to convergence).
  const double th = 0.1;
  const double c = slam::core::libm_cos(th);
  const double s = slam::core::libm_sin(th);
  const Point p1{0.0, -5.0}, p2{0.0, 5.0};
  const Point a{0.25, -0.75};
  const Point q1{c * p1.x + s * p1.y + a.x, -s * p1.x + c * p1.y + a.y};
  const Point q2{c * p2.x + s * p2.y + a.x, -s * p2.x + c * p2.y + a.y};
  Twist got = slam::registration::icp_step({q1, q2}, {p1, p2}, 5.0, 1e-9, 64);
  EXPECT_EQ(got.dx, -0.32362610380462753);
  EXPECT_EQ(got.dy, 0.7212947697968124);
  EXPECT_EQ(got.dtheta, 0.09999999999999999);
  // Correct pairing from the first iteration: one pass already IS the fixed point.
  Twist once = slam::registration::icp_step({q1, q2}, {p1, p2}, 5.0, 1e-9, 1);
  EXPECT_EQ(once.dx, got.dx);
  EXPECT_EQ(once.dy, got.dy);
  EXPECT_EQ(once.dtheta, got.dtheta);
}

TEST(Icp, CaptureBasinIsAFixedPointToo) {
  // The honest limit: points closer than the displacement alias under identity —
  // both source points pair to the SAME target point and the wrong fixed point
  // wins. Displacement +0.6 on spacing 1.0 converges (exactly, pinned) to +0.1 =
  // −(−0.6+0.4)/2 — the mean of the two aliased displacements. ICP is a local
  // method; d_max and the capture basin are its honest limits, not bugs.
  Twist got = slam::registration::icp_step({Point{-0.6, 0.0}, Point{0.4, 0.0}},
                                           {Point{0.0, 0.0}, Point{1.0, 0.0}}, 5.0, 1e-9, 64);
  EXPECT_EQ(got.dx, 0.09999999999999998);
  EXPECT_EQ(got.dy, 0.0);
  EXPECT_EQ(got.dtheta, 0.0);
}

TEST(Icp, UpdateEmitsGaugeThenPosesWithoutCov) {
  // t = 0 adopts the declared gauge pose and emits it WITHOUT cov (ICP carries no
  // uncertainty model); a scan-less step carries the estimate forward untouched.
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/registration/icp.yaml"));
  Icp est(params);
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
  EXPECT_EQ(p0[0], 1.25);
  EXPECT_EQ(p0[1], 1.75);
  EXPECT_EQ(p0[2], 0.0);
  EXPECT_EQ(p1[0], 1.25);  // scan-less step: identity carried
  EXPECT_EQ(p1[1], 1.75);
  EXPECT_EQ(p1[2], 0.0);
  EXPECT_EQ(lines[0].find("\"cov\""), std::string::npos);
  EXPECT_EQ(lines[1].find("\"cov\""), std::string::npos);
}

TEST(Icp, RunOnScenarioIsRegistrationNotOdometry) {
  // The branch story end-to-end on room01_straight (the demo's exact inputs — no
  // sensor params are declared, so the demo injects NOTHING): the gauge pose is
  // emitted verbatim at t = 0, every later step composes one recovered twist, and
  // Step.odom NEVER matters — rebuilding the episode with sigma_xy = 5.0 changes
  // not one bit of any estimate. ATE/RPE pinned to exact doubles (seed 42 pins
  // them in both languages).
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/room01_straight.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              nullptr, sc.sigma_xy, sc.sigma_theta, sc.seed);
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/registration/icp.yaml"));

  std::ostringstream os;
  TraceRecorder rec(os);
  Icp est(params);
  EstimateResult result = est.run(episode, &rec);
  ASSERT_EQ(result.poses.size(), episode.steps.size());
  ASSERT_EQ(episode.steps.size(), 9u);

  const std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 18u);
  for (size_t k = 0; k < 9; ++k) {
    EXPECT_EQ(names[2 * k], "step_observed");
    EXPECT_EQ(names[2 * k + 1], "pose_estimated");
  }
  std::vector<std::string> lines = split_lines(jsonl);
  ASSERT_EQ(lines.size(), 18u);

  // t = 0 is the declared gauge, bit-for-bit; odom arrives at t >= 1 (σ = 0 here —
  // the EXACT command, which the estimator never reads anyway).
  EXPECT_EQ(lines[0].find("\"odom\""), std::string::npos);
  std::array<double, 3> p0 = parse_triple(lines[1], "\"pose\":");
  EXPECT_EQ(p0[0], 1.25);
  EXPECT_EQ(p0[1], 1.75);
  EXPECT_EQ(p0[2], 0.0);
  std::array<double, 3> odom1 = parse_triple(lines[2], "\"odom\":");
  EXPECT_EQ(odom1[0], 0.25);
  EXPECT_EQ(odom1[1], 0.0);
  EXPECT_EQ(odom1[2], 0.0);

  // ATE over the whole run, pinned to the exact double (sub-cell by construction —
  // res 0.5). The sum order mirrors Python: ascending dx²+dy² accumulation.
  double total = 0.0;
  for (size_t i = 0; i < result.poses.size(); ++i) {
    const double dx = result.poses[i].x - episode.steps[i].gt.x;
    const double dy = result.poses[i].y - episode.steps[i].gt.y;
    total += dx * dx + dy * dy;
  }
  const double ate = std::sqrt(total / static_cast<double>(result.poses.size()));
  EXPECT_EQ(ate, 0.020964123524742473);

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
            0.01952285461299684);

  // The estimator never reads Step.odom: a σ that would swamp any odometry model
  // changes NOTHING here, bit for bit.
  Episode noisy = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                            nullptr, 5.0, 5.0, sc.seed);
  EstimateResult result_noisy = Icp(params).run(noisy, nullptr);
  ASSERT_EQ(result_noisy.poses.size(), result.poses.size());
  for (size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_EQ(result_noisy.poses[i].x, result.poses[i].x);
    EXPECT_EQ(result_noisy.poses[i].y, result.poses[i].y);
    EXPECT_EQ(result_noisy.poses[i].theta, result.poses[i].theta);
  }
}
