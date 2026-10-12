// ekf_slam contract tests — the C++ mirror of python/tests/test_ekf_slam.py. The same
// contracts and bit-identical expectations: birth is augmentation NOT an update (the
// mean is the exact back-projection so the innovation is exactly 0 by construction,
// and at t = 0 the pose covariance is still the ZERO matrix — every cross block born
// with it is ±0 exactly); landmarks enter in OBSERVATION order (ids [0,2,3,4,5] at
// t = 0, id 1 late at t = 3 as SLOT 5 — slot ≠ id is part of the page); and Σ stays
// exactly symmetric because every product computes its upper triangle and COPIES it.
//
// The scenario run pins the branch story end-to-end on office01_landmarks: with Q > 0
// the landmarks pull the pose back through the correlation blocks (ATE ≈ 0.041 against
// a raw odometry drift far larger), and σ_xy = σ_θ = 0 is the honest limit — Σ_pose
// stays EXACTLY zero forever, so every estimated pose equals ground truth BIT-FOR-BIT
// while landmarks alone still chase the observation noise floor (ATE/RPE exactly 0.0).

#include <array>
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
#include "slam/filter_based/ekf_slam.hpp"
#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::TraceRecorder;
using slam::filter_based::EkfSlam;

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

// Parse the [x, y, θ] triple of one pose_estimated / cov field.
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

// Parse every {"id":N,"x":X,"y":Y,"sx":SX,"sy":SY} entry of a landmarks_updated line.
std::vector<std::array<double, 5>> parse_estimated(const std::string& line) {
  std::vector<std::array<double, 5>> out;
  const std::string key = "{\"id\":";
  size_t pos = 0;
  while ((pos = line.find(key, pos)) != std::string::npos) {
    size_t p = pos + key.size();
    char* end = nullptr;
    const double id = std::strtod(line.c_str() + p, &end);
    const size_t xp = line.find("\"x\":", p);
    const double x = std::strtod(line.c_str() + xp + 4, &end);
    const size_t yp = line.find("\"y\":", xp);
    const double y = std::strtod(line.c_str() + yp + 4, &end);
    const size_t sxp = line.find("\"sx\":", yp);
    const double sx = std::strtod(line.c_str() + sxp + 5, &end);
    const size_t syp = line.find("\"sy\":", sxp);
    const double sy = std::strtod(line.c_str() + syp + 5, &end);
    out.push_back({id, x, y, sx, sy});
    pos = syp;  // next entry starts after this one's "sy" field
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

slam::maps::Scenario scenario() {
  return slam::maps::load_scenario(slam::test::repo_path("maps/scenarios/office01_landmarks.yaml"));
}

}  // namespace

TEST(EkfSlam, ConfigDefaultsAndRangeValidation) {
  // The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filter_based/ekf_slam.yaml"));
  EXPECT_EQ(params.algorithm(), "ekf_slam");
  EXPECT_EQ(params.section(), "filter_based");
  EXPECT_EQ(params.scenarios(), (std::vector<std::string>{"office01_landmarks"}));
  // The declared noise IS the scenario's injected noise (declared, not injected).
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 0.875);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 4.625);
  EXPECT_DOUBLE_EQ(params.get_float("theta_deg"), 0.0);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_xy"), 0.05);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_theta"), 0.035);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_range"), 0.05);
  EXPECT_DOUBLE_EQ(params.get_float("sigma_bearing"), 0.03);
  // Out-of-range sets throw (no silent clamping — the contract is a range check).
  // sigma_range/sigma_bearing keep min > 0: at t = 0 Σ is the zero matrix, so a zero
  // measurement σ would make the FIRST birth's W = HpΣHᵀ + R singular.
  EXPECT_THROW(params.set("sigma_xy", -0.1), std::runtime_error);   // below min 0.0
  EXPECT_THROW(params.set("sigma_xy", 2.0), std::runtime_error);    // above max 1.0
  EXPECT_THROW(params.set("sigma_range", 0.0), std::runtime_error);  // below min 0.001
}

TEST(EkfSlam, BirthAtT0IsBackprojectionWithZeroCrossBlocks) {
  // t = 0 is the whole lesson in one step. The gauge pose is KNOWN (cov emitted as
  // exactly [0, 0, 0]), so five landmarks are born through the improper-prior limit
  // with cross-covariance exactly ±0 — and their slot order IS their observation
  // order: ids [0,2,3,4,5] at t = 0 (id 1 is not yet visible), then id 1 arrives late
  // at t = 3 as SLOT 5. The pinned sx/sy are sqrt(diag(B⁻¹RB⁻ᵀ)) — pure measurement
  // noise rotated into world frame, no pose contribution.
  slam::maps::Scenario sc = scenario();
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filter_based/ekf_slam.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              &sc.landmarks, sc.sigma_xy, sc.sigma_theta, sc.seed);

  std::ostringstream os;
  TraceRecorder rec(os);
  EkfSlam est(params);
  est.run(episode, &rec);

  const std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 75u);
  for (size_t k = 0; k < 25; ++k) {
    EXPECT_EQ(names[3 * k], "step_observed");
    EXPECT_EQ(names[3 * k + 1], "pose_estimated");
    EXPECT_EQ(names[3 * k + 2], "landmarks_updated");
  }
  std::vector<std::string> lines = split_lines(jsonl);
  ASSERT_EQ(lines.size(), 75u);

  // t = 0: the declared gauge, emitted verbatim — covariance EXACTLY zero.
  std::array<double, 3> p0 = parse_triple(lines[1], "\"pose\":");
  EXPECT_EQ(p0[0], 0.875);
  EXPECT_EQ(p0[1], 4.625);
  EXPECT_EQ(p0[2], 0.0);
  std::array<double, 3> c0 = parse_triple(lines[1], "\"cov\":");
  EXPECT_EQ(c0[0], 0.0);
  EXPECT_EQ(c0[1], 0.0);
  EXPECT_EQ(c0[2], 0.0);

  // Slot order = observation order; id 1 is not visible yet at t = 0.
  std::vector<std::array<double, 5>> lm0 = parse_estimated(lines[2]);
  ASSERT_EQ(lm0.size(), 5u);
  const std::array<int, 5> ids0{0, 2, 3, 4, 5};
  for (size_t k = 0; k < 5; ++k) EXPECT_EQ(lm0[k][0], static_cast<double>(ids0[k]));

  // id 1 is born at t = 3 — as SLOT 5 (the sixth entry): slot ≠ id, by contract.
  std::vector<std::array<double, 5>> lm3 = parse_estimated(lines[3 * 3 + 2]);
  ASSERT_EQ(lm3.size(), 6u);
  const std::array<int, 6> ids3{0, 2, 3, 4, 5, 1};
  for (size_t k = 0; k < 6; ++k) EXPECT_EQ(lm3[k][0], static_cast<double>(ids3[k]));
}

TEST(EkfSlam, NoiseFreeOdometryPinsThePoseBitForBit) {
  // The honest limit, pinned: declare Q = 0 (σ_xy = σ_θ = 0) and feed noise-free
  // odometry. Σ_pose starts at zero and every product that could move it multiplies
  // those zeros — so the pose rows of K stay exactly ±0 and NO observation can ever
  // correct the pose: every estimated pose equals ground truth bit-for-bit, and the
  // emitted cov stays [0, 0, 0] at EVERY step. The landmarks still see noise
  // (σ_range/σ_bearing unchanged), so they alone chase the observation floor —
  // landmark_rmse ≈ 0.031 is that floor; ATE and RPE are EXACTLY zero.
  slam::maps::Scenario sc = scenario();
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filter_based/ekf_slam.yaml"));
  params.set("sigma_xy", 0.0);
  params.set("sigma_theta", 0.0);
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              &sc.landmarks, 0.0, 0.0, sc.seed);
  EstimateResult result = EkfSlam(params).run(episode, nullptr);
  ASSERT_EQ(result.poses.size(), episode.steps.size());
  for (size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_EQ(result.poses[i].x, episode.steps[i].gt.x);
    EXPECT_EQ(result.poses[i].y, episode.steps[i].gt.y);
    EXPECT_EQ(result.poses[i].theta, episode.steps[i].gt.theta);
  }
  std::ostringstream os;
  TraceRecorder rec(os);
  EkfSlam est(params);
  est.run(episode, &rec);
  for (const std::string& line : split_lines(os.str())) {
    if (line.find("\"pose_estimated\"") == std::string::npos) continue;
    std::array<double, 3> cov = parse_triple(line, "\"cov\":");
    EXPECT_EQ(cov[0], 0.0);
    EXPECT_EQ(cov[1], 0.0);
    EXPECT_EQ(cov[2], 0.0);
  }
  const std::map<std::string, double> metrics = slam::core::evaluate(result, episode);
  EXPECT_EQ(metrics.at("ate_rmse"), 0.0);
  EXPECT_EQ(metrics.at("rpe_rmse"), 0.0);
  EXPECT_EQ(metrics.at("landmark_rmse"), 0.031009034055468968);
}

TEST(EkfSlam, RunPinnedMetricsAndFinalState) {
  // The branch story end-to-end at the declared defaults: five landmarks at t = 0,
  // id 1 late at t = 3, ids 7/9 only after the door (t = 20/23) — and once a landmark
  // is re-observed its correlation blocks pull the pose back (ATE ≈ 0.041, RPE ≈
  // 0.053 against σ_xy = 0.05 raw odometry). The final estimates sit in SLOT order
  // ([0,2,3,4,5,1,6,8,7,9] — birth order, not id order); id 0's sx shrinks from its
  // birth value (0.0695… at t = 0) to 0.0467… through re-observations. Every pinned
  // double below is bit-exact in BOTH languages.
  slam::maps::Scenario sc = scenario();
  ParamSet params =
      ParamSet::from_yaml(slam::test::repo_path("configs/filter_based/ekf_slam.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              &sc.landmarks, sc.sigma_xy, sc.sigma_theta, sc.seed);
  EstimateResult result = EkfSlam(params).run(episode, nullptr);
  ASSERT_EQ(result.landmarks.size(), 10u);
  const std::array<int, 10> ids{0, 2, 3, 4, 5, 1, 6, 8, 7, 9};
  const std::array<std::array<double, 2>, 10> pins{
      {{1.4200996828569024, 6.886174368712869},   {1.3803222685724423, 6.359958406045323},
       {2.3619726977007933, 6.368831565718878},   {6.618930570332339, 5.37503286465251},
       {6.643192860733659, 3.618988650677379},    {2.36488622326796, 6.8334875911877235},
       {8.415522255648076, 2.8835850551259687},   {8.384155139907442, 1.931969036455171},
       {10.432688731991588, 2.9304085077584583},  {10.465177740635248, 1.8892355645801564}}};
  for (size_t k = 0; k < 10; ++k) {
    EXPECT_EQ(result.landmarks[k].id, ids[k]);
    EXPECT_EQ(result.landmarks[k].x, pins[k][0]);
    EXPECT_EQ(result.landmarks[k].y, pins[k][1]);
  }
  EXPECT_TRUE(result.landmarks[0].has_sigma);
  EXPECT_EQ(result.landmarks[0].sx, 0.0467706341056405);
  EXPECT_EQ(result.landmarks[0].sy, 0.035023119339539416);
  const std::map<std::string, double> metrics = slam::core::evaluate(result, episode);
  EXPECT_EQ(metrics.at("ate_rmse"), 0.040788530613740936);
  EXPECT_EQ(metrics.at("rpe_rmse"), 0.05301454702803877);
  EXPECT_EQ(metrics.at("landmark_rmse"), 0.04962713080932249);
}
