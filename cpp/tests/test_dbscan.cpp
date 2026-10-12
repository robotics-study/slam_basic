// dbscan contract tests — the C++ mirror of python/tests/test_dbscan.py. The same
// contracts and bit-identical expectations: a CLOSED ball (a point exactly eps away
// is a neighbor), min_pts counting itself, noise that is sticky by visit order (a
// point visited while sparse stays noise even when it sits inside eps of a cluster
// seeded later — on static data this equals the paper's pseudocode exactly), and ids
// that follow DISCOVERY order (ascending seed index), not spatial order.
//
// The scenario run pins the branch story end-to-end on blobs01_line: three single-cell
// blobs become three clusters whose ids are the GT list order, every estimated pose is
// ground truth EXACTLY (this branch measures no motion — σ = 0 odometry composed onto
// the declared gauge lands bit-for-bit on gt, so ATE/RPE are exactly 0.0), and unlike
// the registration branch this estimator DOES consume Step.odom: rebuilding the
// episode with sigma_xy = sigma_theta = 5.0 shatters the world-frame accumulation into
// 33 scattered clusters and moves every pinned number (pinned below, honestly). The
// residual landmark error is the half-cell itself: a beam stops at a blob's face, so
// the honest feature centroid sits on the face (B's estimate lands at x = 5.9999…
// against the cell CENTER 6.25), and landmark_rmse ≈ 0.2389 is that half-cell bias
// plus σ_range noise — measured, pinned.

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
#include "slam/features/dbscan.hpp"
#include "slam/maps/loader.hpp"
#include "test_util.hpp"

using slam::core::Episode;
using slam::core::EstimateResult;
using slam::core::ParamSet;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::features::Dbscan;
using slam::features::dbscan_cluster;

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

// Parse every {"id":N,"x":X,"y":Y} entry of a landmarks_updated line as [id, x, y].
std::vector<std::array<double, 3>> parse_estimated(const std::string& line) {
  std::vector<std::array<double, 3>> out;
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
    out.push_back({id, x, y});
    pos = yp;  // next entry starts after this one's "y" field
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

TEST(Dbscan, ConfigDefaultsAndRangeValidation) {
  // The gauge defaults are the scenario start pose — exact binary floats, θ = 0.
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/features/dbscan.yaml"));
  EXPECT_EQ(params.algorithm(), "dbscan");
  EXPECT_EQ(params.section(), "features");
  EXPECT_EQ(params.scenarios(), (std::vector<std::string>{"blobs01_line"}));
  EXPECT_DOUBLE_EQ(params.get_float("x0"), 0.75);
  EXPECT_DOUBLE_EQ(params.get_float("y0"), 1.75);
  EXPECT_DOUBLE_EQ(params.get_float("theta_deg"), 0.0);
  EXPECT_DOUBLE_EQ(params.get_float("eps"), 0.2);
  EXPECT_EQ(params.get_int("min_pts"), 4);
  // Out-of-range sets throw (no silent clamping — the contract is a range check).
  EXPECT_THROW(params.set("eps", 0.04), std::runtime_error);   // below min 0.05
  EXPECT_THROW(params.set("eps", 2.5), std::runtime_error);    // above max 2.0
  EXPECT_THROW(params.set("min_pts", 0), std::runtime_error);  // below min 1
}

TEST(Dbscan, ClosedBallAndSelfCounting) {
  // A point exactly eps away IS a neighbor (dx² + dy² ≤ eps² — closed ball, no sqrt
  // anywhere) and the neighborhood counts its center itself. Four collinear points
  // one unit apart at eps = 1.0: p0's neighborhood is {p0, p1} — two points, below
  // min_pts 3 — so p0 is noise; p1 sees {p0, p1, p2} (distance EXACTLY 1.0 counts)
  // and seeds the only cluster. One point below the threshold: no core anywhere, so
  // NO cluster at all — empty input and sub-threshold input both yield empty.
  const std::vector<Point> pts{Point{0.0, 0.0}, Point{1.0, 0.0}, Point{2.0, 0.0},
                               Point{3.0, 0.0}};
  const std::vector<std::vector<Point>> want{
      {Point{1.0, 0.0}, Point{2.0, 0.0}, Point{3.0, 0.0}}};
  EXPECT_EQ(dbscan_cluster(pts, 1.0, 3), want);
  EXPECT_TRUE(dbscan_cluster({Point{0.0, 0.0}, Point{1.0, 0.0}}, 1.0, 3).empty());
  EXPECT_TRUE(dbscan_cluster({}, 1.0, 3).empty());
}

TEST(Dbscan, NoiseIsStickyByVisitOrder) {
  // The algorithm's own arbitrariness, pinned. Same four points, reordered: as
  // ((5,0),(0,0),(1,0),(2,0)) the point (0,0) is visited while still sparse (its
  // neighborhood holds 2 < 3), so it is labeled NOISE — and it stays noise even
  // though it sits exactly eps from the later cluster's seed. In the first ordering
  // (0,0) was itself a member; order is the only difference.
  const std::vector<Point> pts{Point{5.0, 0.0}, Point{0.0, 0.0}, Point{1.0, 0.0},
                               Point{2.0, 0.0}};
  const std::vector<std::vector<Point>> want{{Point{1.0, 0.0}, Point{2.0, 0.0}}};
  EXPECT_EQ(dbscan_cluster(pts, 1.0, 3), want);
}

TEST(Dbscan, IdsFollowDiscoveryOrderNotSpace) {
  // Cluster ids are the ascending index of each cluster's first core point — not any
  // spatial order. The RIGHT-hand triple comes first in the input list, so it earns
  // id 0 and the left-hand pair (whose own seed is visited later) earns id 1; each
  // cluster's members keep their append order (seed first).
  const std::vector<Point> pts{Point{5.0, 0.0}, Point{6.0, 0.0}, Point{7.0, 0.0},
                               Point{0.0, 0.0}, Point{0.0, 1.0}, Point{0.0, 2.0}};
  const std::vector<std::vector<Point>> want{{Point{6.0, 0.0}, Point{7.0, 0.0}},
                                             {Point{0.0, 1.0}, Point{0.0, 2.0}}};
  EXPECT_EQ(dbscan_cluster(pts, 1.0, 3), want);
}

TEST(Dbscan, RunOnScenarioLandmarksComeFromPoints) {
  // The branch story end-to-end on blobs01_line with the config defaults (no sensor
  // params are declared, so the demo injects NOTHING): t = 0 emits the declared gauge
  // pose exactly and already resolves three clusters; every later step composes the
  // arriving EXACT twist onto that pose, so every estimated pose equals ground truth
  // bit-for-bit and ATE/RPE are exactly 0.0. The final landmarks sit on the blobs'
  // faces (B at x = 5.9999… against cell center 6.25) — a beam stops at the face,
  // and that half-cell is what landmark_rmse honestly measures.
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/blobs01_line.yaml"));
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/features/dbscan.yaml"));
  Episode episode = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                              &sc.landmarks, sc.sigma_xy, sc.sigma_theta, sc.seed);

  std::ostringstream os;
  TraceRecorder rec(os);
  Dbscan est(params);
  EstimateResult result = est.run(episode, &rec);
  ASSERT_EQ(result.poses.size(), episode.steps.size());
  ASSERT_EQ(episode.steps.size(), 11u);

  const std::string jsonl = os.str();
  std::vector<std::string> names = event_names(jsonl);
  ASSERT_EQ(names.size(), 33u);
  for (size_t k = 0; k < 11; ++k) {
    EXPECT_EQ(names[3 * k], "step_observed");
    EXPECT_EQ(names[3 * k + 1], "pose_estimated");
    EXPECT_EQ(names[3 * k + 2], "landmarks_updated");
  }
  std::vector<std::string> lines = split_lines(jsonl);
  ASSERT_EQ(lines.size(), 33u);

  // t = 0: the declared gauge, emitted verbatim — no odom arrived at step 0, and no
  // "cov" anywhere (dbscan carries no uncertainty model), no sx/sy on landmarks.
  EXPECT_EQ(lines[0].find("\"odom\""), std::string::npos);
  std::array<double, 3> p0 = parse_triple(lines[1], "\"pose\":");
  EXPECT_EQ(p0[0], 0.75);
  EXPECT_EQ(p0[1], 1.75);
  EXPECT_EQ(p0[2], 0.0);
  EXPECT_EQ(lines[1].find("\"cov\""), std::string::npos);

  // Three clusters ALREADY at t = 0, ids [0, 1, 2] on every step (discovery order =
  // bearing order here), and no sx/sy field anywhere (no per-axis sigma emitted).
  for (size_t k = 0; k < 11; ++k) {
    EXPECT_EQ(lines[3 * k + 2].find("\"sx\""), std::string::npos);
    std::vector<std::array<double, 3>> lm = parse_estimated(lines[3 * k + 2]);
    ASSERT_EQ(lm.size(), 3u) << "step " << k;
    for (size_t id = 0; id < 3; ++id) EXPECT_EQ(lm[id][0], static_cast<double>(id));
  }
  const std::array<std::array<double, 2>, 3> t0_pins{
      {{4.493887564731294, 1.2896867167788368},   // A — face x ≈ 4.5, center is 4.75
       {6.005154080104347, 1.795749986472559},     // B — dead-ahead face at x = 6.0
       {4.50441122289002, 2.2452803504816274}}};   // C — mirror of A across y = 1.75
  std::vector<std::array<double, 3>> lm0 = parse_estimated(lines[2]);
  ASSERT_EQ(lm0.size(), 3u);
  for (size_t id = 0; id < 3; ++id) {
    EXPECT_EQ(lm0[id][1], t0_pins[id][0]);
    EXPECT_EQ(lm0[id][2], t0_pins[id][1]);
  }

  // Final estimates: the accumulated clouds densify and the centroids converge onto
  // the faces; every pinned double is bit-exact in both languages.
  ASSERT_EQ(result.landmarks.size(), 3u);
  const std::array<std::array<double, 2>, 3> pins{
      {{4.518062212080072, 1.2759425774767463},
       {5.9999255335068975, 1.7501295266874315},
       {4.518751688800652, 2.224051537406564}}};
  for (size_t id = 0; id < 3; ++id) {
    EXPECT_EQ(result.landmarks[id].id, static_cast<int>(id));
    EXPECT_EQ(result.landmarks[id].x, pins[id][0]);
    EXPECT_EQ(result.landmarks[id].y, pins[id][1]);
  }

  // Pose = declared gauge ⊕ exact odometry — bit-for-bit ground truth, so both pose
  // metrics are EXACTLY zero and the only real number is landmark_rmse.
  for (size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_EQ(result.poses[i].x, episode.steps[i].gt.x);
    EXPECT_EQ(result.poses[i].y, episode.steps[i].gt.y);
    EXPECT_EQ(result.poses[i].theta, episode.steps[i].gt.theta);
  }
  const std::map<std::string, double> metrics = slam::core::evaluate(result, episode);
  EXPECT_EQ(metrics.at("ate_rmse"), 0.0);
  EXPECT_EQ(metrics.at("rpe_rmse"), 0.0);
  EXPECT_EQ(metrics.at("landmark_rmse"), 0.23885456837708618);
}

TEST(Dbscan, RunConsumesOdometryUnlikeRegistration) {
  // The branch contrast, pinned: registration NEVER reads Step.odom; this branch
  // lives on it. Rebuilding the same episode with σ_xy = σ_θ = 5.0 shatters the
  // world-frame accumulation — 33 scattered clusters instead of three, ATE 12.878…,
  // and even the surviving first three landmarks move. Every pinned number below is a
  // measured consequence of consuming noisy odometry.
  slam::maps::Scenario sc = slam::maps::load_scenario(
      slam::test::repo_path("maps/scenarios/blobs01_line.yaml"));
  ParamSet params = ParamSet::from_yaml(slam::test::repo_path("configs/features/dbscan.yaml"));
  Episode noisy = slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                            &sc.landmarks, 5.0, 5.0, sc.seed);
  EstimateResult result = Dbscan(params).run(noisy, nullptr);
  ASSERT_EQ(result.landmarks.size(), 33u);
  const std::map<std::string, double> metrics = slam::core::evaluate(result, noisy);
  EXPECT_EQ(metrics.at("ate_rmse"), 12.878164352825113);
  EXPECT_EQ(metrics.at("rpe_rmse"), 6.941842126318921);
  EXPECT_EQ(metrics.at("landmark_rmse"), 0.2513611765791011);
}
