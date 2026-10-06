// Estimator base (run template) + metrics contract tests — the C++ mirror of
// python/tests/test_estimator.py.

#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/estimator.hpp"
#include "slam/core/geometry.hpp"
#include "slam/core/metrics.hpp"
#include "slam/core/params.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"
#include "test_util.hpp"

using slam::core::Capability;
using slam::core::EstimateResult;
using slam::core::Estimator;
using slam::core::LandmarkEstimate;
using slam::core::LogOddsGrid;
using slam::core::ParamSet;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

namespace {

// Minimal estimator: dead-reckons its pose from odometry only (no sensor use).
class EchoEstimator final : public Estimator {
 public:
  explicit EchoEstimator(ParamSet params) : Estimator(std::move(params)) {}

  std::string name() const override { return "echo"; }
  std::set<Capability> required_capabilities() const override {
    return {Capability::BEAM, Capability::LANDMARKS};
  }

  void update(const Step& step, TraceRecorder* recorder) override {
    if (!pose_.has_value()) {
      // Step 0 has no odom: the estimator starts at the origin pose.
      pose_ = Pose{0.0, 0.0, 0.0};
    } else if (step.has_odom) {
      pose_ = slam::core::pose_compose(*pose_, step.odom);
    }
    poses.push_back(*pose_);
    if (recorder != nullptr) recorder->pose_estimated(step.t, *pose_);
  }

  EstimateResult finalize(TraceRecorder* /*recorder*/) override {
    EstimateResult result;
    result.poses = poses;
    return result;
  }

 private:
  std::vector<Pose> poses;
  std::optional<Pose> pose_;
};

ParamSet empty_params() {
  return ParamSet::from_yaml(slam::test::write_temp("echo.yaml",
                                                    "algorithm: echo\nsection: filtering\n"
                                                    "scenarios: []\nparams: []\n"));
}

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

}  // namespace

TEST(Estimator, RunTemplateEchoesStepsAndReturnsPoses) {
  slam::core::Episode episode;
  Step s0;
  s0.t = 0;
  s0.gt = Pose{1.0, 2.0, 0.0};
  s0.has_scan = true;  // empty scan, no obs — like the Python Step(scan=(), obs=None)
  Step s1;
  s1.t = 1;
  s1.gt = Pose{2.0, 2.0, 0.0};
  s1.has_odom = true;
  s1.odom = Twist{1.0, 0.0, 0.0};
  s1.has_scan = true;
  slam::maps::OccupancyGrid2D grid = slam::test::make_grid({".."});
  episode.grid = &grid;
  episode.steps = {s0, s1};

  EchoEstimator est(empty_params());
  std::ostringstream os;
  TraceRecorder rec(os);
  EstimateResult result = est.run(episode, &rec);

  // Dead reckoning from the origin: (0,0) then compose(Twist(1,0,0)) -> (1,0).
  ASSERT_EQ(result.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(result.poses[0].x, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[0].y, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[1].x, 1.0);
  EXPECT_DOUBLE_EQ(result.poses[1].y, 0.0);

  // step_observed echoed BEFORE the estimate event, in seq order; both carry t of
  // their step (the first two events are the t=0 pair).
  std::vector<std::string> names = event_names(os.str());
  ASSERT_EQ(names.size(), 4u);
  EXPECT_EQ(names[0], "step_observed");
  EXPECT_EQ(names[1], "pose_estimated");
  EXPECT_EQ(names[2], "step_observed");
  EXPECT_EQ(names[3], "pose_estimated");
  std::istringstream in(os.str());
  std::string line0, line1;
  std::getline(in, line0);
  std::getline(in, line1);
  EXPECT_NE(line0.find("\"t\":0"), std::string::npos);
  EXPECT_NE(line1.find("\"t\":0"), std::string::npos);

  // A null recorder costs nothing: no events, same result.
  EchoEstimator est2(empty_params());
  EstimateResult r2 = est2.run(episode, nullptr);
  ASSERT_EQ(r2.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(r2.poses[1].x, 1.0);
}

TEST(Metrics, AteRpeGolden) {
  std::vector<Pose> gts{Pose{0.0, 0.0, 0.0}, Pose{1.0, 0.0, 0.0}};
  std::vector<Pose> ests{Pose{0.0, 1.0, 0.0}, Pose{1.0, 1.0, 0.0}};
  // ATE: position errors (0,1) and (0,1): mean of squares = 1 -> sqrt = 1.
  EXPECT_DOUBLE_EQ(slam::core::ate_rmse(ests, gts), 1.0);
  // RPE: both trajectories move exactly +x per step -> relative twist difference 0.
  EXPECT_DOUBLE_EQ(slam::core::rpe_rmse(ests, gts), 0.0);
}

TEST(Metrics, MapIouAndLandmarks) {
  // gt grid from ascii rows ".#" / ".." at resolution 1: occupied at (row0,col1).
  slam::maps::OccupancyGrid2D grid(std::vector<std::uint16_t>{255, 0, 255, 255}, 2, 2, 1.0,
                                   {0.0, 0.0});
  LogOddsGrid est;
  est.resolution = 1.0;
  est.origin = {0.0, 0.0};
  est.height = 2;
  est.width = 2;
  est.log_odds = {-1.0, 1.0, -1.0, -1.0};  // occupied iff > 0 -> exactly (0,1)
  EXPECT_DOUBLE_EQ(slam::core::map_iou(est, grid), 1.0);

  LogOddsGrid wrong = est;
  wrong.log_odds = {-1.0, -1.0, -1.0, -1.0};  // est sees nothing -> inter 0, union 1
  EXPECT_DOUBLE_EQ(slam::core::map_iou(wrong, grid), 0.0);

  auto lm = slam::core::landmark_rmse({LandmarkEstimate{0, 1.0, 2.0}}, {slam::core::Point{1.0, 3.0}});
  ASSERT_TRUE(lm.has_value());
  EXPECT_DOUBLE_EQ(*lm, 1.0);
  ASSERT_FALSE(slam::core::landmark_rmse({}, {slam::core::Point{1.0, 3.0}}).has_value());
}

TEST(Metrics, EvaluateBundleKeys) {
  slam::maps::OccupancyGrid2D gt_grid(std::vector<std::uint16_t>{255, 0, 255, 255}, 2, 2, 2.0,
                                      {0.0, 0.0});
  slam::core::Episode episode;
  Step s0;
  s0.t = 0;
  s0.gt = Pose{0.0, 0.0, 0.0};
  s0.has_scan = true;
  episode.steps = {s0};
  episode.has_landmarks = true;
  episode.landmarks = {slam::core::Point{1.0, 3.0}};
  episode.grid = &gt_grid;

  // Estimate grid must share the raster: same resolution/origin as gt_grid.
  LogOddsGrid est_grid;
  est_grid.resolution = gt_grid.resolution();
  est_grid.origin = gt_grid.origin();
  est_grid.height = gt_grid.height();
  est_grid.width = gt_grid.width();
  est_grid.log_odds = {-1.0, 1.0, -1.0, -1.0};

  EstimateResult result;
  result.poses = {Pose{0.0, 1.0, 0.0}};
  result.has_landmarks = true;
  result.landmarks = {LandmarkEstimate{0, 1.0, 2.0}};
  result.has_grid = true;
  result.grid = est_grid;

  std::map<std::string, double> m = slam::core::evaluate(result, episode);
  ASSERT_EQ(m.size(), 4u);
  EXPECT_TRUE(m.count("ate_rmse"));
  EXPECT_TRUE(m.count("rpe_rmse"));
  EXPECT_TRUE(m.count("map_iou"));
  EXPECT_TRUE(m.count("landmark_rmse"));
}
