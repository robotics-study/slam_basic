// Trace wire contract: field order, seq monotonicity, sorted maps, int/float bytes.
// The expected lines below are the C++ side of the byte contract; Python's json
// writes 5.0 where write_num writes 5 — parsed values are equal by contract.

#include <array>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"

using slam::core::LandmarkEstimate;
using slam::core::LandmarkObs;
using slam::core::ParamValue;
using slam::core::Pose;
using slam::core::SensorConfig;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

TEST(Trace, ExactWireFormatMatchesPython) {
  std::ostringstream os;
  TraceRecorder rec(os);
  SensorConfig sensor;
  sensor.type = "beam";
  sensor.range_max = 6.0;
  sensor.sigma_range = 0.05;
  sensor.has_beams = true;
  sensor.beams = 90;
  sensor.has_fov_deg = true;
  sensor.fov_deg = 180.0;
  // Params arrive unsorted here; the recorder must emit them sorted, exactly like
  // Python's dict(sorted(...)). Int stays int on the wire.
  std::map<std::string, ParamValue> params{{"zeta", ParamValue(1)}, {"alpha", ParamValue(2.5)}};
  rec.run_started("algo_x", "maps/scenarios/x.yaml", params, 42, sensor, nullptr);

  Step step;
  step.t = 0;
  step.gt = Pose{1.0, 2.0, 0.5};
  step.has_scan = true;
  step.scan = {slam::core::Point{0.5, 0.0}};
  rec.step_observed(step);  // odom omitted at t=0; seq increments

  rec.pose_estimated(0, Pose{1.1, 2.1, 0.4}, std::array<double, 3>{0.1, 0.1, 0.02});
  rec.run_finished({{"rpe_rmse", 0.2}, {"ate_rmse", 0.1}});  // metrics sorted

  const std::vector<std::string> expected = {
      R"({"seq":0,"event":"run_started","algorithm":"algo_x","scenario":"maps/scenarios/x.yaml","params":{"alpha":2.5,"zeta":1},"seed":42,"sensor":{"type":"beam","beams":90,"fov_deg":180,"range_max":6,"sigma_range":0.05}})",
      R"({"seq":1,"event":"step_observed","t":0,"gt":[1,2,0.5],"scan":[[0.5,0]]})",
      R"({"seq":2,"event":"pose_estimated","t":0,"pose":[1.1,2.1,0.4],"cov":[0.1,0.1,0.02]})",
      R"({"seq":3,"event":"run_finished","metrics":{"ate_rmse":0.1,"rpe_rmse":0.2}})",
  };
  std::istringstream in(os.str());
  std::string line;
  size_t i = 0;
  while (std::getline(in, line)) {
    ASSERT_LT(i, expected.size()) << "extra event line: " << line;
    // A SLAM trace carries no wall-clock time — only seq + the step t.
    EXPECT_EQ(line, expected[i]);
    ++i;
  }
  EXPECT_EQ(i, expected.size());
}

TEST(Trace, StepObservedLandmarkFields) {
  std::ostringstream os;
  TraceRecorder rec(os);
  Step step;
  step.t = 3;
  step.gt = Pose{0.0, 0.0, 0.0};
  step.has_odom = true;
  step.odom = Twist{0.5, 0.0, 0.1};
  step.has_obs = true;
  step.obs = {LandmarkObs{2, 0.3, 2.5}};
  rec.step_observed(step);
  // sx/sy omitted when absent; present order fixed (id, x, y).
  rec.landmarks_updated(3, {LandmarkEstimate{2, 1.0, 2.0}});

  std::istringstream in(os.str());
  std::string line;
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_EQ(line,
            R"({"seq":0,"event":"step_observed","t":3,"gt":[0,0,0],"odom":[0.5,0,0.1],"obs":[{"id":2,"bearing":0.3,"range":2.5}]})");
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_EQ(line, R"({"seq":1,"event":"landmarks_updated","t":3,"estimated":[{"id":2,"x":1,"y":2}]})");

  // With sigma present the sx/sy fields ride along after y.
  std::ostringstream os2;
  TraceRecorder rec2(os2);
  LandmarkEstimate with_sigma{2, 1.0, 2.0};
  with_sigma.has_sigma = true;
  with_sigma.sx = 0.1;
  with_sigma.sy = 0.2;
  rec2.landmarks_updated(3, {with_sigma});
  EXPECT_EQ(os2.str(),
            R"({"seq":0,"event":"landmarks_updated","t":3,"estimated":[{"id":2,"x":1,"y":2,"sx":0.1,"sy":0.2}]})"
            "\n");
}

TEST(Trace, BeliefParticlesMapConstraintTrajectory) {
  // Pin the remaining event shapes (Python emits identical fields).
  std::ostringstream os;
  TraceRecorder rec(os);
  rec.belief_updated(1, {{2, 3, 0.25}});
  rec.particles_updated(1, {{0.5, 1.5, 0.25, 1.0}});
  rec.map_updated(1, {{2, 3, -0.5}});
  rec.constraint_added(0, 3, Twist{0.5, 0.0, 0.1}, true);
  rec.trajectory_found({Pose{0.0, 0.0, 0.0}, Pose{1.0, 0.0, 0.25}});

  std::istringstream in(os.str());
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) lines.push_back(line);
  ASSERT_EQ(lines.size(), 5u);
  EXPECT_EQ(lines[0], R"({"seq":0,"event":"belief_updated","t":1,"cells":[[2,3,0.25]]})");
  EXPECT_EQ(lines[1], R"({"seq":1,"event":"particles_updated","t":1,"particles":[[0.5,1.5,0.25,1]]})");
  EXPECT_EQ(lines[2], R"({"seq":2,"event":"map_updated","t":1,"cells":[[2,3,-0.5]]})");
  EXPECT_EQ(lines[3], R"({"seq":3,"event":"constraint_added","i":0,"j":3,"d":[0.5,0,0.1],"loop":true})");
  EXPECT_EQ(lines[4], R"({"seq":4,"event":"trajectory_found","poses":[[0,0,0],[1,0,0.25]]})");
}

TEST(Trace, FloatsUseShortestRoundTripForm) {
  // std::format("{}") emits the shortest decimal that re-reads to the same double —
  // byte-identical to Python's repr; integers stay integer bytes (parsed equal).
  std::ostringstream os;
  TraceRecorder rec(os);
  slam::core::CellProb cell{0, 0, 0.1};
  rec.belief_updated(0, {cell});
  EXPECT_NE(os.str().find("\"cells\":[[0,0,0.1]]"), std::string::npos);
}
