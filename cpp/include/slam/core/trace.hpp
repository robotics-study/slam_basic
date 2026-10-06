#pragma once

#include <array>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "slam/core/params.hpp"
#include "slam/core/types.hpp"

namespace slam::core {

// JSON number writer shared by the trace recorder and the demo stdout summary:
// integral-valued doubles emit integer bytes (Python's json writes 5.0 where this
// writes 5 — parsed values are equal) and everything else in the shortest
// round-trip form (std::format("{}") == Python repr).
void write_num(std::ostream& os, double v);

// Step-by-step trace recorder — the cross-language visualization contract
// (spec/trace_schema.json). seq starts at 0 and increments per event; both
// languages serialize each event's fields in exactly the documented order, so
// parsed traces match field-for-field. A SLAM trace carries NO wall-clock time:
// replay is driven purely by seq order, and the only time an event ever carries
// is the step number t. The demo driver emits run_started (the one event the
// estimator cannot emit — it knows neither its scenario path nor its seed) and,
// through the base Estimator::run loop, every step_observed; the ESTIMATOR emits
// only estimation events at the same t. A null TraceRecorder* is never
// dereferenced by estimators (hot-path guard at the call site), so tracing is
// zero-cost when off. The caller owns the ostream (an ofstream in the demo).
class TraceRecorder {
 public:
  explicit TraceRecorder(std::ostream& os) : os_(os) {}

  // The one event the estimator cannot emit: scenario path, seed and sensor
  // snapshot. params is a std::map — already sorted by key, exactly like Python's
  // dict(sorted(...)). The sensor block keeps the scenario yaml's fixed field order
  // (type, beams?, fov_deg?, range_max, sigma_range, sigma_bearing? — optional keys
  // omitted); landmarks ([[x,y], id = list order]) is omitted for beam scenarios.
  void run_started(const std::string& algorithm, const std::string& scenario,
                   const std::map<std::string, ParamValue>& params, long long seed,
                   const SensorConfig& sensor, const std::vector<Point>* landmarks);

  // One input step: the ground-truth pose (visualization + metrics only), the noisy
  // odometry command that ARRIVED at t (omitted on step 0 — no move arrived there)
  // and the observation: beam-scan endpoints in the ROBOT frame (replay restores
  // world points with the gt pose) or landmark observations sorted by id.
  void step_observed(const Step& step);

  // The estimator's own pose belief at step t; cov is the diagonal standard
  // deviation triple [sx, sy, stheta] (filter families only — replay draws the
  // error ellipse from it).
  void pose_estimated(int t, const Pose& pose,
                      const std::optional<std::array<double, 3>>& cov = {});

  // Histogram-filter belief: EVERY cell as [row, col, p] in row-major order
  // (p = probability the cell is occupied).
  void belief_updated(int t, const std::vector<CellProb>& cells);

  // Particle cloud after this step's update/resampling: [x, y, theta, w] per
  // particle (w = normalized weight; uniform right after resampling).
  void particles_updated(int t, const std::vector<Particle>& particles);

  // Current landmark estimates (id = the scenario list order it tracks); sx/sy
  // omitted when the estimator carries no per-axis sigma.
  void landmarks_updated(int t, const std::vector<LandmarkEstimate>& estimated);

  // Log-odds map growth: ONLY cells whose log-odds changed since the previous
  // emission, as [row, col, l] (sparse by construction).
  void map_updated(int t, const std::vector<CellProb>& cells);

  // Graph branch: a pose-pose constraint between trajectory nodes i and j — the
  // observed relative pose d in node i's frame. `loop` marks a loop closure (a
  // re-observation of a NON-adjacent node); omitted on odometry edges.
  void constraint_added(int i, int j, const Twist& d, const std::optional<bool>& loop = {});

  // Batch (graph_based) final trajectory: index == step t.
  void trajectory_found(const std::vector<Pose>& poses);

  // metrics keys are sorted (std::map iteration) so both languages emit identical
  // order; values go through write_num (integer bytes stay integer bytes).
  void run_finished(const std::map<std::string, double>& metrics);

 private:
  void begin_event(const char* event);
  void end_event();

  std::ostream& os_;
  long long seq_ = 0;
};

}  // namespace slam::core
