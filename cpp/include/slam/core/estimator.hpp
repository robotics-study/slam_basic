#pragma once

#include <set>
#include <string>

#include "slam/core/params.hpp"
#include "slam/core/trace.hpp"
#include "slam/core/types.hpp"

namespace slam::core {

// Base for every estimator: filtering / registration / filter_based / graph_based.
// One abstract per-step hook (update) plus a finalize that returns the run's
// EstimateResult; the concrete run() template is SHARED (not per-algorithm): it
// walks the episode's steps, echoes each input Step to the trace as step_observed
// before the estimator sees it, then calls finalize. An algorithm therefore never
// emits step_observed itself and never touches ground truth except through what
// run() hands it (episode_ is exposed for grid access, not for gt poses).
class Estimator {
 public:
  explicit Estimator(ParamSet params) : params_(std::move(params)) {}
  virtual ~Estimator() = default;

  // Algorithm id; matches the config filename and trace `algorithm` field.
  virtual std::string name() const = 0;
  // Which sensor types this estimator consumes; the demo asserts the scenario's
  // sensor.type is one of them before running.
  virtual std::set<Capability> required_capabilities() const = 0;
  // Consume one step (odometry + observation); emit this step's estimation events
  // on recorder (nullptr = tracing off — zero cost on the hot path).
  virtual void update(const Step& step, TraceRecorder* recorder) = 0;
  // Build the run's result; batch families emit their trajectory_found here.
  virtual EstimateResult finalize(TraceRecorder* recorder) = 0;

  // Shared template: echo each step, update per step, then finalize.
  EstimateResult run(const Episode& episode, TraceRecorder* recorder);

 protected:
  ParamSet params_;
  const Episode* episode_ = nullptr;  // set by run() before the first update
};

}  // namespace slam::core
