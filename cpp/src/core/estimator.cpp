#include "slam/core/estimator.hpp"

namespace slam::core {

EstimateResult Estimator::run(const Episode& episode, TraceRecorder* recorder) {
  // The shared template: every input Step is echoed to the trace BEFORE the
  // estimator sees it (algorithms never emit step_observed themselves).
  episode_ = &episode;
  for (const Step& step : episode.steps) {
    if (recorder != nullptr) recorder->step_observed(step);
    update(step, recorder);
  }
  return finalize(recorder);
}

}  // namespace slam::core
