#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include "slam/core/estimator.hpp"
#include "slam/core/metrics.hpp"
#include "slam/core/params.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/trace.hpp"
#include "slam/maps/loader.hpp"

// Assembly-only scaffold shared by the demo executables: parse CLI args, wire an
// estimator to a loaded scenario, emit the trace, and print a one-line metrics
// summary. No estimation logic lives here. The two events the estimator cannot
// emit — run_started (scenario path / seed / sensor snapshot) and run_finished
// (metrics against ground truth) — are emitted here; everything between them comes
// from the estimator through the base Estimator::run template. A stochastic
// algorithm's `seed` param receives the scenario seed verbatim — sim noise and
// algorithm-internal streams then share the seed value but stay independent
// streams (spec/data_formats.md). Sensor params follow the same injection contract:
// a declared beams/fov_deg/range_max/sigma_range/sigma_bearing param is filled from
// the scenario's sensor block, and declaring a field the scenario lacks is an error.
namespace demo {

struct Args {
  std::string scenario;
  std::string params;
  std::string trace;
};

inline Args parse_args(int argc, char** argv) {
  Args a;
  auto need = [&](int& i) -> std::string {
    if (i + 1 >= argc) throw std::runtime_error("demo: missing value for " + std::string(argv[i]));
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    std::string f = argv[i];
    if (f == "--scenario") {
      a.scenario = need(i);
    } else if (f == "--params") {
      a.params = need(i);
    } else if (f == "--trace") {
      a.trace = need(i);
    } else {
      throw std::runtime_error("demo: unknown flag " + f);
    }
  }
  if (a.scenario.empty() || a.params.empty() || a.trace.empty()) {
    throw std::runtime_error("demo: --scenario --params --trace are required");
  }
  return a;
}

// One demo run: load the scenario, inject the seed, build the episode, echo +
// estimate through the shared run template, then print the one-line JSON summary —
// algorithm first, metric keys SORTED (std::map iteration), numbers written with
// the trace's write_num so both languages' stdout parses to identical values.
template <class EstimatorT>
inline int run(int argc, char** argv) {
  Args a = parse_args(argc, argv);
  slam::core::ParamSet params = slam::core::ParamSet::from_yaml(a.params);
  slam::maps::Scenario sc = slam::maps::load_scenario(a.scenario);
  // A stochastic algorithm declares its own `seed` param; the demo injects the
  // scenario seed verbatim (contract — see module header).
  if (params.has("seed")) params.set("seed", static_cast<int>(sc.seed));
  // Sensor params follow the same injection contract as seed: an algorithm sees the
  // sensor ONLY through declared params, never by reading the scenario itself.
  // Declaring a field the scenario's sensor lacks is an error — no silent defaults.
  if (params.has("beams")) {
    if (!sc.sensor.has_beams)
      throw std::runtime_error("demo: declared 'beams' but the scenario sensor has none");
    params.set("beams", static_cast<int>(sc.sensor.beams));
  }
  if (params.has("fov_deg")) {
    if (!sc.sensor.has_fov_deg)
      throw std::runtime_error("demo: declared 'fov_deg' but the scenario sensor has none");
    params.set("fov_deg", sc.sensor.fov_deg);
  }
  if (params.has("range_max")) params.set("range_max", sc.sensor.range_max);
  if (params.has("sigma_range")) params.set("sigma_range", sc.sensor.sigma_range);
  if (params.has("sigma_bearing")) {
    if (!sc.sensor.has_sigma_bearing)
      throw std::runtime_error("demo: declared 'sigma_bearing' but the scenario sensor has none");
    params.set("sigma_bearing", sc.sensor.sigma_bearing);
  }

  slam::core::Episode episode =
      slam::core::build_episode(sc.grid, sc.waypoints, sc.step_meters, sc.sensor,
                                sc.has_landmarks ? &sc.landmarks : nullptr, sc.sigma_xy,
                                sc.sigma_theta, sc.seed);

  EstimatorT estimator(params);
  std::set<slam::core::Capability> caps = estimator.required_capabilities();
  slam::core::Capability want = slam::core::capability_from(sc.sensor.type);
  if (!caps.count(want)) {
    throw std::runtime_error("demo: " + estimator.name() + " does not consume the '" +
                             sc.sensor.type + "' sensor of " + a.scenario);
  }

  std::ofstream fs(a.trace);
  if (!fs) throw std::runtime_error("demo: cannot open trace file " + a.trace);
  slam::core::TraceRecorder rec(fs);
  rec.run_started(estimator.name(), a.scenario, params.values(), sc.seed, sc.sensor,
                  sc.has_landmarks ? &sc.landmarks : nullptr);
  slam::core::EstimateResult result = estimator.run(episode, &rec);
  std::map<std::string, double> metrics = slam::core::evaluate(result, episode);
  rec.run_finished(metrics);

  // One-line JSON on stdout (bench + web export read it).
  std::cout << "{\"algorithm\":\"" << estimator.name() << '"';
  for (const auto& [key, value] : metrics) {
    std::cout << ",\"" << key << "\":";
    slam::core::write_num(std::cout, value);
  }
  std::cout << "}\n";
  return 0;
}

}  // namespace demo
