#pragma once

#include <string>
#include <vector>

#include "slam/core/types.hpp"
#include "slam/maps/occupancy_grid.hpp"

namespace slam::maps {

// One parsed scenario yaml: the map, the GT polyline (world coords), the arc
// spacing, the sensor block, landmark points (id = list order; absent for beam),
// the odometry noise sigmas and the splitmix64 seed. The loader parses; core/sim
// builds the Step stream from it (the loader never touches rng). Dispatch is on
// the yaml `type` field, not the extension.
struct Scenario {
  std::string map_path;  // absolute, resolved relative to the scenario file
  OccupancyGrid2D grid;
  std::vector<core::Point> waypoints;
  double step_meters = 0.0;
  core::SensorConfig sensor;
  bool has_landmarks = false;
  std::vector<core::Point> landmarks;
  double sigma_xy = 0.0;
  double sigma_theta = 0.0;
  long long seed = 0;
};

// Dispatches on the yaml `type` field. Only occupancy_grid is implemented; other
// types throw a clear error.
OccupancyGrid2D load_map(const std::string& path);

Scenario load_scenario(const std::string& path);

}  // namespace slam::maps
