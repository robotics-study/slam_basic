#pragma once

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace slam::core {

// The double nearest to pi — bit-identical to Python's math.pi (both are the same
// IEEE-754 rounding of pi); every geometry formula uses this exact constant.
inline constexpr double kPi = 3.14159265358979323846264338327950288;

// Grid index (row, col), ints; row 0 = top image row. Owned by the map layer.
struct Cell {
  int row = 0;
  int col = 0;
  bool operator==(const Cell& o) const { return row == o.row && col == o.col; }
};

// World point (x, y), meters. Scan endpoints live in the ROBOT frame; every other
// point in this codebase is world-frame. Exact equality is a real predicate here:
// every float decision upstream is bit-identical across languages.
struct Point {
  double x = 0.0;
  double y = 0.0;
  bool operator==(const Point& o) const { return x == o.x && y == o.y; }
};

// World pose (x, y meters, theta radians in [-pi, pi) after wrap).
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

// Robot-frame motion command: displacement (dx, dy) meters + rotation dtheta rad.
// A Step's twist is the EXACT inverse composition gt_{t-1} (+)⁻¹ gt_t plus drawn
// noise — algorithms read `odom`, never ground truth.
struct Twist {
  double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
};

// One landmark observation: id = scenario list order, bearing relative to the robot
// heading (rad), range in meters. Association is GIVEN (as in the FastSLAM papers).
struct LandmarkObs {
  int id = 0;
  double bearing = 0.0;
  double range = 0.0;
};

// An estimator's current belief about landmark `id` (world position + optional
// per-axis standard deviations, emitted on landmarks_updated events).
struct LandmarkEstimate {
  int id = 0;
  double x = 0.0;
  double y = 0.0;
  bool has_sigma = false;
  double sx = 0.0;
  double sy = 0.0;
};

// One cell of a belief/map update payload ([row, col, value]): p is the occupancy
// probability on belief_updated and the log-odds on map_updated (Python mirrors
// both with plain (int, int, float) tuples).
struct CellProb {
  int row = 0;
  int col = 0;
  double p = 0.0;
};

// One particle of a particles_updated payload: pose + normalized weight (uniform
// right after resampling).
struct Particle {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double w = 0.0;
};

// One simulator step t. `gt` is ground truth (visualization + metrics ONLY);
// `has_odom/odom` is the noisy robot-frame command that ARRIVED at t (absent on
// step 0 — no move arrived there); exactly one of scan / obs carries the reading.
struct Step {
  int t = 0;
  Pose gt;
  bool has_odom = false;
  Twist odom;
  bool has_scan = false;
  std::vector<Point> scan;
  bool has_obs = false;
  std::vector<LandmarkObs> obs;
};

// Parsed `sensor:` block of a scenario yaml (see spec/data_formats.md). The `type`
// string equals the Capability value an estimator must declare.
struct SensorConfig {
  std::string type;  // "beam" | "landmarks"
  double range_max = 0.0;
  double sigma_range = 0.0;
  bool has_beams = false;
  int beams = 0;
  bool has_fov_deg = false;
  double fov_deg = 0.0;
  bool has_sigma_bearing = false;
  double sigma_bearing = 0.0;
};

// A log-odds occupancy map (the mapping branch's output): one float per cell on the
// SAME raster geometry as the ground-truth grid it is scored against. A cell counts
// occupied iff log_odds > 0 (unknown stays at its prior, never "occupied").
struct LogOddsGrid {
  double resolution = 0.0;
  std::array<double, 2> origin{0.0, 0.0};
  int height = 0;
  int width = 0;
  std::vector<double> log_odds;  // row-major [height][width]

  double at(int row, int col) const { return log_odds[static_cast<size_t>(row) * width + col]; }
};

// The sensor kinds an estimator can consume (matches the scenario `sensor.type`).
enum class Capability { BEAM, LANDMARKS };

inline const char* to_string(Capability c) { return c == Capability::BEAM ? "beam" : "landmarks"; }
inline Capability capability_from(const std::string& s) {
  if (s == "beam") return Capability::BEAM;
  if (s == "landmarks") return Capability::LANDMARKS;
  throw std::runtime_error("unknown capability '" + s + "'");
}

// Read-only grid view the simulator raycasts against and metrics score against.
// Algorithms depend on THIS (via Episode.grid), never on a concrete map class;
// maps/OccupancyGrid2D is the one implementation. Coordinate-frame conversion is
// owned by the map layer (fixed formulas mirrored 1:1 from Python).
struct ScanGrid {
  virtual ~ScanGrid() = default;
  virtual int width() const = 0;
  virtual int height() const = 0;
  virtual double resolution() const = 0;
  virtual std::array<double, 2> origin() const = 0;
  virtual bool in_bounds(int row, int col) const = 0;
  // Out of bounds counts as NOT occupied (a ray leaving the map misses; the
  // scenario guarantees the robot itself never leaves free space).
  virtual bool occupied(int row, int col) const = 0;
  virtual Point cell_to_world(int row, int col) const = 0;
  virtual Cell world_to_cell(double x, double y) const = 0;
};

// Everything one estimator run consumes: the step stream (ground truth + noisy
// odometry + observations, drawn in contract order from the scenario seed), the
// ground-truth landmark points (id = list order; absent for beam scenarios —
// visualization and metrics only), and the ground-truth grid.
struct Episode {
  std::vector<Step> steps;
  bool has_landmarks = false;
  std::vector<Point> landmarks;
  const ScanGrid* grid = nullptr;
};

// What an estimator returns: per-step estimated poses (index == step t) plus the
// branch-specific estimates (landmark list / log-odds map); unset ones skip the
// matching metric in core/metrics.
struct EstimateResult {
  std::vector<Pose> poses;
  bool has_landmarks = false;
  std::vector<LandmarkEstimate> landmarks;
  bool has_grid = false;
  LogOddsGrid grid;
};

}  // namespace slam::core
