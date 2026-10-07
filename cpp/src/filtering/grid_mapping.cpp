#include "slam/filtering/grid_mapping.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "slam/core/geometry.hpp"

// Bit-identical mirror of the Python module. log binds directly to libSystem's
// scalar log (no pair fold exists for it — see core/libm.hpp); floor inside
// world_to_cell is a hardware instruction in both languages. Every float op below
// rounds exactly where its Python line does: individual multiply/add/subtract, no
// fusion (-ffp-contract=off).

namespace slam::filtering {

using slam::core::Cell;
using slam::core::CellProb;
using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::Point;
using slam::core::Pose;
using slam::core::ScanGrid;
using slam::core::Step;
using slam::core::TraceRecorder;

namespace {

// Strict weak order on Cell by (row, col) — the walk order Python's sorted() over
// (row, col) tuples produces.
struct CellLt {
  bool operator()(const Cell& a, const Cell& b) const {
    return a.row != b.row ? a.row < b.row : a.col < b.col;
  }
};

}  // namespace

std::vector<Cell> cells_along(const ScanGrid& grid, Point from_pt, Point to_pt) {
  const double x0 = from_pt.x;
  const double y0 = from_pt.y;
  const double dx = to_pt.x - x0;
  const double dy = to_pt.y - y0;
  const int h = grid.height();  // row index grows downward — only height enters tMax
  Cell rc = grid.world_to_cell(x0, y0);
  if (!grid.in_bounds(rc.row, rc.col)) return {};  // start off-raster: nothing exists out there
  const Cell target = grid.world_to_cell(to_pt.x, to_pt.y);
  std::vector<Cell> cells{rc};
  if (rc == target) return cells;  // degenerate segment (or same cell): the start IS the hit

  const std::array<double, 2> origin = grid.origin();
  const double ox = origin[0];
  const double oy = origin[1];
  const double res = grid.resolution();
  const double inf = std::numeric_limits<double>::infinity();
  int step_x;
  double t_max_x;
  if (dx > 0.0) {
    step_x = 1;
    t_max_x = (ox + (static_cast<double>(rc.col) + 1.0) * res - x0) / dx;
  } else if (dx < 0.0) {
    step_x = -1;
    t_max_x = (ox + static_cast<double>(rc.col) * res - x0) / dx;
  } else {
    step_x = 0;
    t_max_x = inf;
  }
  int step_y;
  double t_max_y;
  if (dy > 0.0) {
    step_y = -1;  // row index grows downward
    t_max_y = (oy + static_cast<double>(h - rc.row) * res - y0) / dy;
  } else if (dy < 0.0) {
    step_y = 1;
    t_max_y = (oy + static_cast<double>(h - 1 - rc.row) * res - y0) / dy;
  } else {
    step_y = 0;
    t_max_y = inf;
  }
  const double delta_x = dx != 0.0 ? res / std::abs(dx) : inf;
  const double delta_y = dy != 0.0 ? res / std::abs(dy) : inf;

  while (true) {
    if (t_max_x < t_max_y) {
      rc.col += step_x;
      t_max_x += delta_x;
    } else {
      rc.row += step_y;
      t_max_y += delta_y;
    }
    if (!grid.in_bounds(rc.row, rc.col)) return cells;  // left the raster — last cell absorbs
    cells.push_back(rc);
    if (rc == target) return cells;
  }
}

GridMapping::GridMapping(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)) {
  const double p_hit = params_.get_float("p_hit");
  x0_ = params_.get_float("x0");
  y0_ = params_.get_float("y0");
  theta_deg_ = params_.get_float("theta_deg");
  // The single log-odds increment magnitude (symmetric model, header docstring).
  logit_ = std::log(p_hit / (1.0 - p_hit));
}

void GridMapping::update(const Step& step, TraceRecorder* recorder) {
  const ScanGrid& grid = *episode_->grid;
  if (!initialized_) {  // first update: allocate on the ground-truth raster
    field_.assign(static_cast<size_t>(grid.height()) * static_cast<size_t>(grid.width()), 0.0);
    initialized_ = true;
  }

  Pose pose;
  if (!step.has_odom) {  // t = 0 — the declared anchor IS the pose
    pose = Pose{x0_, y0_, theta_deg_ * (kPi / 180.0)};  // == Python math.radians
  } else {
    pose = slam::core::pose_compose(poses_.back(), step.odom);
  }
  poses_.push_back(pose);

  if (step.has_scan) {
    const double hit_l = logit_;
    const int w = grid.width();
    std::set<Cell, CellLt> touched;
    for (const Point& z : step.scan) {  // scan order (beam ascending) — fixed accumulation order
      Point w_pt = slam::core::robot_to_world(z, pose);
      std::vector<Cell> cells = cells_along(grid, Point{pose.x, pose.y}, w_pt);
      if (cells.empty()) continue;
      for (size_t i = 0; i + 1 < cells.size(); ++i) {  // passed through → free evidence
        const Cell cell = cells[i];
        field_[static_cast<size_t>(cell.row) * static_cast<size_t>(w) +
               static_cast<size_t>(cell.col)] -= hit_l;
        touched.insert(cell);
      }
      const Cell hit = cells.back();  // the endpoint's cell → occupied evidence
      field_[static_cast<size_t>(hit.row) * static_cast<size_t>(w) +
             static_cast<size_t>(hit.col)] += hit_l;
      touched.insert(hit);
    }
    if (recorder != nullptr) {
      recorder->pose_estimated(step.t, pose);
      // Row-major sorted emission (std::set ordered by (row, col) walks in exactly
      // Python's sorted(tuples) order).
      std::vector<CellProb> cells_out;
      for (const Cell& c : touched) {
        const double l = field_[static_cast<size_t>(c.row) * static_cast<size_t>(w) +
                               static_cast<size_t>(c.col)];
        cells_out.push_back(CellProb{c.row, c.col, l});
      }
      recorder->map_updated(step.t, cells_out);
    }
    return;
  }
  if (recorder != nullptr) {  // no scan (impossible on beam scenarios, honest anyway)
    recorder->pose_estimated(step.t, pose);
  }
}

EstimateResult GridMapping::finalize(TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  result.has_grid = true;
  result.grid.resolution = episode_->grid->resolution();
  result.grid.origin = episode_->grid->origin();
  result.grid.height = episode_->grid->height();
  result.grid.width = episode_->grid->width();
  result.grid.log_odds = field_;
  return result;
}

}  // namespace slam::filtering
