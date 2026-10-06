#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "slam/core/types.hpp"

namespace slam::maps {

// OccupancyGrid2D — ROS map_server-style raster implementing the core ScanGrid
// view. world<->grid conversion lives ONLY here (the map layer owns the coordinate
// frames, per the repo rule): cell (r, c) center = (origin_x + (c+0.5)*res,
// origin_y + ((H-1-r)+0.5)*res); inverse c = floor((x-origin_x)/res), r =
// (H-1) - floor((y-origin_y)/res). Row 0 is the TOP image row — both formulas
// encode that flip; Python mirrors them operation for operation.
//
// Occupancy follows spec/data_formats.md: occ = 1 - pixel/255, a cell counts
// occupied iff occ > free_thresh (default 0.65). The repo's maps use only 0/255 so
// the threshold never wobbles; it exists because ROS yamls may carry unknowns.
class OccupancyGrid2D final : public core::ScanGrid {
 public:
  // Default-constructed (empty) grid: a Scenario default-constructs one before its
  // map yaml is loaded. pixels is row-major [height][width], values in [0, maxval].
  OccupancyGrid2D() = default;
  OccupancyGrid2D(std::vector<std::uint16_t> pixels, int height, int width, double resolution,
                  std::array<double, 2> origin, double free_thresh = 0.65);

  int width() const override { return width_; }
  int height() const override { return height_; }
  double resolution() const override { return resolution_; }
  // World pose (x, y) of the bottom-left pixel — readers outside the map layer
  // convert with the same frame.
  std::array<double, 2> origin() const override { return origin_; }

  bool in_bounds(int row, int col) const override;
  // Out of bounds counts as NOT occupied (a ray leaving the map misses; the
  // scenario guarantees the robot itself never leaves free space).
  bool occupied(int row, int col) const override;

  core::Point cell_to_world(int row, int col) const override;
  core::Cell world_to_cell(double x, double y) const override;

 private:
  int height_ = 0;
  int width_ = 0;
  double resolution_ = 0.0;
  std::array<double, 2> origin_{0.0, 0.0};
  // Free-cell mask (row-major): occupied(r,c) == in_bounds && !free_[r][c]. The
  // Python side exposes the same raster as a free_mask() array; C++ callers only
  // ever ask cell-by-cell (raycast DDA, metrics), so the mask stays private.
  std::vector<bool> free_;
};

}  // namespace slam::maps
