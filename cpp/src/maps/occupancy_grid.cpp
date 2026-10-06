#include "slam/maps/occupancy_grid.hpp"

#include <cmath>
#include <stdexcept>

namespace slam::maps {

OccupancyGrid2D::OccupancyGrid2D(std::vector<std::uint16_t> pixels, int height, int width,
                                 double resolution, std::array<double, 2> origin,
                                 double free_thresh)
    : height_(height), width_(width), resolution_(resolution), origin_(origin) {
  if (pixels.size() != static_cast<size_t>(height) * static_cast<size_t>(width)) {
    throw std::runtime_error("occupancy grid: pixel count does not match height x width");
  }
  // occ = 1 - p/255; occupied iff occ > free_thresh (unknown counts as occupied —
  // only clearly-free cells are free). Fixed operation order, mirrored in Python.
  free_.resize(pixels.size());
  for (size_t i = 0; i < pixels.size(); ++i) {
    double occ = 1.0 - static_cast<double>(pixels[i]) / 255.0;
    free_[i] = !(occ > free_thresh);
  }
}

bool OccupancyGrid2D::in_bounds(int row, int col) const {
  return 0 <= row && row < height_ && 0 <= col && col < width_;
}

bool OccupancyGrid2D::occupied(int row, int col) const {
  return in_bounds(row, col) && !free_[static_cast<size_t>(row) * static_cast<size_t>(width_) +
                                      static_cast<size_t>(col)];
}

core::Point OccupancyGrid2D::cell_to_world(int row, int col) const {
  double x = origin_[0] + (static_cast<double>(col) + 0.5) * resolution_;
  double y =
      origin_[1] + ((static_cast<double>(height_) - 1.0 - static_cast<double>(row)) + 0.5) * resolution_;
  return core::Point{x, y};
}

core::Cell OccupancyGrid2D::world_to_cell(double x, double y) const {
  int col = static_cast<int>(std::floor((x - origin_[0]) / resolution_));
  int row = static_cast<int>(height_ - 1) -
            static_cast<int>(std::floor((y - origin_[1]) / resolution_));
  return core::Cell{row, col};
}

}  // namespace slam::maps
