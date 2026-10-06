#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "slam/maps/occupancy_grid.hpp"

namespace slam::test {

// Builds a grid from an ASCII layout: '.' = free (255), '#' = occupied (0). rows[0]
// is the TOP image row. resolution default 1.0, origin (0,0) — mirrors the Python
// conftest helper so both languages test against identical grids.
inline slam::maps::OccupancyGrid2D make_grid(const std::vector<std::string>& rows,
                                             double resolution = 1.0) {
  int h = static_cast<int>(rows.size());
  int w = rows.empty() ? 0 : static_cast<int>(rows[0].size());
  std::vector<std::uint16_t> pixels(static_cast<size_t>(h) * static_cast<size_t>(w));
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      pixels[static_cast<size_t>(r) * static_cast<size_t>(w) + static_cast<size_t>(c)] =
          rows[static_cast<size_t>(r)][static_cast<size_t>(c)] == '.' ? 255 : 0;
    }
  }
  return slam::maps::OccupancyGrid2D(std::move(pixels), h, w, resolution, {0.0, 0.0});
}

// Writes content to a unique temp file and returns its path (auto-cleaned by the OS
// temp dir). Used to exercise loaders/validators against real file contents.
inline std::string write_temp(const std::string& name, const std::string& content) {
  static std::atomic<int> counter{0};
  std::filesystem::path dir = std::filesystem::temp_directory_path() / "slam_tests";
  std::filesystem::create_directories(dir);
  std::filesystem::path p = dir / (std::to_string(counter++) + "_" + name);
  std::ofstream(p) << content;
  return p.string();
}

inline std::string repo_path(const std::string& rel) {
  return std::string(SLAM_REPO_DIR) + "/" + rel;
}

}  // namespace slam::test
