#include "slam/core/sim.hpp"

#include <cmath>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"
#include "slam/core/rng.hpp"

namespace slam::core {
namespace {

// The polyline point at arc length s (fixed walk: first segment whose remaining
// length covers the remainder, linear interpolation on it).
Point point_at_arc(const std::vector<Point>& path, const std::vector<double>& seg_len, double s) {
  double acc = 0.0;
  for (size_t k = 0; k < seg_len.size(); ++k) {
    if (s - acc <= seg_len[k]) {
      double frac = (s - acc) / seg_len[k];
      double ax = path[k].x, ay = path[k].y;
      double bx = path[k + 1].x, by = path[k + 1].y;
      return Point{ax + frac * (bx - ax), ay + frac * (by - ay)};
    }
    acc += seg_len[k];
  }
  // s >= total: the polyline end.
  return path.back();
}

// One step's observation, drawn in contract order (see module header).
void observe(Rng& rng, const ScanGrid& grid, const Pose& gt, const SensorConfig& sensor,
             const std::vector<Point>* landmarks, Step& step) {
  if (sensor.type == "beam") {
    double fov = sensor.fov_deg * (kPi / 180.0);
    double half = fov / 2.0;
    double step_a = fov / static_cast<double>(sensor.beams - 1);
    for (int i = 0; i < sensor.beams; ++i) {
      double phi = gt.theta - half + static_cast<double>(i) * step_a;
      std::optional<double> r_hit = raycast(grid, gt.x, gt.y, phi, sensor.range_max);
      if (!r_hit.has_value()) continue;  // miss emits no point
      double r_noisy = *r_hit + rng.gaussian(0.0, sensor.sigma_range);
      // libm_cos/libm_sin: the same-argument pair is exactly what Apple clang
      // folds into __sincos_stret (see core/libm.hpp).
      Point e{gt.x + libm_cos(phi) * r_noisy, gt.y + libm_sin(phi) * r_noisy};
      step.scan.push_back(world_to_robot(e, gt));
    }
    step.has_scan = true;
    return;
  }
  // landmarks: id ascending; range noise then bearing noise per visible landmark.
  for (size_t lm_id = 0; lm_id < landmarks->size(); ++lm_id) {
    const Point& lm = (*landmarks)[lm_id];
    double dx = lm.x - gt.x;
    double dy = lm.y - gt.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > sensor.range_max || !landmark_visible(grid, Point{gt.x, gt.y}, lm)) continue;
    double r_noisy = dist + rng.gaussian(0.0, sensor.sigma_range);
    double bearing_exact = wrap(std::atan2(dy, dx) - gt.theta);
    double b_noisy = wrap(bearing_exact + rng.gaussian(0.0, sensor.sigma_bearing));
    step.obs.push_back(LandmarkObs{static_cast<int>(lm_id), b_noisy, r_noisy});
  }
  step.has_obs = true;
}

}  // namespace

std::vector<Pose> resample(const std::vector<Point>& path, double step_meters) {
  std::vector<double> seg_len;
  double total = 0.0;
  for (size_t k = 0; k + 1 < path.size(); ++k) {
    double dx = path[k + 1].x - path[k].x;
    double dy = path[k + 1].y - path[k].y;
    double seg_l = std::sqrt(dx * dx + dy * dy);
    seg_len.push_back(seg_l);
    total += seg_l;
  }

  std::vector<Point> points;
  double s = 0.0;
  while (s < total) {
    points.push_back(point_at_arc(path, seg_len, s));
    s += step_meters;
  }
  // The polyline end is always a point (its step may be shorter than the spacing).
  points.push_back(Point{path.back().x, path.back().y});

  std::vector<Pose> poses;
  for (size_t k = 0; k < points.size(); ++k) {
    double theta;
    if (k + 1 < points.size()) {
      theta = std::atan2(points[k + 1].y - points[k].y, points[k + 1].x - points[k].x);
    } else {
      // The final point keeps the previous heading (a single-waypoint path has no
      // segment at all and keeps 0.0 — Python raises there; scenarios always carry ≥2).
      theta = poses.empty() ? 0.0 : poses.back().theta;
    }
    poses.push_back(Pose{points[k].x, points[k].y, theta});
  }
  return poses;
}

std::optional<double> raycast(const ScanGrid& grid, double x, double y, double phi,
                              double range_max) {
  // libm_cos/libm_sin (not std): same-argument pair — see core/libm.hpp.
  double cdx = libm_cos(phi);
  double sdy = libm_sin(phi);
  Cell cell = grid.world_to_cell(x, y);
  int r = cell.row, c = cell.col;
  if (!grid.in_bounds(r, c)) return std::nullopt;
  if (grid.occupied(r, c)) return 0.0;
  std::array<double, 2> origin = grid.origin();
  double ox = origin[0], oy = origin[1];
  double res = grid.resolution();
  int h = grid.height();

  const double inf = INFINITY;
  int step_x;
  double t_max_x;
  if (cdx > 0.0) {
    step_x = 1;
    t_max_x = (ox + (static_cast<double>(c) + 1.0) * res - x) / cdx;
  } else if (cdx < 0.0) {
    step_x = -1;
    t_max_x = (ox + static_cast<double>(c) * res - x) / cdx;
  } else {
    step_x = 0;
    t_max_x = inf;
  }
  int step_y;
  double t_max_y;
  if (sdy > 0.0) {
    step_y = -1;  // world y grows DOWNWARD in row index
    // Row r's TOP edge is at oy + (h - r) * res (row 0 is the top image row).
    t_max_y = (oy + (static_cast<double>(h) - static_cast<double>(r)) * res - y) / sdy;
  } else if (sdy < 0.0) {
    step_y = 1;
    // Row r's BOTTOM edge is at oy + (h - 1 - r) * res.
    t_max_y =
        (oy + (static_cast<double>(h) - 1.0 - static_cast<double>(r)) * res - y) / sdy;
  } else {
    step_y = 0;
    t_max_y = inf;
  }

  double delta_x = cdx != 0.0 ? res / std::abs(cdx) : inf;
  double delta_y = sdy != 0.0 ? res / std::abs(sdy) : inf;

  while (true) {
    double hit_t;
    if (t_max_x < t_max_y) {
      hit_t = t_max_x;
      if (hit_t >= range_max) return std::nullopt;
      c += step_x;
      t_max_x += delta_x;
    } else {
      hit_t = t_max_y;
      if (hit_t >= range_max) return std::nullopt;
      r += step_y;
      t_max_y += delta_y;
    }
    if (!grid.in_bounds(r, c)) return std::nullopt;
    if (grid.occupied(r, c)) return hit_t;
  }
}

bool landmark_visible(const ScanGrid& grid, const Point& from_pt, const Point& lm) {
  double min_x = std::min(from_pt.x, lm.x);
  double max_x = std::max(from_pt.x, lm.x);
  double min_y = std::min(from_pt.y, lm.y);
  double max_y = std::max(from_pt.y, lm.y);
  std::array<double, 2> origin = grid.origin();
  double ox = origin[0], oy = origin[1];
  double res = grid.resolution();
  int h = grid.height(), w = grid.width();
  for (int r = 0; r < h; ++r) {
    double y_lo = oy + (static_cast<double>(h) - 1.0 - static_cast<double>(r)) * res;
    double y_hi = oy + (static_cast<double>(h) - static_cast<double>(r)) * res;
    if (y_lo > max_y || y_hi < min_y) continue;
    for (int c = 0; c < w; ++c) {
      double x_lo = ox + static_cast<double>(c) * res;
      double x_hi = ox + (static_cast<double>(c) + 1.0) * res;
      if (x_lo > max_x || x_hi < min_x) continue;
      if (grid.occupied(r, c) && segment_intersects_rect(from_pt, lm, Rect{x_lo, y_lo, x_hi, y_hi})) {
        return false;
      }
    }
  }
  return true;
}

Episode build_episode(const ScanGrid& grid, const std::vector<Point>& path, double step_meters,
                      const SensorConfig& sensor, const std::vector<Point>* landmarks,
                      double sigma_xy, double sigma_theta, long long seed) {
  std::vector<Pose> poses = resample(path, step_meters);
  Rng rng(seed);
  Episode ep;
  if (landmarks != nullptr) {
    ep.has_landmarks = true;
    ep.landmarks = *landmarks;
  }
  ep.grid = &grid;
  bool has_odom = false;
  Twist u_arriving{0.0, 0.0, 0.0};
  int last = static_cast<int>(poses.size()) - 1;
  for (int t = 0; t < static_cast<int>(poses.size()); ++t) {
    Step step;
    step.t = t;
    step.gt = poses[static_cast<size_t>(t)];
    observe(rng, grid, step.gt, sensor, landmarks, step);
    if (has_odom) {
      step.odom = u_arriving;
      step.has_odom = true;
    }
    ep.steps.push_back(step);
    if (t < last) {
      Twist u_exact = pose_minus(poses[static_cast<size_t>(t)], poses[static_cast<size_t>(t) + 1]);
      double ex = rng.gaussian(0.0, sigma_xy);
      double ey = rng.gaussian(0.0, sigma_xy);
      double etheta = rng.gaussian(0.0, sigma_theta);
      u_arriving = Twist{u_exact.dx + ex, u_exact.dy + ey, wrap(u_exact.dtheta + etheta)};
      has_odom = true;
    }
  }
  return ep;
}

}  // namespace slam::core
