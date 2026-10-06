#pragma once

#include <optional>
#include <vector>

#include "slam/core/types.hpp"

namespace slam::core {

// The simulator — GT trajectory resampling, odometry noise, DDA raycast, landmark
// observations. Part of the CONTRACT (core), not an algorithm: every beam scan and
// landmark observation an estimator ever sees is produced here, bit-identically in
// both languages, from the scenario's waypoints + seed (spec/data_formats.md).
//
// Draw order (determinism contract): at each step t draw the OBSERVATION noise
// first (beam: beams 0..beams-1 in order; landmarks: id ascending, range then
// bearing — invisible landmarks consume no draws), THEN the odometry noise
// (ex, ey, etheta) for the NEXT move. Step 0 has no arriving command so it carries
// no odom field and the last step draws no odometry.

// Equal-arc resampling of the waypoint polyline (contract-exact): points sit at arc
// lengths 0, s, 2s, ... while k*s < L (strictly less — a final point exactly on L is
// not duplicated), plus one last point at the polyline end when leftover length
// remains. Heading theta_k = atan2 of the segment p_k -> p_{k+1} over consecutive
// RESAMPLED points (a corner's whole rotation lands in one step's dtheta); the final
// point keeps the previous heading.
std::vector<Pose> resample(const std::vector<Point>& path, double step_meters);

// DDA ray from (x, y) at angle phi over the grid; returns the distance to the first
// occupied cell's closed boundary (< range_max) or nullopt for a miss. Fixed
// formulas (both languages compute identically): cdx = cos(phi), sdy = sin(phi); an
// axis never steps when its cosine is 0 (tMax = inf). A tie (tMaxX == tMaxY) takes
// the Y step. Starting inside an occupied cell hits at distance 0; leaving the grid
// bounds misses.
std::optional<double> raycast(const ScanGrid& grid, double x, double y, double phi,
                              double range_max);

// True iff the segment from_pt -> lm crosses NO occupied cell square (the contract's
// visibility predicate; bbox-rejected cells are skipped exactly).
bool landmark_visible(const ScanGrid& grid, const Point& from_pt, const Point& lm);

// Assemble the full Step stream from a scenario: resample GT poses, then per step
// draw observation noise first and odometry noise for the NEXT move second;
// Step(t).odom is the command that arrived at t. `landmarks` = nullptr for beam
// scenarios (the pointer mirrors Python's None).
Episode build_episode(const ScanGrid& grid, const std::vector<Point>& path, double step_meters,
                      const SensorConfig& sensor, const std::vector<Point>* landmarks,
                      double sigma_xy, double sigma_theta, long long seed);

}  // namespace slam::core
