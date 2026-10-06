#include "slam/filtering/histogram_filter.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "slam/core/geometry.hpp"
#include "slam/core/sim.hpp"

// Bit-identical mirror of the Python module. exp/atan2 bind directly to libSystem's
// scalar exp/atan2 (like log — no same-argument pair fold exists for them; only the
// (sin, cos) pair needs core/libm.hpp routing, and that lives inside pose_compose).
// sqrt/floor are hardware instructions in both languages.

namespace slam::filtering {

namespace {

// Python's % is floor-mod: a negative left operand wraps positive. C++'s % does not;
// this helper IS the mirrored operator (used for the H-table rotation index).
int fmod_int(int x, int m) {
  int r = x % m;
  return r < 0 ? r + m : r;
}

}  // namespace

using slam::core::Cell;
using slam::core::CellProb;
using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::Point;
using slam::core::Pose;
using slam::core::ScanGrid;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

HistogramFilter::HistogramFilter(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)) {
  p_slip_ = params_.get_float("p_slip");
  sigma_range_ = params_.get_float("sigma_range");
  beams_ = params_.get_int("beams");
  fov_deg_ = params_.get_float("fov_deg");
  range_max_ = params_.get_float("range_max");
}

void HistogramFilter::init_state() {
  // Uniform prior over free cells x the heading lattice + the ray table. The
  // heading lattice is the beam grid extended to the full circle: bin b's center
  // θ_b = −π + b·w equals a possible beam direction, so a state (cell fi, heading
  // θ_b) expects exactly R[fi][(b + j − off) mod B] — the ray the simulator fires
  // when the true pose IS that state. One DDA per (free cell, lattice angle),
  // ascending (fi, k); a miss stores the sentinel range_max + resolution.
  const ScanGrid& grid = *episode_->grid;
  int h = grid.height();
  int w = grid.width();
  double res = grid.resolution();

  double fov = fov_deg_ * (kPi / 180.0);  // == Python math.radians(fov_deg)
  double step_a = fov / static_cast<double>(beams_ - 1);
  double half = fov / 2.0;
  int b_bins = static_cast<int>(std::nearbyint(2.0 * kPi / step_a));  // round-half-even, like Python
  int off = (beams_ - 1) / 2;
  // The config must keep the lattice aligned: B steps of w close the circle and
  // off steps of w land exactly on half the field of view. (Python asserts these.)
  if (!(std::abs(static_cast<double>(b_bins) * step_a - 2.0 * kPi) < 1e-9)) {
    throw std::runtime_error("histogram_filter: heading lattice not closed");
  }
  if (!(std::abs(static_cast<double>(off) * step_a - half) < 1e-9)) {
    throw std::runtime_error("histogram_filter: heading lattice not aligned to beams");
  }

  std::vector<Point> centers;
  std::vector<int> fi_of_cell(static_cast<size_t>(h) * static_cast<size_t>(w), -1);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (grid.occupied(r, c)) continue;
      fi_of_cell[static_cast<size_t>(r) * static_cast<size_t>(w) + static_cast<size_t>(c)] =
          static_cast<int>(centers.size());
      centers.push_back(grid.cell_to_world(r, c));
    }
  }

  w_bin_ = step_a;
  bins_ = b_bins;
  half_ = half;
  off_ = off;
  free_cells_.clear();
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (fi_of_cell[static_cast<size_t>(r) * static_cast<size_t>(w) + static_cast<size_t>(c)] >= 0) {
        free_cells_.push_back(Cell{r, c});
      }
    }
  }
  fi_of_cell_ = std::move(fi_of_cell);
  centers_ = std::move(centers);
  grid_h_ = h;
  grid_w_ = w;
  int n_states = static_cast<int>(centers_.size()) * b_bins;
  n_states_ = n_states;

  // R: one DDA per (free cell, lattice angle), ascending.
  std::vector<double> bin_centers(static_cast<size_t>(b_bins));
  for (int k = 0; k < b_bins; ++k) {
    bin_centers[static_cast<size_t>(k)] = -kPi + static_cast<double>(k) * step_a;
  }
  double miss = range_max_ + res;
  std::vector<double> r_flat;
  for (const Point& c : centers_) {
    for (int k = 0; k < b_bins; ++k) {
      std::optional<double> hit = slam::core::raycast(grid, c.x, c.y, bin_centers[static_cast<size_t>(k)],
                                                      range_max_);
      r_flat.push_back(hit.has_value() ? *hit : miss);
    }
  }

  // H[j] per beam j: the same numbers gathered through the rotation identity
  // H[j][fi*B + b] = R[fi][(b + j − off) mod B] (ascending s order).
  h_tables_.assign(static_cast<size_t>(beams_), std::vector<double>(static_cast<size_t>(n_states), 0.0));
  for (int j = 0; j < beams_; ++j) {
    std::vector<double>& col = h_tables_[static_cast<size_t>(j)];
    size_t i = 0;
    for (size_t fi = 0; fi < centers_.size(); ++fi) {
      size_t base = fi * static_cast<size_t>(b_bins);
      for (int b = 0; b < b_bins; ++b, ++i) {
        col[i] = r_flat[base + static_cast<size_t>(fmod_int(b + j - off, b_bins))];
      }
    }
  }

  double p0 = 1.0 / static_cast<double>(n_states);
  belief_.assign(static_cast<size_t>(n_states), p0);
  initialized_ = true;
}

void HistogramFilter::predict(const Twist& u) {
  // Child-of-movement per state + slip mix, ascending s order. A hypothesis whose
  // commanded move lands on an occupied cell or off the grid does not move (slide
  // semantics — its heading still rotates). Exact-zero beliefs are skipped; adding
  // 0.0 could not change any sum anyway.
  double w_bin = w_bin_;
  int bins = bins_;
  const ScanGrid& grid = *episode_->grid;
  int w = grid_w_;
  double p_stay = p_slip_;
  std::vector<double> new_p(static_cast<size_t>(n_states_), 0.0);
  for (size_t fi = 0; fi < free_cells_.size(); ++fi) {
    Cell cell = free_cells_[fi];
    Point c = centers_[fi];
    size_t base = fi * static_cast<size_t>(bins);
    for (int b = 0; b < bins; ++b) {
      double p = belief_[base + static_cast<size_t>(b)];
      if (p == 0.0) continue;  // exact zero — adding it could not change any sum
      Pose moved = slam::core::pose_compose(
          Pose{c.x, c.y, -kPi + static_cast<double>(b) * w_bin}, u);
      Cell qc = grid.world_to_cell(moved.x, moved.y);
      int qb = static_cast<int>(std::floor((moved.theta + kPi) / w_bin + 0.5));
      if (qb >= bins) qb -= bins;
      int q_ci = cell.row * w + cell.col;
      // Slide semantics: off-grid or occupied target => the move does not move.
      if (grid.in_bounds(qc.row, qc.col) &&
          fi_of_cell_[static_cast<size_t>(qc.row) * static_cast<size_t>(w) + static_cast<size_t>(qc.col)] >= 0) {
        q_ci = qc.row * w + qc.col;
      }
      size_t q = static_cast<size_t>(fi_of_cell_[static_cast<size_t>(q_ci)]) *
                     static_cast<size_t>(bins) +
                 static_cast<size_t>(qb);
      new_p[q] += (1.0 - p_stay) * p;
      new_p[base + static_cast<size_t>(b)] += p_stay * p;
    }
  }
  belief_ = std::move(new_p);
}

void HistogramFilter::update_scan(const std::vector<Point>& scan) {
  // Bayes product with the prior (observation-major, state-minor — fixed order).
  // Every observed point contributes −½·((r − H[j][s])/σ)² to every state at once;
  // the exponential itself is scalar exp so both languages hit the same libm.
  // States whose prior is exactly 0 skip the exp (their weight is +0.0 either way,
  // bit-identically).
  double sigma = sigma_range_;
  size_t n = static_cast<size_t>(n_states_);
  std::vector<double> ll(n, 0.0);
  for (const Point& z : scan) {
    double r = std::sqrt(z.x * z.x + z.y * z.y);
    double beta = std::atan2(z.y, z.x);
    int j = static_cast<int>(std::floor((beta + half_) / w_bin_ + 0.5));
    if (j < 0) j = 0;
    if (j > beams_ - 1) j = beams_ - 1;
    const std::vector<double>& col = h_tables_[static_cast<size_t>(j)];
    for (size_t s = 0; s < n; ++s) {
      double d = (r - col[s]) / sigma;
      ll[s] += (-0.5 * d) * d;
    }
  }
  // max is exact — order-independent by construction.
  double m = *std::max_element(ll.begin(), ll.end());
  std::vector<double> w_list;
  w_list.reserve(n);
  double total = 0.0;
  for (size_t s = 0; s < n; ++s) {
    double p_prior = belief_[s];
    double wv = 0.0;
    if (p_prior != 0.0) wv = p_prior * std::exp(ll[s] - m);
    w_list.push_back(wv);
    total += wv;
  }
  if (total != 0.0) {
    std::vector<double> next(n);
    for (size_t s = 0; s < n; ++s) next[s] = w_list[s] / total;
    belief_ = std::move(next);
  }
}

void HistogramFilter::update(const Step& step, TraceRecorder* recorder) {
  if (!initialized_) init_state();
  if (step.has_odom) predict(step.odom);
  if (step.has_scan && !step.scan.empty()) update_scan(step.scan);

  int bins = bins_;
  const std::vector<double>& belief = belief_;

  // Readout (fixed order): position mean ascending over all states, then the
  // bin-marginal argmax (first max wins), then population std devs.
  double x_hat = 0.0;
  double y_hat = 0.0;
  std::vector<double> bin_marginal(static_cast<size_t>(bins), 0.0);
  for (size_t fi = 0; fi < free_cells_.size(); ++fi) {
    Point c = centers_[fi];
    size_t base = fi * static_cast<size_t>(bins);
    for (int b = 0; b < bins; ++b) {
      double p = belief[base + static_cast<size_t>(b)];
      x_hat += p * c.x;
      y_hat += p * c.y;
      bin_marginal[static_cast<size_t>(b)] += p;
    }
  }
  double theta_hat = -kPi;
  double best = -std::numeric_limits<double>::infinity();
  for (int b = 0; b < bins; ++b) {
    if (bin_marginal[static_cast<size_t>(b)] > best) {
      best = bin_marginal[static_cast<size_t>(b)];
      theta_hat = -kPi + static_cast<double>(b) * w_bin_;
    }
  }

  double var_x = 0.0;
  double var_y = 0.0;
  double var_t = 0.0;
  for (size_t fi = 0; fi < free_cells_.size(); ++fi) {
    Point c = centers_[fi];
    size_t base = fi * static_cast<size_t>(bins);
    for (int b = 0; b < bins; ++b) {
      double p = belief[base + static_cast<size_t>(b)];
      double dx = c.x - x_hat;
      double dy = c.y - y_hat;
      double dt = slam::core::wrap(-kPi + static_cast<double>(b) * w_bin_ - theta_hat);
      var_x += p * (dx * dx);
      var_y += p * (dy * dy);
      var_t += p * (dt * dt);
    }
  }
  Pose pose{x_hat, y_hat, theta_hat};
  std::array<double, 3> cov{std::sqrt(var_x), std::sqrt(var_y), std::sqrt(var_t)};

  if (recorder != nullptr) {
    // EVERY cell row-major ([row, col, p] per the trace contract); occupied cells
    // are not states and emit exactly 0.0.
    std::vector<CellProb> cells;
    for (int ci = 0; ci < grid_h_ * grid_w_; ++ci) {
      int fi = fi_of_cell_[static_cast<size_t>(ci)];
      int r0 = ci / grid_w_;
      int c0 = ci % grid_w_;
      double p_cell = 0.0;
      if (fi >= 0) {
        size_t base = static_cast<size_t>(fi) * static_cast<size_t>(bins);
        for (int b = 0; b < bins; ++b) p_cell += belief[base + static_cast<size_t>(b)];
      }
      cells.push_back(CellProb{r0, c0, p_cell});
    }
    recorder->pose_estimated(step.t, pose, cov);
    recorder->belief_updated(step.t, cells);
  }
  poses_.push_back(pose);
}

EstimateResult HistogramFilter::finalize(TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  return result;
}

}  // namespace slam::filtering
