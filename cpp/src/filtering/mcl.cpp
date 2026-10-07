#include "slam/filtering/mcl.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"
#include "slam/core/sim.hpp"
#include "slam/core/stats.hpp"

// Bit-identical mirror of the Python module. exp/atan2 bind directly to libSystem's
// scalar exp/atan2 (like log — no same-argument pair fold exists for them); the
// readout's sin/cos PAIR on each θ routes through core/libm.hpp (see there for
// why), and raycast already routes its own cos/sin. sqrt/floor are hardware
// instructions in both languages. Every float op rounds exactly where the Python
// line does: individual multiply/add/subtract, no fusion (-ffp-contract=off).

namespace slam::filtering {

using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::Particle;
using slam::core::Point;
using slam::core::Pose;
using slam::core::ScanGrid;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

double kld_bound(int k, double epsilon, double z_q) {
  if (k <= 1) return 0.0;  // degenerate: a one-bin belief needs no bound beyond n_min
  const double nu = static_cast<double>(k - 1);
  const double q9 = 2.0 / (9.0 * nu);
  const double t = (1.0 - q9) + z_q * std::sqrt(q9);
  return (nu / (2.0 * epsilon)) * t * t * t;  // left-associative t·t·t — fixed order
}

Mcl::Mcl(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)),
      x0_(params_.get_float("x0")),
      y0_(params_.get_float("y0")),
      epsilon_(params_.get_float("epsilon")),
      delta_(params_.get_float("delta")),
      bin_xy_(params_.get_float("bin_xy")),
      bin_theta_(params_.get_float("bin_theta")),
      n_min_(static_cast<int>(params_.get_int("n_min"))),
      max_particles_(static_cast<int>(params_.get_int("max_particles"))),
      sigma_range_(params_.get_float("sigma_range")),
      sigma_xy_(params_.get_float("sigma_xy")),
      sigma_theta_(params_.get_float("sigma_theta")),
      range_max_(params_.get_float("range_max")),
      rng_(params_.get_int("seed")) {  // the demo injected the scenario's seed here
  // The bound's quantile is a constant of the config — computed ONCE, here.
  z_q_ = slam::core::inv_norm_cdf(1.0 - delta_);
}

void Mcl::init_particles() {
  // Draw max_particles particles in ascending order (the PRIOR is fixed-size —
  // adaptivity starts at the first KLD step): the start cell (the one holding
  // x0/y0) uniformly in x and y, heading uniform on [−π, π). Three draws per
  // particle — x offset, y offset, heading — from the algorithm's own stream.
  const ScanGrid& grid = *episode_->grid;
  const int h = grid.height();
  const std::array<double, 2> origin = grid.origin();
  const double ox = origin[0];
  const double oy = origin[1];
  const double res = grid.resolution();
  const slam::core::Cell start = grid.world_to_cell(x0_, y0_);
  for (int i = 0; i < max_particles_; ++i) {
    x_.push_back(ox + (static_cast<double>(start.col) + rng_.uniform01()) * res);
    y_.push_back(oy + (static_cast<double>(h - 1 - start.row) + rng_.uniform01()) * res);
    theta_.push_back(rng_.uniform01() * (2.0 * kPi) - kPi);
  }
  // Uniform prior — exactly 1/N each.
  w_.assign(static_cast<size_t>(max_particles_), 1.0 / static_cast<double>(max_particles_));
  initialized_ = true;
}

std::vector<Point> Mcl::polar_points(const std::vector<Point>& scan) {
  // (r, β) per scan point, in scan order — the polar form of each robot-frame
  // endpoint. Fixed formulas: sqrt(zx² + zy²), atan2(zy, zx).
  std::vector<Point> points;
  points.reserve(scan.size());
  for (const Point& z : scan) {
    points.push_back(Point{std::sqrt(z.x * z.x + z.y * z.y), std::atan2(z.y, z.x)});
  }
  return points;
}

void Mcl::weight(const std::vector<Point>& points) {
  // t = 0 only: multiply every uniform weight by exp(ll − max ll); renormalize.
  // Same fixed pattern as particle_filter: ll is the sum over this step's scan
  // points IN SCAN ORDER of the beam-model Gaussian −½·((r − expected)/σ)²; a miss
  // keeps the sentinel range_max + res; exp never overflows after the max shift.
  const ScanGrid& grid = *episode_->grid;
  const double sentinel = range_max_ + grid.resolution();
  const int n = static_cast<int>(w_.size());

  double m = -std::numeric_limits<double>::infinity();
  std::vector<double> ll;
  ll.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const size_t k = static_cast<size_t>(i);
    double acc = 0.0;
    for (const Point& p : points) {
      std::optional<double> e = slam::core::raycast(grid, x_[k], y_[k], theta_[k] + p.y,
                                                    range_max_);
      const double expected = e.has_value() ? *e : sentinel;
      const double d = (p.x - expected) / sigma_range_;
      acc += (-0.5 * d) * d;
    }
    ll.push_back(acc);
    if (acc > m) m = acc;  // max is exact — order-independent by construction
  }

  double total = 0.0;
  std::vector<double> weighted;
  weighted.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const size_t k = static_cast<size_t>(i);
    const double wv = w_[k] * std::exp(ll[k] - m);
    weighted.push_back(wv);
    total += wv;
  }
  if (total != 0.0) {  // a total of 0.0 leaves the belief unchanged (documented case)
    for (int i = 0; i < n; ++i) {
      const size_t k = static_cast<size_t>(i);
      w_[k] = weighted[k] / total;
    }
  }
}

void Mcl::kld_update(const Twist& u, const std::vector<Point>& scan) {
  // One KLD-sampling step: resample-move-reweight ONE sample at a time until the
  // bound certifies coverage (or the cap). Per sample, fixed order: ancestor walk
  // over the previous weights (ONE uniform draw, strict `>`, guard i < n−1 — this
  // is plain multinomial resampling; KLD changes WHEN it stops, not HOW it picks),
  // then three gaussian draws and x ⊕ (u + ε); then the arriving scan's likelihood
  // at the new pose (scan order, sentinel on a miss) and the bin key — a first-seen
  // bin bumps k and recomputes n_chi. The loop stops when n ≥ n_chi AND n ≥ n_min,
  // or at max_particles.
  const ScanGrid& grid = *episode_->grid;
  const double sentinel = range_max_ + grid.resolution();
  std::vector<Point> points;
  if (!scan.empty()) points = polar_points(scan);

  const int n_old = static_cast<int>(w_.size());
  std::vector<double> x_new, y_new, th_new, ll_new;
  std::set<std::tuple<int, int, int>> seen;
  int k = 0;
  int n = 0;
  double m = -std::numeric_limits<double>::infinity();
  double n_chi = std::numeric_limits<double>::infinity();
  while (true) {
    const double u1 = rng_.uniform01();
    int i = 0;
    double c = w_[0];
    while (i < n_old - 1 && u1 > c) {
      i += 1;
      c += w_[static_cast<size_t>(i)];
    }
    const double ex = rng_.gaussian(0.0, sigma_xy_);
    const double ey = rng_.gaussian(0.0, sigma_xy_);
    const double et = rng_.gaussian(0.0, sigma_theta_);
    const size_t old_i = static_cast<size_t>(i);
    const Pose p = slam::core::pose_compose(
        Pose{x_[old_i], y_[old_i], theta_[old_i]}, Twist{u.dx + ex, u.dy + ey, u.dtheta + et});
    double acc = 0.0;
    for (const Point& pt : points) {
      std::optional<double> e =
          slam::core::raycast(grid, p.x, p.y, p.theta + pt.y, range_max_);
      const double expected = e.has_value() ? *e : sentinel;
      const double d = (pt.x - expected) / sigma_range_;
      acc += (-0.5 * d) * d;
    }
    x_new.push_back(p.x);
    y_new.push_back(p.y);
    th_new.push_back(p.theta);
    ll_new.push_back(acc);
    if (acc > m) m = acc;  // max is exact — order-independent by construction
    const std::tuple<int, int, int> key{static_cast<int>(std::floor(p.x / bin_xy_)),
                                        static_cast<int>(std::floor(p.y / bin_xy_)),
                                        static_cast<int>(std::floor(p.theta / bin_theta_))};
    if (seen.insert(key).second) k += 1;
    n += 1;
    if (n >= n_min_) n_chi = kld_bound(k, epsilon_, z_q_);
    if ((static_cast<double>(n) >= n_chi && n >= n_min_) || n >= max_particles_) break;
  }

  double total = 0.0;
  std::vector<double> weighted;
  weighted.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double wv = std::exp(ll_new[static_cast<size_t>(i)] - m);
    weighted.push_back(wv);
    total += wv;
  }
  // The old weights already acted — they drove the ancestor walk.
  w_.clear();
  w_.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    w_.push_back(weighted[static_cast<size_t>(i)] / total);
  }
  x_ = std::move(x_new);
  y_ = std::move(y_new);
  theta_ = std::move(th_new);
}

void Mcl::update(const Step& step, TraceRecorder* recorder) {
  // t = 0 initializes the fixed-size prior cloud and weights it by the first scan;
  // every later step with an arriving command runs one KLD resample-move-reweight.
  // Then read out — pose first, cloud second (identical to the particle_filter
  // page's readout).
  const bool first = !initialized_;
  if (first) {
    init_particles();
    if (step.has_scan && !step.scan.empty()) weight(polar_points(step.scan));
  } else if (step.has_odom) {
    kld_update(step.odom, step.scan);
  }

  // Readout (fixed order): weighted mean x, y ascending; then the circular heading
  // from the sin/cos sums; then population variances in a second ascending pass
  // (θ's deviation wrapped around the seam).
  double x_hat = 0.0, y_hat = 0.0, s_sin = 0.0, s_cos = 0.0;
  const int n = static_cast<int>(w_.size());
  for (int i = 0; i < n; ++i) {
    const size_t k = static_cast<size_t>(i);
    const double wv = w_[k];
    x_hat += wv * x_[k];
    y_hat += wv * y_[k];
    s_sin += wv * slam::core::libm_sin(theta_[k]);
    s_cos += wv * slam::core::libm_cos(theta_[k]);
  }
  const double theta_hat = std::atan2(s_sin, s_cos);

  double var_x = 0.0, var_y = 0.0, var_t = 0.0;
  for (int i = 0; i < n; ++i) {
    const size_t k = static_cast<size_t>(i);
    const double wv = w_[k];
    const double dx = x_[k] - x_hat;
    const double dy = y_[k] - y_hat;
    const double dt = slam::core::wrap(theta_[k] - theta_hat);
    var_x += wv * (dx * dx);
    var_y += wv * (dy * dy);
    var_t += wv * (dt * dt);
  }
  std::array<double, 3> cov{std::sqrt(var_x), std::sqrt(var_y), std::sqrt(var_t)};

  const Pose pose{x_hat, y_hat, theta_hat};
  poses_.push_back(pose);
  if (recorder != nullptr) {
    recorder->pose_estimated(step.t, pose, cov);
    std::vector<Particle> cloud;
    cloud.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const size_t k = static_cast<size_t>(i);
      cloud.push_back(Particle{x_[k], y_[k], theta_[k], w_[k]});
    }
    recorder->particles_updated(step.t, cloud);
  }
}

EstimateResult Mcl::finalize(TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  return result;
}

}  // namespace slam::filtering
