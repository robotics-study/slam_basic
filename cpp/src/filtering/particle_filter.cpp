#include "slam/filtering/particle_filter.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"
#include "slam/core/sim.hpp"

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

ParticleFilter::ParticleFilter(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)),
      sigma_range_(params_.get_float("sigma_range")),
      sigma_xy_(params_.get_float("sigma_xy")),
      sigma_theta_(params_.get_float("sigma_theta")),
      x0_(params_.get_float("x0")),
      y0_(params_.get_float("y0")),
      range_max_(params_.get_float("range_max")),
      rng_(params_.get_int("seed")) {  // the demo injected the scenario's seed here
  n_ = static_cast<int>(params_.get_int("n_particles"));
  inv_n_ = 1.0 / static_cast<double>(n_);
}

void ParticleFilter::init_particles() {
  // Draw N particles in ascending order: the start cell (the one holding x0/y0)
  // uniformly in x and y, heading uniform on [−π, π). Three draws per particle —
  // x offset, y offset, heading.
  const ScanGrid& grid = *episode_->grid;
  const int h = grid.height();
  const std::array<double, 2> origin = grid.origin();
  const double ox = origin[0];
  const double oy = origin[1];
  const double res = grid.resolution();
  const slam::core::Cell start = grid.world_to_cell(x0_, y0_);
  for (int i = 0; i < n_; ++i) {
    x_.push_back(ox + (static_cast<double>(start.col) + rng_.uniform01()) * res);
    y_.push_back(oy + (static_cast<double>(h - 1 - start.row) + rng_.uniform01()) * res);
    theta_.push_back(rng_.uniform01() * (2.0 * kPi) - kPi);
  }
  w_.assign(static_cast<size_t>(n_), inv_n_);  // uniform prior — exactly 1/N each
  initialized_ = true;
}

void ParticleFilter::move(const Twist& u) {
  // x ⊕ (u + ε): three gaussians per particle (two uniform draws each, fixed order
  // ex, ey, eθ) drawn in ascending particle order.
  for (int i = 0; i < n_; ++i) {
    const double ex = rng_.gaussian(0.0, sigma_xy_);
    const double ey = rng_.gaussian(0.0, sigma_xy_);
    const double et = rng_.gaussian(0.0, sigma_theta_);
    const Pose p = slam::core::pose_compose(
        Pose{x_[static_cast<size_t>(i)], y_[static_cast<size_t>(i)],
             theta_[static_cast<size_t>(i)]},
        Twist{u.dx + ex, u.dy + ey, u.dtheta + et});
    x_[static_cast<size_t>(i)] = p.x;
    y_[static_cast<size_t>(i)] = p.y;
    theta_[static_cast<size_t>(i)] = p.theta;
  }
}

void ParticleFilter::weight(const std::vector<Point>& scan) {
  // Multiply every particle's weight by exp(ll − max ll); renormalize. ll is the
  // sum over this step's scan points IN SCAN ORDER of the beam-model Gaussian
  // −½·((r − expected)/σ)²; `expected` is the same DDA raycast the simulator
  // fired, run at the particle's own pose along heading + β (a miss keeps the
  // sentinel range_max + res — a point exists where the particle sees nothing, so
  // it pays the full Gaussian penalty). exp(ll − max) never overflows and the
  // argmax keeps its weight exactly; a total of 0.0 would make normalization
  // impossible, so the belief simply stays unchanged.
  const ScanGrid& grid = *episode_->grid;
  const double res = grid.resolution();
  const double sigma = sigma_range_;
  const double sentinel = range_max_ + res;

  // (r, β) per scan point, in scan order — the polar form of each robot-frame
  // endpoint. Fixed formulas: sqrt(zx² + zy²), atan2(zy, zx).
  std::vector<Point> points;
  points.reserve(scan.size());
  for (const Point& z : scan) {
    points.push_back(Point{std::sqrt(z.x * z.x + z.y * z.y), std::atan2(z.y, z.x)});
  }

  // Per-particle log-likelihood, ascending particle order.
  double m = -std::numeric_limits<double>::infinity();
  std::vector<double> ll;
  ll.reserve(static_cast<size_t>(n_));
  for (int i = 0; i < n_; ++i) {
    const double s = theta_[static_cast<size_t>(i)];
    double acc = 0.0;
    for (const Point& p : points) {
      std::optional<double> e =
          slam::core::raycast(grid, x_[static_cast<size_t>(i)], y_[static_cast<size_t>(i)],
                              s + p.y, range_max_);
      const double expected = e.has_value() ? *e : sentinel;
      const double d = (p.x - expected) / sigma;
      acc += (-0.5 * d) * d;
    }
    ll.push_back(acc);
    if (acc > m) m = acc;  // max is exact — order-independent by construction
  }

  double total = 0.0;
  std::vector<double> weighted;
  weighted.reserve(static_cast<size_t>(n_));
  for (int i = 0; i < n_; ++i) {
    const double wv = w_[static_cast<size_t>(i)] * std::exp(ll[static_cast<size_t>(i)] - m);
    weighted.push_back(wv);
    total += wv;
  }
  if (total != 0.0) {
    for (int i = 0; i < n_; ++i) {
      w_[static_cast<size_t>(i)] = weighted[static_cast<size_t>(i)] / total;
    }
  }
}

void ParticleFilter::resample_if_effective() {
  // Systematic resampling when ESS = 1/Σw² drops below N/2 (Thrun's pseudocode
  // 2.8): ONE uniform draw scaled into [0, 1/N), then a walk of the cumulative
  // weights picking ancestors — strict `>` at every comparison, and the loop keeps
  // its final (unused) u += 1/N so all languages step alike.
  double ess_den = 0.0;
  for (int i = 0; i < n_; ++i) {
    const double wv = w_[static_cast<size_t>(i)];
    ess_den += wv * wv;
  }
  const double ess = 1.0 / ess_den;
  if (ess < static_cast<double>(n_) / 2.0) {
    double u = rng_.uniform01() * inv_n_;
    int i = 0;
    double c = w_[0];
    std::vector<double> xs, ys, ths;
    xs.reserve(static_cast<size_t>(n_));
    ys.reserve(static_cast<size_t>(n_));
    ths.reserve(static_cast<size_t>(n_));
    for (int j = 0; j < n_; ++j) {
      while (i < n_ - 1 && u > c) {
        i += 1;
        c += w_[static_cast<size_t>(i)];
      }
      xs.push_back(x_[static_cast<size_t>(i)]);
      ys.push_back(y_[static_cast<size_t>(i)]);
      ths.push_back(theta_[static_cast<size_t>(i)]);
      u += inv_n_;  // including the final, unused increment — fixed loop shape
    }
    x_ = std::move(xs);
    y_ = std::move(ys);
    theta_ = std::move(ths);
    w_.assign(static_cast<size_t>(n_), inv_n_);  // equal weights after resampling
  }
}

void ParticleFilter::update(const Step& step, TraceRecorder* recorder) {
  // Move (if a command arrived), weight by the scan, resample when the cloud
  // collapses, then read out — pose first, cloud second.
  if (!initialized_) init_particles();
  if (step.has_odom) move(step.odom);
  if (step.has_scan && !step.scan.empty()) weight(step.scan);
  resample_if_effective();

  // Readout (fixed order): weighted mean x, y ascending; then the circular heading
  // from the sin/cos sums; then population variances in a second ascending pass
  // (θ's deviation wrapped around the seam).
  double x_hat = 0.0, y_hat = 0.0, s_sin = 0.0, s_cos = 0.0;
  for (int i = 0; i < n_; ++i) {
    const size_t k = static_cast<size_t>(i);
    const double wv = w_[k];
    x_hat += wv * x_[k];
    y_hat += wv * y_[k];
    s_sin += wv * slam::core::libm_sin(theta_[k]);
    s_cos += wv * slam::core::libm_cos(theta_[k]);
  }
  const double theta_hat = std::atan2(s_sin, s_cos);

  double var_x = 0.0, var_y = 0.0, var_t = 0.0;
  for (int i = 0; i < n_; ++i) {
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
    cloud.reserve(static_cast<size_t>(n_));
    for (int i = 0; i < n_; ++i) {
      const size_t k = static_cast<size_t>(i);
      cloud.push_back(Particle{x_[k], y_[k], theta_[k], w_[k]});
    }
    recorder->particles_updated(step.t, cloud);
  }
}

EstimateResult ParticleFilter::finalize(TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  return result;
}

}  // namespace slam::filtering
