#include "slam/filter_based/ekf_slam.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "slam/core/geometry.hpp"
#include "slam/core/libm.hpp"

// Bit-identical mirror of the Python module. Every float op rounds exactly where the
// Python line does: individual multiply/add, no fusion (-ffp-contract=off); every
// same-argument (cos, sin) pair routes through core/libm (the dlsym-resolved scalar
// libSystem pair — Apple clang's __sincos_stret fold differs by 1 ulp on rare inputs).
// atan2 and sqrt need no routing (no pair fold exists; sqrt is a hardware instruction).

namespace slam::filter_based {

using slam::core::EstimateResult;
using slam::core::kPi;
using slam::core::LandmarkEstimate;
using slam::core::libm_cos;
using slam::core::libm_sin;
using slam::core::Point;
using slam::core::Pose;
using slam::core::Step;
using slam::core::TraceRecorder;
using slam::core::Twist;

namespace {

using Mat = std::vector<std::vector<double>>;

// Dense matmul with the contract's fixed order: out[i][j] accumulates a[i][k]·b[k][j]
// over k ASCENDING into an accumulator that starts at +0.0. No term is ever skipped —
// the sums themselves are the contract (a multiplied ±0.0 term changes no summand's
// value; sign-of-zero parses equal anyway).
Mat matmul(const Mat& a, const Mat& b) {
  const size_t n = a.size();
  const size_t m = b[0].size();
  Mat out;
  for (size_t i = 0; i < n; ++i) {
    const std::vector<double>& row_a = a[i];
    std::vector<double> row_out;
    for (size_t j = 0; j < m; ++j) {
      double acc = 0.0;
      for (size_t k = 0; k < row_a.size(); ++k) acc += row_a[k] * b[k][j];
      row_out.push_back(acc);
    }
    out.push_back(row_out);
  }
  return out;
}

// out[i][j] = Σ_k a[i][k]·h[j][k], k ASCENDING — product with the TRANSPOSED right
// factor (ΣH in both branches, A Fᵀ in predict, −G B⁻ᵀ at birth).
Mat matmul_at_b(const Mat& a, const Mat& h) {
  const size_t n = a.size();
  const size_t m = h.size();
  Mat out;
  for (size_t i = 0; i < n; ++i) {
    const std::vector<double>& row_a = a[i];
    std::vector<double> row_out;
    for (size_t j = 0; j < m; ++j) {
      double acc = 0.0;
      const std::vector<double>& row_h = h[j];
      for (size_t k = 0; k < row_a.size(); ++k) acc += row_a[k] * row_h[k];
      row_out.push_back(acc);
    }
    out.push_back(row_out);
  }
  return out;
}

// Explicit 2×2 inverse — det and adjugate, no solver anywhere:
// det = m00·m11 − m01·m10; inv = [[m11/det, −m01/det], [−m10/det, m00/det]].
Mat invert2(const Mat& m) {
  const double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  return Mat{{m[1][1] / det, -m[0][1] / det}, {-m[1][0] / det, m[0][0] / det}};
}

}  // namespace

EkfSlam::EkfSlam(slam::core::ParamSet params)
    : slam::core::Estimator(std::move(params)),
      x0_(params_.get_float("x0")),
      y0_(params_.get_float("y0")),
      theta_deg_(params_.get_float("theta_deg")),
      q_pos_(0.0),
      q_theta_(0.0),
      r_range_(0.0),
      r_bearing_(0.0) {
  // Q — the motion noise the filter DECLARES (the same values the scenario injects
  // into its simulator; declaring them is how the algorithm "knows" the model).
  const double sigma_xy = params_.get_float("sigma_xy");
  const double sigma_theta = params_.get_float("sigma_theta");
  q_pos_ = sigma_xy * sigma_xy;
  q_theta_ = sigma_theta * sigma_theta;
  // R — measurement noise, diagonal in MEASUREMENT space (range m, bearing rad):
  // different units, so a diagonal that is anything but isotropic.
  const double sigma_range = params_.get_float("sigma_range");
  const double sigma_bearing = params_.get_float("sigma_bearing");
  r_range_ = sigma_range * sigma_range;
  r_bearing_ = sigma_bearing * sigma_bearing;
}

// predict: x̂ ← x ⊕ u (core pose_compose — the exact inverse of what the simulator
// used to differentiate ground truth), then Σ̄ = FΣFᵀ + Q with F = blockdiag(J, I). J
// is evaluated at the PRIOR mean; every matmul runs in the contract's ascending order.
void EkfSlam::predict(double dx_o, double dy_o, double dtheta_o) {
  const double px = x_[0], py = x_[1], th = x_[2];
  const double c = libm_cos(th);  // same-argument pair — routed through libm
  const double s = libm_sin(th);
  // J = ∂(x ⊕ u)/∂x at the prior mean — fixed expressions.
  const double j02 = -((s * dx_o) + (c * dy_o));
  const double j12 = (c * dx_o) - (s * dy_o);
  // The pose triple propagates through the SAME pose_compose the simulator used on
  // ground truth; landmark slots are STATIC — their mean components carry over by
  // exact copy (the identity block of F applied to the mean, nothing more).
  const Pose p_new = slam::core::pose_compose(Pose{px, py, th}, Twist{dx_o, dy_o, dtheta_o});
  std::vector<double> new_x{p_new.x, p_new.y, p_new.theta};
  for (size_t i = 3; i < x_.size(); ++i) new_x.push_back(x_[i]);
  x_ = new_x;
  const size_t n = x_.size();
  Mat f(n, std::vector<double>(n, 0.0));
  f[0][0] = 1.0;
  f[0][2] = j02;
  f[1][1] = 1.0;
  f[1][2] = j12;
  f[2][2] = 1.0;
  for (size_t i = 3; i < n; ++i) f[i][i] = 1.0;  // static landmark — identity rows/cols
  const Mat a = matmul(f, p_);  // A = FΣ (ascending k; multiplied zero terms change nothing)
  Mat p_next(n, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; ++i) {  // Σ̄ = A·Fᵀ — upper ascending, lower copied (exact copy)
    const std::vector<double>& row_a = a[i];
    for (size_t j = i; j < n; ++j) {
      double acc = 0.0;
      const std::vector<double>& row_f = f[j];
      for (size_t k = 0; k < n; ++k) acc += row_a[k] * row_f[k];
      p_next[i][j] = acc;
      p_next[j][i] = acc;
    }
  }
  p_next[0][0] += q_pos_;  // Q lands on the pose diagonal in slot order
  p_next[1][1] += q_pos_;
  p_next[2][2] += q_theta_;
  p_ = p_next;
}

// One observation, ascending id. KNOWN slot → standard EKF update (Joseph form);
// unknown id → augmentation (improper-prior limit — see the header comment).
void EkfSlam::observe(int obs_id, double z_range, double z_bearing) {
  const double px = x_[0], py = x_[1], th = x_[2];
  int slot = -1;
  for (size_t k = 0; k < ids_.size(); ++k) {  // association is GIVEN: linear id lookup
    if (ids_[k] == obs_id) {
      slot = static_cast<int>(k);
      break;
    }
  }

  const size_t n0 = x_.size();
  if (slot < 0) {
    // ---- BIRTH. Mean: the observation back-projected through the CURRENT pose —
    // the exact inverse of the measurement model, so this observation's innovation
    // is exactly 0 and no mean correction could exist anyway (that is what birth
    // MEANS: an unknown landmark carries no information about the pose).
    const double zx = z_range * libm_cos(z_bearing);  // same-argument pair — libm
    const double zy = z_range * libm_sin(z_bearing);
    const Point w = slam::core::robot_to_world(Point{zx, zy}, Pose{px, py, th});
    const double lx = w.x, ly = w.y;
    const double dx = lx - px;
    const double dy = ly - py;
    const double r_hat = std::sqrt(dx * dx + dy * dy);
    const double beta = std::atan2(dy, dx);  // no pair fold for atan2 — direct bind
    const double cb = libm_cos(beta);        // same-argument pair — libm
    const double sb = libm_sin(beta);
    const size_t n = n0;
    // H at the current mean — rows 3.. are ZERO (this landmark is not in the state
    // yet), which is exactly why birth cannot move the pose.
    Mat h(n, std::vector<double>(2, 0.0));
    h[0][0] = -cb;
    h[1][0] = -sb;
    h[2][0] = 0.0;
    h[0][1] = sb / r_hat;
    h[1][1] = -cb / r_hat;
    h[2][1] = -1.0;
    const Mat g = matmul(p_, h);  // G = ΣH — the SAME fixed-order product as an update
    Mat w_mat{{0.0, 0.0}, {0.0, 0.0}};
    double w01 = 0.0;
    for (size_t i = 0; i < n; ++i) {
      w_mat[0][0] += h[i][0] * g[i][0];
      w01 += h[i][0] * g[i][1];
      w_mat[1][1] += h[i][1] * g[i][1];
    }
    w_mat[0][1] = w01;
    w_mat[1][0] = w01;  // symmetric by copy (exact)
    w_mat[0][0] += r_range_;
    w_mat[1][1] += r_bearing_;
    // B = ∂h/∂landmark (det = 1/r̂ ≠ 0): row 0 is the range gradient, row 1 the
    // bearing gradient — the SAME entries as the landmark columns of H above.
    const Mat b_inv = invert2(Mat{{cb, sb}, {-sb / r_hat, cb / r_hat}});
    // New diagonal block B⁻¹WB⁻ᵀ (generic products; upper computed, lower copied);
    // cross columns −GB⁻ᵀ — exactly ±0 at t = 0, where Σ_pp is still zero.
    const Mat t_mat = matmul(b_inv, w_mat);
    Mat nb = matmul_at_b(t_mat, b_inv);
    nb[1][0] = nb[0][1];  // B⁻¹WB⁻ᵀ is symmetric — lower entry copied (exact)
    const Mat cross = matmul_at_b(g, b_inv);
    // Grow the state: append the mean pair, then grow Σ by one column pair and its
    // mirrored row (the copy keeps the matrix EXACTLY symmetric).
    x_.push_back(lx);
    x_.push_back(ly);
    Mat grown;
    for (size_t i = 0; i < n; ++i) {
      std::vector<double> row = p_[i];
      row.push_back(-cross[i][0]);
      row.push_back(-cross[i][1]);
      grown.push_back(row);
    }
    std::vector<double> col_x, col_y;
    for (size_t i = 0; i < n; ++i) {
      col_x.push_back(grown[i][n]);  // the just-appended cross column (slot n)
      col_y.push_back(grown[i][n + 1]);
    }
    std::vector<double> row_x = col_x;
    row_x.push_back(nb[0][0]);
    row_x.push_back(nb[0][1]);
    grown.push_back(row_x);
    std::vector<double> row_y = col_y;
    row_y.push_back(nb[1][0]);
    row_y.push_back(nb[1][1]);
    grown.push_back(row_y);
    p_ = grown;
    ids_.push_back(obs_id);
    return;
  }

  // ---- RE-OBSERVATION: standard EKF update on the existing slot.
  const size_t c1 = 3 + 2 * static_cast<size_t>(slot);
  const double lx = x_[c1];
  const double ly = x_[c1 + 1];
  const double dx = lx - px;
  const double dy = ly - py;
  const double r_hat = std::sqrt(dx * dx + dy * dy);
  const double beta = std::atan2(dy, dx);
  const double cb = libm_cos(beta);  // same-argument pair — libm
  const double sb = libm_sin(beta);
  const double y_r = z_range - r_hat;                    // range never wraps
  const double y_b = slam::core::wrap(z_bearing - slam::core::wrap(beta - th));
  const size_t n = n0;  // bearing wraps like the sim's does
  Mat h(n, std::vector<double>(2, 0.0));
  h[0][0] = -cb;
  h[1][0] = -sb;
  h[2][0] = 0.0;
  h[0][1] = sb / r_hat;
  h[1][1] = -cb / r_hat;
  h[2][1] = -1.0;
  h[c1][0] = cb;
  h[c1][1] = -sb / r_hat;
  h[c1 + 1][0] = sb;
  h[c1 + 1][1] = cb / r_hat;
  const Mat g = matmul(p_, h);  // G = ΣH (n×2), ascending order
  Mat s_mat{{0.0, 0.0}, {0.0, 0.0}};
  double s01 = 0.0;
  for (size_t i = 0; i < n; ++i) {
    s_mat[0][0] += h[i][0] * g[i][0];
    s01 += h[i][0] * g[i][1];
    s_mat[1][1] += h[i][1] * g[i][1];
  }
  s_mat[0][1] = s01;
  s_mat[1][0] = s01;  // symmetric by copy (exact)
  s_mat[0][0] += r_range_;
  s_mat[1][1] += r_bearing_;
  const Mat s_inv = invert2(s_mat);
  Mat k_gain;
  for (size_t i = 0; i < n; ++i) {
    const std::vector<double>& gi = g[i];
    k_gain.push_back({gi[0] * s_inv[0][0] + gi[1] * s_inv[1][0],
                      gi[0] * s_inv[0][1] + gi[1] * s_inv[1][1]});
  }
  // Mean update — componentwise in index order; θ re-wraps after its own add.
  x_[0] += k_gain[0][0] * y_r + k_gain[0][1] * y_b;
  x_[1] += k_gain[1][0] * y_r + k_gain[1][1] * y_b;
  x_[2] = slam::core::wrap(x_[2] + (k_gain[2][0] * y_r + k_gain[2][1] * y_b));
  for (size_t i = 3; i < n; ++i) {
    x_[i] += k_gain[i][0] * y_r + k_gain[i][1] * y_b;
  }
  // Joseph form: Σ ← AΣAᵀ + KRKᵀ with A = I − KH (fixed ascending matmuls; upper
  // triangle computed, lower copied).
  Mat a_mat;
  for (size_t i = 0; i < n; ++i) {
    std::vector<double> row;
    for (size_t j = 0; j < n; ++j) {
      double acc = 0.0;
      for (size_t k = 0; k < 2; ++k) acc += k_gain[i][k] * h[j][k];
      row.push_back((i == j ? 1.0 : 0.0) - acc);
    }
    a_mat.push_back(row);
  }
  const Mat m_mat = matmul(a_mat, p_);  // M = AΣ (ascending k)
  Mat sigma_new(n, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; ++i) {
    const std::vector<double>& row_m = m_mat[i];
    const double t_i0 = k_gain[i][0] * r_range_;  // (KR)[i][0], R diagonal — exact products
    const double t_i1 = k_gain[i][1] * r_bearing_;
    for (size_t j = i; j < n; ++j) {
      double acc = 0.0;
      const std::vector<double>& row_j = a_mat[j];
      for (size_t k = 0; k < n; ++k) acc += row_m[k] * row_j[k];
      acc += t_i0 * k_gain[j][0] + t_i1 * k_gain[j][1];
      sigma_new[i][j] = acc;
      sigma_new[j][i] = acc;
    }
  }
  p_ = sigma_new;
}

void EkfSlam::update(const Step& step, TraceRecorder* recorder) {
  if (poses_.empty()) {
    // t = 0 adopts the declared gauge pose with an EXACTLY zero covariance —
    // the gauge convention of this whole repo: the start pose IS known.
    x_ = {x0_, y0_, theta_deg_ * (kPi / 180.0)};  // math.radians, bit-identical
    p_ = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  } else {
    if (!step.has_odom) {  // contract: t ≥ 1 always carries the command
      throw std::runtime_error("ekf_slam: step " + std::to_string(step.t) +
                               " arrived without an odom command");
    }
    predict(step.odom.dx, step.odom.dy, step.odom.dtheta);
  }
  if (step.has_obs) {
    for (const auto& obs : step.obs) {  // ascending id — fixed observation order
      observe(obs.id, obs.range, obs.bearing);
    }
  }
  const Pose pose{x_[0], x_[1], x_[2]};
  poses_.push_back(pose);
  if (recorder != nullptr) {
    std::array<double, 3> cov{std::sqrt(p_[0][0]), std::sqrt(p_[1][1]), std::sqrt(p_[2][2])};
    recorder->pose_estimated(step.t, pose, cov);
    std::vector<LandmarkEstimate> estimates;
    for (size_t k = 0; k < ids_.size(); ++k) {
      estimates.push_back(LandmarkEstimate{ids_[k],
                                           x_[3 + 2 * k],
                                           x_[4 + 2 * k],
                                           true,
                                           std::sqrt(p_[3 + 2 * k][3 + 2 * k]),
                                           std::sqrt(p_[4 + 2 * k][4 + 2 * k])});
    }
    recorder->landmarks_updated(step.t, estimates);
  }
}

EstimateResult EkfSlam::finalize(slam::core::TraceRecorder* /*recorder*/) {
  EstimateResult result;
  result.poses = poses_;
  result.has_landmarks = true;
  for (size_t k = 0; k < ids_.size(); ++k) {
    result.landmarks.push_back(LandmarkEstimate{ids_[k],
                                                x_[3 + 2 * k],
                                                x_[4 + 2 * k],
                                                true,
                                                std::sqrt(p_[3 + 2 * k][3 + 2 * k]),
                                                std::sqrt(p_[4 + 2 * k][4 + 2 * k])});
  }
  return result;
}

}  // namespace slam::filter_based
