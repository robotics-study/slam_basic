#pragma once

#include <string>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/types.hpp"

// ekf_slam — Extended Kalman Filter SLAM (Smith & Cheeseman, ICRA 1986), the
// filter_based branch's first member: the map lives INSIDE the estimator. Every
// earlier branch CONSUMED (id, bearing, range) observations with association given;
// this is the page that shows what an observation DOES once association is given — it
// augments the state. The state vector is x = [px, py, θ | l_0x, l_0y, l_1x, ...]:
// the pose plus one (x, y) pair per landmark discovered so far, and ONE dense
// covariance Σ over all of it grows as landmarks are born. The C++ mirror of
// python/slam/filter_based/ekf_slam.py, operation for operation.
//
// THE GAUGE, HONESTLY. The declared start pose IS known exactly (the scenario's first
// GT pose IS the gauge — this repo's convention since histogram_filter), so Σ starts
// as the 3×3 ZERO matrix: the pose uncertainty is honestly zero at t = 0 and grows
// ONLY from Q, the odometry noise the filter declares. A landmark's FIRST observation
// therefore lands with covariance B⁻¹RB⁻ᵀ alone (pure measurement noise rotated into
// world frame — no pose contribution yet) and cross-covariance exactly ±0; every
// later re-observation of a KNOWN id then pulls the pose back through those
// correlation blocks. That is the whole lesson of the branch: landmarks do not just
// get estimated, they ESTIMATE the pose back (what graph_based will later restate as
// factor graphs).
//
// THE FIXED OPERATION ORDER (bit-identical with Python; every float op below rounds
// exactly where the Python line rounds — -ffp-contract=off everywhere):
//
//   t = 0            x ← gauge (x0, y0, radians(theta_deg)); Σ ← 3×3 zeros.
//   t ≥ 1 predict    x̂ ← x ⊕ u (the SAME pose_compose the simulator used to
//                    differentiate ground truth); Σ̄ = FΣFᵀ + Q embedded, F =
//                    blockdiag(J, I) with J evaluated at the PRIOR mean; Q lands on
//                    the pose diagonal in slot order. Landmark rows/cols of F are
//                    identity — a static landmark needs no motion model.
//   per observation  (ascending id, exactly as the simulator emitted them):
//     KNOWN id → standard EKF update, Joseph form Σ ← AΣAᵀ + KRKᵀ with A = I − KH
//                (the shortcut (I−KH)Σ is only ALGEBRAICALLY equal — not bit-equal).
//     NEW id   → augmentation, NOT an update: the mean becomes the observation
//                back-projected through the CURRENT pose (exact inverse of the
//                measurement model, so the innovation is exactly 0 by construction);
//                the covariance grows by B⁻¹WB⁻ᵀ with cross columns −GB⁻ᵀ — the
//                improper-prior limit (an unknown landmark carries no information
//                about the pose, so the pose block of Σ does not move).
//
// Σ stays EXACTLY symmetric by contract: every product computes its upper triangle in
// fixed ascending order and COPIES it below the diagonal (a copy is exact; two
// independent ulp-different ways of computing the same entry would not be). Landmarks
// are appended in OBSERVATION order, so state slots are NOT id-ordered when ids
// arrive out of order — the id↔slot bookkeeping is part of the page.

namespace slam::filter_based {

class EkfSlam final : public slam::core::Estimator {
 public:
  explicit EkfSlam(slam::core::ParamSet params);

  std::string name() const override { return "ekf_slam"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::LANDMARKS};
  }

  // t = 0 adopts the declared gauge pose with an EXACTLY zero covariance; every later
  // step predicts (pose_compose + FΣFᵀ + Q) then consumes this step's observations in
  // ascending id order. Events: pose_estimated (with cov = Σ diagonal square roots)
  // first, landmarks_updated (slot order, sx/sy present) second.
  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;

  // Poses = the whole estimated-pose history; landmarks = the state's landmark slots
  // in SLOT order (birth order), each with its per-axis standard deviations.
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 private:
  // predict: x̂ ← x ⊕ u, Σ̄ = FΣFᵀ + Q (fixed ascending-order products, upper triangle
  // computed and copied below the diagonal).
  void predict(double dx_o, double dy_o, double dtheta_o);
  // one observation, ascending id: known slot → Joseph update; unknown → augmentation.
  void observe(int obs_id, double z_range, double z_bearing);

  double x0_, y0_, theta_deg_;
  double q_pos_, q_theta_;    // Q diagonal (σ² — declared, not injected)
  double r_range_, r_bearing_;  // R diagonal (σ² in MEASUREMENT space)
  std::vector<double> x_;              // [px, py, θ | landmark pairs] in slot order
  std::vector<std::vector<double>> p_;  // dense covariance, same slot layout
  std::vector<int> ids_;               // slot k → scenario landmark id
  std::vector<slam::core::Pose> poses_;
};

}  // namespace slam::filter_based
