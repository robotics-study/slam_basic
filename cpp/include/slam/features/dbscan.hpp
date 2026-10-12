#pragma once

#include <set>
#include <string>
#include <vector>

#include "slam/core/estimator.hpp"
#include "slam/core/types.hpp"

// dbscan — Density-Based Spatial Clustering of Applications with Noise (Ester,
// Kriegel, Sander & Xu, KDD 1996), the features branch's first member and the answer
// to where this repo's landmark-based branches (filter_based: ekf_slam, fastslam) get
// their (id, bearing, range) observations from. They CONSUME association as given;
// this is the page that shows an id being BORN: raw scan points density-clustered
// into stable candidates, each candidate's id its discovery order, its position the
// mean of its members. The C++ mirror of python/slam/features/dbscan.py, operation
// for operation.
//
// THE ALGORITHM (Ester's Algorithm 1, fixed operation order — every comparison below
// is a squared-distance compare dx² + dy² ≤ eps², no sqrt anywhere): points are
// visited in ASCENDING INDEX (the accumulation order: step-major, beam-minor). An
// unvisited point is core iff its closed eps-neighborhood holds at least min_pts
// points INCLUDING ITSELF; a non-core visit labels the point NOISE and noise is
// sticky — a point visited while sparse stays noise forever even when a later cluster
// grows within eps of it (on static data, re-querying a noise point returns the same
// sub-threshold neighborhood, so this equals the paper's pseudocode exactly). A core
// seed opens a new cluster with the next id and expands: a growing list walked by
// index pointer; every unvisited point reached joins the cluster, but only a reached
// point whose OWN neighborhood also reaches min_pts (a core point) extends the
// frontier — border points join without expanding. Cluster id = seed order = ascending
// index of each cluster's first core point, which is why ids stay identical across
// languages AND stable across steps.
//
// WHAT THIS BRANCH MEASURES — and what it does not: dbscan measures NO motion. The
// pose that lifts robot-frame scan points into world frame is the declared gauge pose
// composed with the scenario's odometry (σ_xy = σ_θ = 0 in this scenario, so the
// integrated pose equals ground truth bit-for-bit) — exactly grid_mapping's "pose is
// given" convention. ATE/RPE read ~0 by construction; the only real metric here is
// landmark_rmse over ids present on both sides.

namespace slam::features {

// Label sentinels — the paper's UNVISITED / NOISE labels as ints (cluster ids are k ≥ 0).
inline constexpr int kUnvisited = -2;
inline constexpr int kNoise = -1;

// Ester Algorithm 1 over `points` in index order (see the module comment for the full
// fixed-order contract). Returns clusters in DISCOVERY order; each cluster's members
// sit in append order (seed first). Points labeled NOISE appear nowhere. The seed is
// labeled BEFORE ExpandCluster runs, and a core q re-appends itself to the growing
// list (it is still UNVISITED at filter time) — the duplicate entry it meets again is
// skipped; both details are part of the bit-for-bit contract with Python.
std::vector<std::vector<slam::core::Point>> dbscan_cluster(
    const std::vector<slam::core::Point>& points, double eps, int min_pts);

// Density clustering of accumulated scan points → landmark candidates (the features
// branch: where the landmark-based branches' ids come from). The pose is the declared
// gauge pose composed with every arriving odom command (this branch CONSUMES odometry
// — the exact opposite of the registration branch); each step's scan points are lifted
// to world frame in beam order, appended to the accumulation, and ALL accumulated
// points re-clustered. The centroid is a mean over members in append order — plain
// += loops, matching the Python side (which deliberately avoids CPython 3.12's
// Neumaier-compensated builtin sum). sx/sy stay unset: this estimator carries no
// uncertainty model; that is what filter_based adds on top of this.
class Dbscan final : public slam::core::Estimator {
 public:
  explicit Dbscan(slam::core::ParamSet params);

  std::string name() const override { return "dbscan"; }
  std::set<slam::core::Capability> required_capabilities() const override {
    return {slam::core::Capability::BEAM};
  }

  // t = 0 adopts the declared gauge pose (x0, y0, radians(theta_deg)); every later
  // step composes the arriving odom command onto the running pose. A step without a
  // scan accumulates nothing and re-emits (re-clustering an unchanged cloud is a
  // fixed point). Events: pose_estimated first, landmarks_updated second.
  void update(const slam::core::Step& step, slam::core::TraceRecorder* recorder) override;

  // Poses = the whole running-pose history; landmarks = the LAST step's clusters
  // (id = discovery order, position = member mean). has_landmarks is always set —
  // an empty list simply omits landmark_rmse in metrics, exactly like Python's None.
  slam::core::EstimateResult finalize(slam::core::TraceRecorder* recorder) override;

 private:
  double x0_, y0_, theta_deg_, eps_;
  int min_pts_;
  slam::core::Pose pose_{};                    // gauge composed with odom so far
  std::vector<slam::core::Point> points_;      // accumulated world points — step-major, beam-minor
  std::vector<slam::core::Pose> poses_;        // readout history (index == step t)
  std::vector<slam::core::LandmarkEstimate> landmarks_;  // last re-clustering's output
};

}  // namespace slam::features
