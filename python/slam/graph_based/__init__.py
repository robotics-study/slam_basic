"""graph_based — measurements become constraints, not state.

Pose-graph SLAM: SPA (Konolige 2001, pose-only graph with landmark co-visibility
constraints) and full information-form GraphSLAM (Grisetti et al.). Observations
become relative-pose constraints between trajectory nodes; the batch solve happens
in finalize(), and loop closures are constraints to NON-adjacent nodes."""
