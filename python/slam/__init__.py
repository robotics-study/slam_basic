"""slam — single-robot SLAM / state estimation algorithms (Python mirror of C++).

Algorithm packages are named after the site's sections — the branches of the SLAM
genealogy: ``filtering`` (recursive Bayes filters over a KNOWN map: histogram filter,
grid mapping, particle filter, MCL), ``registration`` (scan-matching odometry without
wheels: ICP, NDT), ``filter_based`` (the map lives inside the estimator: EKF-SLAM,
FastSLAM 1.0/2.0, GMapping) and ``graph_based`` (measurements become constraints:
SPA, GraphSLAM). Every estimator consumes the same contract — noisy odometry plus one
sensor type (beam scans or landmark bearings/ranges) produced by the core simulator —
and emits its estimates as trace events; nothing here plans a path.
"""

__version__ = "0.1.0"
