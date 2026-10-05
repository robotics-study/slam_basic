"""filter_based — the map lives INSIDE the estimator.

Landmarks joined to the state: EKF-SLAM (Smith & Cheeseman 1986) → Rao-Blackwellised
particle filters with exact landmark Kalman filters (FastSLAM 1.0/2.0, Montemerlo et
al.) → adaptive resampling + per-particle log-odds grids + incremental smoothing
(GMapping, Grisetti et al.). Association is GIVEN (id-matched observations), exactly
as in the source papers."""
