"""features — where landmarks come from (the genealogy's extraction branch).

The landmark-based branches (ekf_slam, fastslam) consume ``(id, bearing, range)``
observations with association GIVEN; this branch shows where those stable landmark
candidates come from in the first place: density clustering of raw scan points
(DBSCAN), expanded deterministically by point index order so the cluster ids stay
bit-identical across languages."""
