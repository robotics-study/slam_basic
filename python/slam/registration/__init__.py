"""registration — pose from measurements alone (odometry without wheels).

Scan-to-scan / scan-to-map registration: point-to-point ICP (Besl & McKay 1992)
and Normal Distributions Transform (Biber & Strasser 2003). These estimators never
read Step.odom — the whole point is estimating motion from scans alone."""
