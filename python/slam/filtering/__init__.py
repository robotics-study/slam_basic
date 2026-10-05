"""filtering — recursive Bayes filters over a KNOWN map (the genealogy's root).

The two halves of SLAM separated for teaching: belief over the ROBOT pose on a
given map (histogram filter → particle filter → MCL) and belief over the MAP given
a known pose (log-odds grid mapping). Everything here consumes Step.odom + one
sensor capability and emits per-step estimation events; none of it plans."""
