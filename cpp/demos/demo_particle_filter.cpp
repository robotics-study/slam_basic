// particle_filter demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_particle_filter --scenario maps/scenarios/corridor03_drift.yaml \
//       --params configs/filtering/particle_filter.yaml --trace out/trace.jsonl

#include "slam/filtering/particle_filter.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::filtering::ParticleFilter>(argc, argv); }
