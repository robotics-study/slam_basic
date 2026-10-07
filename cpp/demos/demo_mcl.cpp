// mcl demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_mcl --scenario maps/scenarios/corridor03_drift.yaml \
//       --params configs/filtering/mcl.yaml --trace out/trace.jsonl

#include "slam/filtering/mcl.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::filtering::Mcl>(argc, argv); }
