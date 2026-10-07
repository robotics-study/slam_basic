// grid_mapping demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_grid_mapping --scenario maps/scenarios/office01_tour.yaml \
//       --params configs/filtering/grid_mapping.yaml --trace out/trace.jsonl

#include "slam/filtering/grid_mapping.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::filtering::GridMapping>(argc, argv); }
