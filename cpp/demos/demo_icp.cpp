// icp demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_icp --scenario maps/scenarios/room01_straight.yaml \
//       --params configs/registration/icp.yaml --trace out/trace.jsonl

#include "slam/registration/icp.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::registration::Icp>(argc, argv); }
