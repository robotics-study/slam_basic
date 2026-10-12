// ndt demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_ndt --scenario maps/scenarios/room02_diagonal.yaml \
//       --params configs/registration/ndt.yaml --trace out/trace.jsonl

#include "slam/registration/ndt.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::registration::Ndt>(argc, argv); }
