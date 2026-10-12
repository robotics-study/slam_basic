// ekf_slam demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_ekf_slam --scenario maps/scenarios/office01_landmarks.yaml \
//       --params configs/filter_based/ekf_slam.yaml --trace out/trace.jsonl

#include "slam/filter_based/ekf_slam.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::filter_based::EkfSlam>(argc, argv); }
