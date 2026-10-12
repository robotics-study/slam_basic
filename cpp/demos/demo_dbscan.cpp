// dbscan demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_dbscan --scenario maps/scenarios/blobs01_line.yaml \
//       --params configs/features/dbscan.yaml --trace out/trace.jsonl

#include "slam/features/dbscan.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::features::Dbscan>(argc, argv); }
