// histogram_filter demo — assembly only (see demos/demo_common.hpp).
//
//   ./demo_histogram_filter --scenario maps/scenarios/corridor01_back_and_forth.yaml \
//       --params configs/filtering/histogram_filter.yaml --trace out/trace.jsonl

#include "slam/filtering/histogram_filter.hpp"
#include "demo_common.hpp"

int main(int argc, char** argv) { return demo::run<slam::filtering::HistogramFilter>(argc, argv); }
