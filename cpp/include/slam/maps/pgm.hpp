#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace slam::maps {

// Grayscale image loaded from a PGM file, row-major with row 0 = top. The repo's
// own maps are P2 (ascii) on purpose — git diffs of maps/ must read.
struct PgmImage {
  int width = 0;
  int height = 0;
  std::vector<std::uint16_t> pixels;  // size width*height, values in [0, maxval]
};

// Reads PGM P2 (ascii) and P5 (binary). Header tokens (magic, width, height,
// maxval) are whitespace separated and may carry '#' comments; ascii pixel data
// likewise flows across line breaks, so the whole stream is tokenized rather than
// trusting line layout. Throws std::runtime_error on malformed input.
PgmImage load_pgm(const std::string& path);

}  // namespace slam::maps
