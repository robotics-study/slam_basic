#include "slam/maps/pgm.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>

namespace slam::maps {
namespace {

// Reads the next whitespace-separated token, skipping '#' comment lines. PGM
// headers allow comments between any tokens; ascii pixel data flows across line
// breaks, so tokenizing the stream (never trusting line layout) is the contract.
std::string next_token(std::istream& in) {
  std::string tok;
  char c;
  while (in.get(c)) {
    if (c == '#') {
      while (in.get(c) && c != '\n') {
      }
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!tok.empty()) return tok;
      continue;
    }
    tok += c;
  }
  return tok;
}

int parse_int(const std::string& tok, const std::string& path) {
  try {
    return std::stoi(tok);
  } catch (const std::exception&) {
    throw std::runtime_error("pgm: invalid integer token in '" + path + "'");
  }
}

}  // namespace

PgmImage load_pgm(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("pgm: cannot open '" + path + "'");

  std::string magic = next_token(in);
  if (magic != "P2" && magic != "P5") {
    throw std::runtime_error("unsupported PGM magic '" + magic + "' (only P2/P5)");
  }

  int width = parse_int(next_token(in), path);
  int height = parse_int(next_token(in), path);
  int maxval = parse_int(next_token(in), path);
  if (width <= 0 || height <= 0 || maxval <= 0 || maxval > 65535) {
    throw std::runtime_error("pgm: invalid header in '" + path + "'");
  }

  const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);
  PgmImage img;
  img.width = width;
  img.height = height;
  img.pixels.resize(count);

  if (magic == "P2") {
    for (size_t i = 0; i < count; ++i) {
      std::string tok = next_token(in);
      if (tok.empty()) throw std::runtime_error("pgm: truncated ascii data in '" + path + "'");
      img.pixels[i] = static_cast<std::uint16_t>(parse_int(tok, path));
    }
  } else {
    // Exactly ONE whitespace char separates the header from the binary payload —
    // next_token consumed it with its trailing-whitespace skip, so the stream now
    // sits at the first pixel byte. maxval < 256 packs one byte per sample; larger
    // maxvals use two bytes per sample, big-endian (Netpbm spec).
    bool wide = maxval >= 256;
    for (size_t i = 0; i < count; ++i) {
      if (wide) {
        int hi = in.get();
        int lo = in.get();
        if (hi == EOF || lo == EOF) {
          throw std::runtime_error("pgm: truncated binary data in '" + path + "'");
        }
        img.pixels[i] = static_cast<std::uint16_t>((hi << 8) | lo);
      } else {
        int v = in.get();
        if (v == EOF) throw std::runtime_error("pgm: truncated binary data in '" + path + "'");
        img.pixels[i] = static_cast<std::uint16_t>(v);
      }
    }
  }
  return img;
}

}  // namespace slam::maps
