#pragma once

#include <cmath>
#include <cstdint>

#include "slam/core/libm.hpp"
#include "slam/core/types.hpp"

namespace slam::core {

// splitmix64 — the bit-identical cross-language PRNG (spec/data_formats.md). Every
// random number in this repository comes from here. The state starts at the seed and
// each draw advances it with splitmix64; integers are uint64_t (mod 2^64, overflow
// included) so Python ints and C++ compute identical bits, floats are float64/double.
//
//     next():  x += 0x9E3779B97F4A7C15            # mod 2^64
//              z = x
//              z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
//              z = (z ^ (z >> 27)) * 0x94D049BB133111EB
//              return z ^ (z >> 31)
//     uniform01(): float(next() >> 11) * 2**-53     # exactly 53 bits, [0, 1)
//     gaussian(mu, sigma): u1 = uniform01(); u2 = uniform01()   # TWO draws, fixed
//                          mu + sigma * sqrt(-2 ln(1 - u1)) * cos(2 pi u2)
//
// Box-Muller keeps only the `cos` branch (the sin branch is discarded): two draws
// per gaussian, no cached spare — the draw COUNT per gaussian is fixed.
class Rng {
 public:
  explicit Rng(long long seed) : x_(static_cast<uint64_t>(seed)) {}

  uint64_t next_u64() {
    x_ = static_cast<uint64_t>(x_ + 0x9E3779B97F4A7C15ULL);
    uint64_t z = x_;
    z = static_cast<uint64_t>(z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = static_cast<uint64_t>(z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }

  // Uniform in [0, 1): the top 53 bits of one draw scaled by 2^-53.
  double uniform01() {
    return static_cast<double>(next_u64() >> 11) * std::ldexp(1.0, -53);
  }

  // Box-Muller (cos branch only), exactly two uniform draws (defaults mirror the
  // Python signature's mu=0.0 / sigma=1.0). cos routes through libm_cos (scalar
  // libSystem cos — what math.cos calls; see core/libm.hpp); sqrt/log are hardware
  // sqrtsd and the direct _log libcall.
  double gaussian(double mu = 0.0, double sigma = 1.0) {
    double u1 = uniform01();
    double u2 = uniform01();
    return mu + sigma * std::sqrt(-2.0 * std::log(1.0 - u1)) * libm_cos(2.0 * kPi * u2);
  }

 private:
  uint64_t x_;
};

}  // namespace slam::core
