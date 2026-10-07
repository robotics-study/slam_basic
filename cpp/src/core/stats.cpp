#include "slam/core/stats.hpp"

#include <cmath>

// Bit-identical mirror of the Python module: std::log/std::sqrt bind to the same
// libSystem scalar _log / hardware sqrtsd that CPython's math.log/math.sqrt call —
// no dlsym routing needed (the pair-fold hazard is sin/cos only, see core/libm.hpp).

namespace slam::core {

double inv_norm_cdf(double p) {
  if (p < kPLow) {
    const double q = std::sqrt(-2.0 * std::log(p));
    return (((((kC1 * q + kC2) * q + kC3) * q + kC4) * q + kC5) * q + kC6) /
           ((((kD1 * q + kD2) * q + kD3) * q + kD4) * q + 1.0);
  }
  if (p > 1.0 - kPLow) {
    const double q = std::sqrt(-2.0 * std::log(1.0 - p));
    // Python's `-num / den` is (-num)/den — sign-symmetric division makes this exact.
    return -(((((kC1 * q + kC2) * q + kC3) * q + kC4) * q + kC5) * q + kC6) /
           ((((kD1 * q + kD2) * q + kD3) * q + kD4) * q + 1.0);
  }
  const double q = p - 0.5;
  const double r = q * q;
  return (((((kA1 * r + kA2) * r + kA3) * r + kA4) * r + kA5) * r + kA6) * q /
         (((((kB1 * r + kB2) * r + kB3) * r + kB4) * r + kB5) * r + 1.0);
}

}  // namespace slam::core
