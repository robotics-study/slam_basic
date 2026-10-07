// core/stats tests — the inverse normal CDF's fixed-order contract (the C++ mirror
// of python/tests/test_stats.py). The branch structure IS the contract: lower tail
// (p < 0.02425) and upper tail (p > 1 − 0.02425) are rational functions of
// sqrt(−2 log p), the central region a rational function of q = p − 0.5. The
// approximation is accurate to |relative error| < 1.15e-9 by construction — pinned
// here against table values (Φ⁻¹(0.975) = 1.9599639845400536, Φ⁻¹(0.99) =
// 2.3263478740408408). Because the central branch is a rational function of q with
// both subtractions exact (Sterbenz for p in [0.5, 0.75]), inv(1 − p) == −inv(p)
// holds BIT-IDENTICALLY there — and since both languages call the same libSystem
// log/sqrt on identical doubles, every pinned value below is bit-identical across
// Python and C++.

#include <cmath>

#include <gtest/gtest.h>

#include "slam/core/stats.hpp"

using slam::core::inv_norm_cdf;

TEST(Stats, CentralBranchIsExactAtHalfAndSymmetric) {
  // p = 0.5 lands exactly on q = p − 0.5 = 0 → the rational returns exactly +0.0;
  // and for central samples the sign flip is bit-exact, not merely close.
  EXPECT_EQ(inv_norm_cdf(0.5), 0.0);
  for (double p : {0.5, 0.55, 0.6, 0.65, 0.7, 0.75}) {
    // Bit-identical symmetry: q flips sign exactly (both subtractions exact),
    // r = q·q is identical, so the rational evaluates to the exact negation.
    EXPECT_EQ(inv_norm_cdf(1.0 - p), -inv_norm_cdf(p));
  }
}

TEST(Stats, PinnedValuesMatchTheTablesWithinTheClaimedBound) {
  // The values the KLD bound actually consumes (δ = 0.01 → z_q = Φ⁻¹(0.99)) and
  // one central-branch pin, each against its table value; the approximation's
  // claimed |relative error| < 1.15e-9 shows up here as an absolute diff well
  // under 2e-9. The pins are EXACT doubles — bit-identical to Python's values.
  EXPECT_EQ(inv_norm_cdf(0.975), 1.959963986120195);  // true: 1.9599639845400536
  EXPECT_LT(std::abs(inv_norm_cdf(0.975) - 1.9599639845400536), 2e-9);
  EXPECT_EQ(inv_norm_cdf(0.99), 2.326347874388028);  // true: 2.3263478740408408
  EXPECT_LT(std::abs(inv_norm_cdf(0.99) - 2.3263478740408408), 1e-9);
}

TEST(Stats, TailsAndMonotonicity) {
  // Both tail branches (p < P_LOW and p > 1 − P_LOW) pin to exact doubles, and the
  // whole function is non-decreasing on a coarse grid across all three branches.
  EXPECT_EQ(inv_norm_cdf(0.001), -3.090232304709404);
  EXPECT_EQ(inv_norm_cdf(0.999), 3.090232304709404);
  double prev = inv_norm_cdf(1.0 / 100.0);
  for (int i = 2; i < 100; ++i) {
    const double y = inv_norm_cdf(static_cast<double>(i) / 100.0);
    EXPECT_LE(prev, y);
    prev = y;
  }
}
