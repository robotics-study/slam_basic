// The splitmix64 contract: golden values pin the bit-identical cross-language
// stream. These constants ARE the contract — identical to the Python goldens; if a
// golden changes, every language mirror breaks on purpose.

#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

#include "slam/core/libm.hpp"
#include "slam/core/rng.hpp"

using slam::core::Rng;

TEST(Rng, ScalarLibmGoldenAtDivergentInput) {
  // Apple clang folds same-argument (sin, cos) call pairs into __sincos_stret — a
  // SIMD variant that differs from the scalar libSystem sin/cos (what Python's
  // math.sin/math.cos call) by 1 ulp on rare inputs. libm_sin/libm_cos route
  // through dlsym-resolved pointers so no fold can happen; this input is a KNOWN
  // divergence point — code calling std::sin/std::cos as a pair fails these goldens
  // (the Python test_rng pins the same values for math.sin/math.cos).
  double x = std::strtod("0x1.f9cbc4269ab30p-2", nullptr);
  EXPECT_DOUBLE_EQ(slam::core::libm_sin(x), std::strtod("0x1.e57a6c8be62efp-2", nullptr));
  EXPECT_DOUBLE_EQ(slam::core::libm_cos(x), std::strtod("0x1.c2cd1ba67a4fdp-1", nullptr));
}

TEST(Rng, GoldenU64) {
  // The reference splitmix64 stream from seed 0 (Steele et al. variant).
  Rng r(0);
  EXPECT_EQ(r.next_u64(), 0xE220A8397B1DCDAFULL);
  EXPECT_EQ(r.next_u64(), 0x6E789E6AA1B965F4ULL);

  Rng r42(42);
  EXPECT_EQ(r42.next_u64(), 0xBDD732262FEB6E95ULL);
  EXPECT_EQ(r42.next_u64(), 0x28EFE333B266F103ULL);
}

TEST(Rng, Uniform01GoldenAndRange) {
  Rng r(42);
  EXPECT_DOUBLE_EQ(r.uniform01(), 0.7415648787718233);
  EXPECT_DOUBLE_EQ(r.uniform01(), 0.1599103928769201);
  // Range: exactly 53 bits, [0, 1).
  Rng r2(7);
  for (int i = 0; i < 1000; ++i) {
    double u = r2.uniform01();
    EXPECT_GE(u, 0.0);
    EXPECT_LT(u, 1.0);
  }
}

TEST(Rng, GaussianGolden) {
  // cos-branch Box-Muller, two draws per gaussian — golden pairs pin the order.
  EXPECT_DOUBLE_EQ(Rng(42).gaussian(), 0.8822489062222688);
  EXPECT_DOUBLE_EQ(Rng(0).gaussian(), -1.8839083333524402);
}

TEST(Rng, GaussianAffineAndDeterminism) {
  // gaussian(mu, sigma) = mu + sigma * standard(golden pair) — same draws, affine.
  Rng r(42);
  EXPECT_NEAR(r.gaussian(1.0, 2.0), 1.0 + 2.0 * 0.8822489062222688, 1e-12);
  // Same seed -> identical stream.
  Rng a(5), b(5);
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(a.gaussian(), b.gaussian());
}

TEST(Rng, GaussianStatisticsSanity) {
  Rng r(1234);
  double sum = 0.0, sum_sq = 0.0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    double x = r.gaussian();
    sum += x;
    sum_sq += x * x;
  }
  double mean = sum / n;
  double var = sum_sq / n - mean * mean;
  EXPECT_LT(std::abs(mean), 0.05);
  EXPECT_LT(std::abs(var - 1.0), 0.1);
}
