"""The splitmix64 contract: golden values pin the bit-identical cross-language stream.

These constants ARE the contract — C++ must reproduce them exactly (the C++ test
suite asserts the same numbers). If a golden changes, every language mirror breaks
on purpose."""

import pytest

from slam.core.rng import Rng


def test_splitmix64_golden_u64() -> None:
    r = Rng(0)
    # The reference splitmix64 stream from seed 0 (Steele et al. variant).
    assert r.next_u64() == 0xE220A8397B1DCDAF
    assert r.next_u64() == 0x6E789E6AA1B965F4


def test_splitmix64_golden_seed_42() -> None:
    r = Rng(42)
    assert r.next_u64() == 0xBDD732262FEB6E95
    assert r.next_u64() == 0x28EFE333B266F103


def test_uniform01_golden_and_range() -> None:
    r = Rng(42)
    assert r.uniform01() == 0.7415648787718233
    assert r.uniform01() == 0.1599103928769201
    # Range: exactly 53 bits, [0, 1).
    r2 = Rng(7)
    for _ in range(1000):
        u = r2.uniform01()
        assert 0.0 <= u < 1.0


def test_gaussian_golden() -> None:
    # cos-branch Box-Muller, two draws per gaussian — golden pairs pin the order.
    assert Rng(42).gaussian() == 0.8822489062222688
    assert Rng(0).gaussian() == -1.8839083333524402


def test_gaussian_affine_and_determinism() -> None:
    # gaussian(mu, sigma) = mu + sigma * standard(golden pair) — same draws, affine.
    r = Rng(42)
    assert r.gaussian(1.0, 2.0) == pytest.approx(1.0 + 2.0 * 0.8822489062222688, rel=1e-12)
    # Same seed -> identical stream.
    a = [Rng(5).gaussian() for _ in range(3)]
    b = [Rng(5).gaussian() for _ in range(3)]
    assert a == b


def test_gaussian_statistics_sanity() -> None:
    r = Rng(1234)
    xs = [r.gaussian() for _ in range(20000)]
    mean = sum(xs) / len(xs)
    var = sum(x * x for x in xs) / len(xs) - mean * mean
    assert abs(mean) < 0.05
    assert abs(var - 1.0) < 0.1
