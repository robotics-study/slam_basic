"""splitmix64 — the cross-language bit-identical PRNG (spec/data_formats.md).

Every random number in this repository comes from here. The state starts at the
seed and each draw advances it with splitmix64; integers are uint64 (mod 2**64,
overflow included) so Python ints and C++ uint64_t compute identical bits, floats
are float64/double throughout.

    next():  x += 0x9E3779B97F4A7C15            # mod 2^64
             z = x
             z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
             z = (z ^ (z >> 27)) * 0x94D049BB133111EB
             return z ^ (z >> 31)

    uniform01(): float(next() >> 11) * 2**-53     # exactly 53 bits, [0, 1)
    gaussian(mu, sigma): u1 = uniform01(); u2 = uniform01()   # TWO draws, fixed
                         mu + sigma * sqrt(-2 ln(1 - u1)) * cos(2 pi u2)

Box-Muller keeps only the `cos` branch (the sin branch is discarded): two draws per
gaussian, no cached spare — so the draw COUNT per gaussian is fixed and language-
independent. 1 - u1 never underflows because u1 < 1 always.
"""

from __future__ import annotations

import math

_MASK = 0xFFFFFFFFFFFFFFFF


class Rng:
    """splitmix64 stream seeded at `seed` (the scenario seed, or an algorithm's own
    declared seed parameter — see spec/data_formats.md draw-order contract)."""

    def __init__(self, seed: int) -> None:
        self._x = seed & _MASK

    def next_u64(self) -> int:
        """One splitmix64 draw as a uint64 value (the raw 64-bit integer)."""
        x = (self._x + 0x9E3779B97F4A7C15) & _MASK
        self._x = x
        z = x
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & _MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & _MASK
        return z ^ (z >> 31)

    def uniform01(self) -> float:
        """Uniform in [0, 1): the top 53 bits of one draw scaled by 2**-53."""
        return float(self.next_u64() >> 11) * 2.0**-53

    def gaussian(self, mu: float = 0.0, sigma: float = 1.0) -> float:
        """Box-Muller (cos branch only), exactly two uniform draws."""
        u1 = self.uniform01()
        u2 = self.uniform01()
        return mu + sigma * math.sqrt(-2.0 * math.log(1.0 - u1)) * math.cos(2.0 * math.pi * u2)
