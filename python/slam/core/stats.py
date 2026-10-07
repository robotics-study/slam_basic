"""Core statistics — the fixed-formula helpers every estimator shares (mirrors C++
``core/stats.hpp`` and the web engine's ``libs/stats.ts``, operation for operation).

One function lives here: the inverse standard-normal CDF (the probit) by Acklam's
rational approximation — the piece of mathematics KLD-sampling needs to turn a
probability into a chi-square quantile. The coefficients are Acklam's published
constants verbatim; the branch structure and operation order ARE the contract, so
all three languages round identically (log/sqrt bind to the same libSystem scalar
calls / hardware instructions as CPython's math module).

Accuracy: |relative error| < 1.15e-9 by construction — measured here against table
values: inv_norm_cdf(0.975) = 1.959963986120195 vs the true 1.9599639845400536
(diff 1.58e-9), inv_norm_cdf(0.99) = 2.326347874388028 vs 2.3263478740408408
(diff 3.47e-10). The central branch is a rational function of q = p − 0.5, so for
p in [0.5, 0.75] both subtractions are exact (Sterbenz) and the symmetry
inv(1−p) == −inv(p) holds BIT-identically — tests pin it. The tails switch to a
rational function of sqrt(−2 log p) instead; there the symmetry is only ulp-deep.
"""

from __future__ import annotations

import math

# Acklam's coefficients, verbatim (a1..a6 / b1..b5 central, c1..c6 / d1..d4 tails).
_A1 = -3.969683028665376e+01
_A2 = 2.209460984245205e+02
_A3 = -2.759285104469687e+02
_A4 = 1.383577518672690e+02
_A5 = -3.066479806614716e+01
_A6 = 2.506628277459239e+00
_B1 = -5.447609879822406e+01
_B2 = 1.615858368580409e+02
_B3 = -1.556989798598866e+02
_B4 = 6.680131188771972e+01
_B5 = -1.328068155288572e+01
_C1 = -7.784894002430293e-03
_C2 = -3.223964580411365e-01
_C3 = -2.400758277161838e+00
_C4 = -2.549732539343734e+00
_C5 = 4.374664141464968e+00
_C6 = 2.938163982698783e+00
_D1 = 7.784695709041462e-03
_D2 = 3.224671290700398e-01
_D3 = 2.445134137142996e+00
_D4 = 3.754408661907416e+00

# Break-point between the tail rational and the central one (Acklam's constant).
_P_LOW = 0.02425


def inv_norm_cdf(p: float) -> float:
    """Inverse standard-normal CDF by Acklam's rational approximation.

    Fixed branch structure, fixed operation order — this exact expression IS the
    contract (the KLD bound consumes the result; rounding drift would shift a
    sample count). Lower tail p < P_LOW and upper tail p > 1 − P_LOW use the
    sqrt(−2 log p) rational; the central region uses the q = p − 0.5 rational. At
    p = 0.5 exactly: q = 0, so the result is exactly +0.0."""
    if p < _P_LOW:
        q = math.sqrt(-2.0 * math.log(p))
        return (((((_C1 * q + _C2) * q + _C3) * q + _C4) * q + _C5) * q + _C6) / \
               ((((_D1 * q + _D2) * q + _D3) * q + _D4) * q + 1.0)
    if p > 1.0 - _P_LOW:
        q = math.sqrt(-2.0 * math.log(1.0 - p))
        return -((((( _C1 * q + _C2) * q + _C3) * q + _C4) * q + _C5) * q + _C6) / \
               ((((_D1 * q + _D2) * q + _D3) * q + _D4) * q + 1.0)
    q = p - 0.5
    r = q * q
    return (((((_A1 * r + _A2) * r + _A3) * r + _A4) * r + _A5) * r + _A6) * q / \
           (((((_B1 * r + _B2) * r + _B3) * r + _B4) * r + _B5) * r + 1.0)
