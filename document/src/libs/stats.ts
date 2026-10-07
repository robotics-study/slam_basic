// core/stats(stats.py / stats.hpp)의 라이브 미러 — 역표준정규 CDF(probit)를 Acklam의
// 유리식 근사로. KLD-sampling이 확률을 카이제곱 분위수로 바꾸는 수학적 조각이다.
// 계수는 Acklam의 공개 상수 그대로이고, 분기 구조와 연산 순서 자체가 계약이다 —
// JS의 Math.log/Math.sqrt는 libSystem과 ulp 단위로 다를 수 있어도(사이트 문서의 사실)
// 세 엔진이 같은 식을 계산한다. 정확도: 구성상 |상대 오차| < 1.15e-9. 중앙 분기는
// q = p − 0.5의 유리식이라 p ∈ [0.5, 0.75]에서 두 뺄셈이 모두 정확(Sterbenz)하고
// inv(1−p) == −inv(p)가 비트 단위로 성립한다 — 꼬리는 sqrt(−2 log p)의 유리식이라
// 그 대칭은 ulp 깊이에만 성립한다.

// Acklam 계수 그대로 (a1..a6 / b1..b5 중앙, c1..c6 / d1..d4 꼬리).
const A1 = -3.969683028665376e+01
const A2 = 2.209460984245205e+02
const A3 = -2.759285104469687e+02
const A4 = 1.383577518672690e+02
const A5 = -3.066479806614716e+01
const A6 = 2.506628277459239e+00
const B1 = -5.447609879822406e+01
const B2 = 1.615858368580409e+02
const B3 = -1.556989798598866e+02
const B4 = 6.680131188771972e+01
const B5 = -1.328068155288572e+01
const C1 = -7.784894002430293e-03
const C2 = -3.223964580411365e-01
const C3 = -2.400758277161838e+00
const C4 = -2.549732539343734e+00
const C5 = 4.374664141464968e+00
const C6 = 2.938163982698783e+00
const D1 = 7.784695709041462e-03
const D2 = 3.224671290700398e-01
const D3 = 2.445134137142996e+00
const D4 = 3.754408661907416e+00

// 꼬리 유리식과 중앙 유리식의 분기점 (Acklam의 상수).
const P_LOW = 0.02425

export function invNormCdf(p: number): number {
    if (p < P_LOW) {
        const q = Math.sqrt(-2.0 * Math.log(p))
        return (((((C1 * q + C2) * q + C3) * q + C4) * q + C5) * q + C6) /
            ((((D1 * q + D2) * q + D3) * q + D4) * q + 1.0)
    }
    if (p > 1.0 - P_LOW) {
        const q = Math.sqrt(-2.0 * Math.log(1.0 - p))
        // Python의 `-num / den`은 (-num)/den — 나눗셈은 부호 대칭이라 정확히 같다.
        return -(((((C1 * q + C2) * q + C3) * q + C4) * q + C5) * q + C6) /
            ((((D1 * q + D2) * q + D3) * q + D4) * q + 1.0)
    }
    const q = p - 0.5
    const r = q * q
    return (((((A1 * r + A2) * r + A3) * r + A4) * r + A5) * r + A6) * q /
        (((((B1 * r + B2) * r + B3) * r + B4) * r + B5) * r + 1.0)
}
