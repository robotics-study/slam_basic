// splitmix64 — 언어 공용 비트 동일 PRNG(python/slam/core/rng.py의 정확한 미러).
// BigInt uint64 산술은 Python int(mod 2**64)와 C++ uint64_t 와 비트 단위로 같고,
// float64 연산(sqrt/log/cos)은 JS 고유의 ulp 을 낼 수 있어도 세 엔진이 같은 식을 계산한다.
// Box-Muller는 cos 분기만 쓰고 예비 값을 캐시하지 않는다 — 가우시안 하나당 드로 수가 고정이다.

// 드로마다 BigInt.asUintN(64, …) 이 마스크 역할을 한다 — 별도 상수 없음.
const GOLDEN = 0x9E3779B97F4A7C15n

export class Rng {
    private x: bigint

    constructor(seed: number | bigint) {
        this.x = BigInt.asUintN(64, BigInt(seed))
    }

    // splitmix64 드로 하나 — uint64(raw 정수).
    nextU64(): bigint {
        const x = BigInt.asUintN(64, this.x + GOLDEN)
        this.x = x
        let z = x
        z = BigInt.asUintN(64, (z ^ (z >> 30n)) * 0xBF58476D1CE4E5B9n)
        z = BigInt.asUintN(64, (z ^ (z >> 27n)) * 0x94D049BB133111EBn)
        return z ^ (z >> 31n)
    }

    // [0, 1) — 드로 하나의 상위 53비트를 2^-53으로 스케일.
    uniform01(): number {
        return Number(this.nextU64() >> 11n) * 2 ** -53
    }

    // Box-Muller(cos 분기만), 균일 드로 정확히 두 번.
    gaussian(mu: number = 0.0, sigma: number = 1.0): number {
        const u1 = this.uniform01()
        const u2 = this.uniform01()
        return mu + sigma * Math.sqrt(-2.0 * Math.log(1.0 - u1)) * Math.cos(2.0 * Math.PI * u2)
    }
}
