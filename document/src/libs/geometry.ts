// python/slam/core/geometry.py의 정확한 미러. 모든 함수는 고정 IEEE-754 double 산술식이고
// 연산 순서가 계약의 일부다 — Python/C++과 같은 판정(부동소수 비교 포함). JS 의 Math.* 는
// libSystem 과 다른 ulp 을 낼 수 있지만(사이트 문서에 명시된 사실), 시뮬레이터와 TS 엔진이
// 같은 JS 산술을 공유하는 한 브라우저 실행은 자체적으로 결정적이다.
import {Point, Pose, Twist} from "./trace/types";

// 각도를 [-pi, pi) 로 접는다 — 공식 자체가 계약이다 (a = pi 에서 -pi 를 반환).
export function wrap(a: number): number {
    return a - 2.0 * Math.PI * Math.floor((a + Math.PI) / (2.0 * Math.PI))
}

// p (+) t — 로봇 프레임 twist를 world 자세에 적용한다 (고정 연산 순서).
export function poseCompose(p: Pose, t: Twist): Pose {
    const c = Math.cos(p[2])
    const s = Math.sin(p[2])
    return [p[0] + c * t[0] - s * t[1], p[1] + s * t[0] + c * t[1], wrap(p[2] + t[2])]
}

// a⁻¹ (+) b — a를 b로 옮기는 로봇 프레임 twist (시뮬레이터가 GT를 미분하는 정확한 역합성).
export function poseMinus(a: Pose, b: Pose): Twist {
    const c = Math.cos(a[2])
    const s = Math.sin(a[2])
    const dx = b[0] - a[0]
    const dy = b[1] - a[1]
    return [c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])]
}

// world 점을 p의 로봇 프레임으로: z = R(-theta)(e - p).
export function worldToRobot(e: Point, p: Pose): Point {
    const ex = e[0] - p[0]
    const ey = e[1] - p[1]
    const c = Math.cos(p[2])
    const s = Math.sin(p[2])
    return [c * ex + s * ey, -s * ex + c * ey]
}

// 로봇 프레임 점을 world로: e = R(theta) z + (px, py) — robotToRobot의 정확한 역.
export function robotToWorld(z: Point, p: Pose): Point {
    const c = Math.cos(p[2])
    const s = Math.sin(p[2])
    return [p[0] + c * z[0] - s * z[1], p[1] + s * z[0] + c * z[1]]
}

// 부호가 a→b→c의 orientation인 외적 (b-a) x (c-a).
function orient(a: Point, b: Point, c: Point): number {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

// p는 a→b와 collinear(호출자가 orient == 0 확인): 세그먼트 위에 있는가?
function onSegment(a: Point, b: Point, p: Point): boolean {
    return Math.min(a[0], b[0]) <= p[0] && p[0] <= Math.max(a[0], b[0])
        && Math.min(a[1], b[1]) <= p[1] && p[1] <= Math.max(a[1], b[1])
}

// 닫힌 세그먼트 교차 판정(접촉 포함). collinear overlap은 on-segment 케이스로 떨어진다.
export function segmentsIntersect(a: Point, b: Point, c: Point, d: Point): boolean {
    const o1 = orient(a, b, c)
    const o2 = orient(a, b, d)
    const o3 = orient(c, d, a)
    const o4 = orient(c, d, b)
    if (o1 === 0.0 && onSegment(a, b, c)) return true
    if (o2 === 0.0 && onSegment(a, b, d)) return true
    if (o3 === 0.0 && onSegment(c, d, a)) return true
    if (o4 === 0.0 && onSegment(c, d, b)) return true
    return (o1 > 0.0) !== (o2 > 0.0) && (o3 > 0.0) !== (o4 > 0.0)
}

// 고정 사각형 (x_lo, y_lo, x_hi, y_hi) 에 대한 닫힌 포함 판정.
export function pointInRect(p: Point, rect: [number, number, number, number]): boolean {
    const [xLo, yLo, xHi, yHi] = rect
    return xLo <= p[0] && p[0] <= xHi && yLo <= p[1] && p[1] <= yHi
}

// 닫힌 세그먼트 a→b가 닫힌 셀 정사각형과 만나는가 — 랜드마크 가시성 판정.
export function segmentIntersectsRect(a: Point, b: Point,
                                      rect: [number, number, number, number]): boolean {
    const [xLo, yLo, xHi, yHi] = rect
    if (pointInRect(a, rect) || pointInRect(b, rect)) return true
    const c0: Point = [xLo, yLo]
    const c1: Point = [xHi, yLo]
    const c2: Point = [xHi, yHi]
    const c3: Point = [xLo, yHi]
    const edges: Array<[Point, Point]> = [[c0, c1], [c1, c2], [c2, c3], [c3, c0]]
    for (const e of edges) {
        if (segmentsIntersect(a, b, e[0], e[1])) return true
    }
    return false
}
