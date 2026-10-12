// python/slam/registration/ndt.py의 라이브 미러 — Normal Distributions Transform
// (Biber & Strasser, IROS 2003)을 스캔-투-필드 매칭으로: 대응을 만들지 않는다. 기준
// 스캔을 셀별 확률 분포(평균 + RAW 산포 행렬)로 컴파일하고 도착한 스캔을 그 필드에
// 그대로 점수화한다. JS 산술은 libm과 ulp 단위로 다를 수 있어서(사이트 문서의 사실)
// 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 — 패리티 체커의 허용 오차(1e-9)
// 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다: 스캔 순서 버킷팅, 셀마다
// 좌결합 합산, exp(−m/2)의 m = qᵀS⁻¹q를 (dxq·u1 + dyq·u2)/2 로 고정 계산, λ 배가 루프,
// row-0 여인수 크래머 풀이, 이진에서 정확한 절반 줄이기. 버킷 키는 "(ix,iy)" 문자열 —
// 조회는 키로만 이루어지므로 Map 삽입 순서는 비트에 영향 없다(python dict와 동일 계약).

import {GridMap} from "../grid"
import {poseCompose} from "../geometry"
import {buildEpisode} from "../sim"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Point, Pose, TraceEvent, Twist} from "../trace/types"

// 한 셀의 분포: [μx, μy, Sxx, Sxy, Syy, det S] — 평균은 n으로 정규화, 산포는 RAW.
// 점을 3개 미만 담거나 산포 행렬식(스칼라)이 정확히 0.0 인 셀은 필드에 없다.
export type CellDist = [number, number, number, number, number, number]

// 점을 스캔 순서로 (floor(x/cell), floor(y/cell)) 버킷에 담고 셀마다 분포를 맞힌다 —
// 합산은 삽입 순서 좌결합(python 의 명시적 루프와 동일; 내장 sum 은 3.12+ 보정 합이라
// python 쪽도 루프를 쓴다). n < 3 스킵, det 정확히 0.0 스킵.
export function buildNdt(points: Point[], cellSize: number): Map<string, CellDist> {
    const buckets = new Map<string, Point[]>()
    for (const p of points) { // 스캔 순서 — 고정
        const key = `${Math.floor(p[0] / cellSize)},${Math.floor(p[1] / cellSize)}`
        const bucket = buckets.get(key)
        if (bucket === undefined) buckets.set(key, [p])
        else bucket.push(p)
    }
    const dists = new Map<string, CellDist>()
    for (const [key, pts] of buckets) {
        const n = pts.length
        if (n < 3) continue // 점 두 개로는 평면을 못 잰다 — 분포 없음
        let mx = 0.0, my = 0.0
        for (const p of pts) { mx += p[0]; my += p[1] } // 좌결합 합산 — 고정
        mx /= n; my /= n
        let sxx = 0.0, syy = 0.0, sxy = 0.0
        for (const p of pts) {
            const dx = p[0] - mx, dy = p[1] - my
            sxx += dx * dx; syy += dy * dy; sxy += dx * dy
        }
        const det = sxx * syy - sxy * sxy
        if (det === 0.0) continue // 축에 정확히 정렬된 퇴화 클러스터 — 분포 없음
        dists.set(key, [mx, my, sxx, sxy, syy, det])
    }
    return dists
}

// score(a): 소스 점 하나당 가우스 bump 하나를 스캔 순서로 더한 합. 변환된 점이 분포 없는
// 셀에 닿으면 기여는 정확히 0. 이차형식은 (dxq·u1 + dyq·u2)/2 — 아래 기울기가 미분하는
// 그 표현 그대로.
export function ndtScore(source: Point[], dists: Map<string, CellDist>, cellSize: number,
                         ax: number, ay: number, ang: number): number {
    let s = 0.0
    const c = Math.cos(ang) // 코사인 먼저 — 고정 순서
    const sn = Math.sin(ang)
    for (const q of source) { // 오름차순 스캔 순서 — 고정
        const x = c * q[0] - sn * q[1] + ax
        const y = sn * q[0] + c * q[1] + ay
        const d = dists.get(`${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`)
        if (d === undefined) continue // 분포 없음 — 기여 정확히 0
        const [mx, my, sxx, sxy, syy, det] = d
        const ixx = syy / det, iyy = sxx / det, ixy = -sxy / det // S⁻¹ = adj(S)/det(S)
        const dxq = x - mx, dyq = y - my
        const u1 = ixx * dxq + ixy * dyq
        const u2 = ixy * dxq + iyy * dyq
        s += Math.exp(-(dxq * u1 + dyq * u2) / 2.0)
    }
    return s
}

// NDT 해 하나: source 를 target 의 필드 위에 얹는 SE(2) 원소. 고정 순서 — 스캔 순서
// 축적, 실베스터 주소식이 모두 양수가 될 때까지 λ 배가, row-0 여인수 크래머, 엄격한
// 개선이 있을 때만 수용(아니면 이진 정확 절반), max|Δ| ≤ eps 면 고정점에서 선다. 빈 입력
// 어느 한쪽이라도 identity twist — 정보 없으면 운동 없음.
export function ndtStep(source: Point[], target: Point[], cellSize: number,
                        lam0: number, eps: number, maxIters: number): Twist {
    if (source.length === 0 || target.length === 0) return [0.0, 0.0, 0.0]
    const dists = buildNdt(target, cellSize)
    let ax = 0.0, ay = 0.0, ang = 0.0
    for (let it = 0; it < maxIters; it++) {
        const g = [0.0, 0.0, 0.0]
        const h = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        const ca = Math.cos(ang), sa = Math.sin(ang)
        for (const q of source) { // 오름차순 스캔 순서 — 고정
            const x = ca * q[0] - sa * q[1] + ax
            const y = sa * q[0] + ca * q[1] + ay
            const found = dists.get(`${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`)
            if (found === undefined) continue
            const [mx, my, sxx, sxy, syy, det] = found
            const ixx = syy / det, iyy = sxx / det, ixy = -sxy / det
            const dxq = x - mx, dyq = y - my
            const u1 = ixx * dxq + ixy * dyq
            const u2 = ixy * dxq + iyy * dyq
            const e = Math.exp(-(dxq * u1 + dyq * u2) / 2.0)
            const j3x = -sa * q[0] - ca * q[1]
            const j3y = ca * q[0] - sa * q[1]
            const qddx = -(ca * q[0] - sa * q[1])
            const qddy = -(sa * q[0] + ca * q[1])
            const u3 = j3x * u1 + j3y * u2
            const w11 = ixx, w12 = ixy, w22 = iyy
            const w13 = ixx * j3x + ixy * j3y
            const w23 = ixy * j3x + iyy * j3y
            const w33 = (qddx * u1 + qddy * u2) +
                (j3x * (ixx * j3x + ixy * j3y) + j3y * (ixy * j3x + iyy * j3y))
            g[0] += e * u1; g[1] += e * u2; g[2] += e * u3
            h[0][0] += e * (-u1 * u1 + w11)
            h[0][1] = h[1][0] = h[0][1] + e * (-u1 * u2 + w12)
            h[0][2] = h[2][0] = h[0][2] + e * (-u1 * u3 + w13)
            h[1][1] += e * (-u2 * u2 + w22)
            h[1][2] = h[2][1] = h[1][2] + e * (-u2 * u3 + w23)
            h[2][2] += e * (-u3 * u3 + w33)
        }
        // λ 는 lam0 에서 시작해 H+λI 의 실베스터 주소식이 모두 양수가 될 때까지 배가 —
        // H 의 원소가 유한하므로 유한 종료.
        let lam = lam0
        const hl = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        let det = 0.0
        for (;;) {
            for (let i = 0; i < 3; i++)
                for (let j = 0; j < 3; j++) hl[i][j] = h[i][j] + (i === j ? lam : 0.0)
            const m1 = hl[0][0]
            const m2 = hl[0][0] * hl[1][1] - hl[0][1] * hl[1][0]
            det = hl[0][0] * (hl[1][1] * hl[2][2] - hl[1][2] * hl[2][1]) -
                hl[0][1] * (hl[1][0] * hl[2][2] - hl[1][2] * hl[2][0]) +
                hl[0][2] * (hl[1][0] * hl[2][1] - hl[1][1] * hl[2][0])
            if (m1 > 0.0 && m2 > 0.0 && det > 0.0) break
            lam *= 2.0
        }
        // 크래머 규칙, row-0 여인수 전개 — 고정 순서.
        const step = [0.0, 0.0, 0.0]
        for (let k = 0; k < 3; k++) {
            const aMat = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
            for (let i = 0; i < 3; i++)
                for (let j = 0; j < 3; j++) aMat[i][j] = hl[i][j]
            for (let i = 0; i < 3; i++) aMat[i][k] = -g[i]
            const num = aMat[0][0] * (aMat[1][1] * aMat[2][2] - aMat[1][2] * aMat[2][1]) -
                aMat[0][1] * (aMat[1][0] * aMat[2][2] - aMat[1][2] * aMat[2][0]) +
                aMat[0][2] * (aMat[1][0] * aMat[2][1] - aMat[1][1] * aMat[2][0])
            step[k] = num / det
        }
        const sCur = ndtScore(source, dists, cellSize, ax, ay, ang)
        let d0 = step[0], d1 = step[1], d2 = step[2]
        for (;;) { // 엄격한 개선이 있어야만 수용 — 절반 줄이기는 이진에서 정확
            if (ndtScore(source, dists, cellSize, ax + d0, ay + d1, ang + d2) > sCur) break
            if (Math.max(Math.abs(d0), Math.max(Math.abs(d1), Math.abs(d2))) <= eps)
                return [ax, ay, ang] // 고정점: 어떤 절반 수준도 개선하지 못한다
            d0 /= 2.0; d1 /= 2.0; d2 /= 2.0
        }
        ax += d0; ay += d1; ang += d2
        if (Math.max(Math.abs(d0), Math.max(Math.abs(d1), Math.abs(d2))) <= eps) break
    }
    return [ax, ay, ang]
}

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// 이 알고리즘은 센서 파라미터를 선언하지 않는다 — 게이지(x0/y0/theta_deg)와 솔버 세 값만
// 소비한다. t=0은 선언된 게이지 포즈를 그대로 방출하고(절대 좌표는 측정이 아니라 선언),
// 이후 모든 스텝은 도착 스캔을 직전 스캔의 필드에 점수화해 복원한 twist를 합성한다.
// odom은 절대 소비되지 않는다 — 그것이 이 갈래다.
export function runNdt(grid: GridMap, scenario: ScenarioJson,
                       params: Record<string, ParamValue>): TraceEvent[] {
    const x0 = Number(params["x0"]), y0 = Number(params["y0"])
    const thetaDeg = Number(params["theta_deg"])
    const cellSize = Number(params["cell_size"]), lam0 = Number(params["lam0"])
    const eps = Number(params["eps"]), maxIters = Number(params["max_iters"])

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor, null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    let hasPose = false
    let pose: Pose = [0.0, 0.0, 0.0]
    let prev: Point[] = [] // 직전 스텝의 스캔(로봇 프레임) — 아직 없으면 빈 배열

    const poses: Pose[] = []
    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "ndt",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
    })

    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.scan !== undefined ? {scan: step.scan} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})
        if (!hasPose) {
            // t = 0은 선언된 게이지 포즈를 채택한다 — math.radians와 같은 식.
            pose = [x0, y0, thetaDeg * Math.PI / 180]
            prev = step.scan !== undefined ? step.scan : []
            hasPose = true
        } else {
            const twist = ndtStep(step.scan !== undefined ? step.scan : [], prev,
                cellSize, lam0, eps, maxIters)
            pose = poseCompose(pose, twist)
            if (step.scan !== undefined) prev = step.scan // 스캔 없는 스텝은 타건을 그대로 둔다
        }
        poses.push(pose)
        // cov 없음 — NDT는 불확실성 모델을 싣지 않는다(그 위가 filter_based 갈래의 일).
        events.push({seq: seq++, event: "pose_estimated", t: step.t, pose})
    }

    const result: EstimateResult = {poses}
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
