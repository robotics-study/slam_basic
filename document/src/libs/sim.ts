// python/slam/core/sim.py의 정확한 미러 — GT 궤적 재샘플, 오도메트리 노이즈, DDA 레이캐스트,
// 랜드마크 관측 생성. 드로 순서(결정성 계약): 각 스텝에서 관측 노이즈를 먼저 그리고
// (beam: 빔 0..beams-1 순서; landmarks: id 오름차순, range then bearing) 다음에 다음 이동의
// 오도메트리 노이즈를 그린다. 스텝 0은 도착한 명령이 없어 odom이 없고 마지막 스텝은 드로하지 않는다.
import {GridMap, inBounds, occupiedAt, worldToCell} from "./grid"
import {poseMinus, robotToWorld, segmentIntersectsRect, wrap} from "./geometry"
import {Rng} from "./rng"
import {LandmarkObs, Point, Pose, SensorConfig, Twist} from "./trace/types"

export interface Step {
    t: number;
    gt: Pose;
    odom?: Twist;
    scan?: Point[];
    obs?: LandmarkObs[];
}

export interface Episode {
    steps: Step[];
    landmarks: Point[] | null;
    grid: GridMap;
}

// 웨이포인트 폴리라인의 등아크 재샘플 (계약 정확한 순서). 마지막 점은 항상 폴리라인 끝.
export function resample(path: Point[], stepMeters: number): Pose[] {
    const segLen: number[] = []
    let total = 0.0
    for (let k = 0; k < path.length - 1; k++) {
        const dx = path[k + 1][0] - path[k][0]
        const dy = path[k + 1][1] - path[k][1]
        const segL = Math.sqrt(dx * dx + dy * dy)
        segLen.push(segL)
        total += segL
    }
    const points: Point[] = []
    let s = 0.0
    while (s < total) {
        points.push(pointAtArc(path, segLen, s))
        s += stepMeters
    }
    points.push([path[path.length - 1][0], path[path.length - 1][1]])

    const poses: Pose[] = []
    for (let k = 0; k < points.length; k++) {
        let theta: number
        if (k + 1 < points.length) {
            theta = Math.atan2(points[k + 1][1] - points[k][1], points[k + 1][0] - points[k][0])
        } else {
            theta = poses[poses.length - 1][2] // 마지막 점은 이전 heading 유지
        }
        poses.push([points[k][0], points[k][1], theta])
    }
    return poses
}

// 아크 길이 s의 폴리라인 점 — 남은 길이를 덮는 첫 세그먼트를 찾아 선형 보간 (고정 walk).
function pointAtArc(path: Point[], segLen: number[], s: number): Point {
    let acc = 0.0
    for (let k = 0; k < segLen.length; k++) {
        if (s - acc <= segLen[k]) {
            const frac = (s - acc) / segLen[k]
            const ax = path[k][0], ay = path[k][1]
            const bx = path[k + 1][0], by = path[k + 1][1]
            return [ax + frac * (bx - ax), ay + frac * (by - ay)]
        }
        acc += segLen[k]
    }
    return path[path.length - 1] // s >= total: 폴리라인 끝
}

// Amanatides & Woo grid DDA — 고정 공식(두 언어가 동일하게 계산한다). tMaxX == tMaxY 타이
// 는 Y 스텝을 택하고(cos>0이면 step_y=-1이라 위쪽 행), 코사인 0 축은 절대 스텝하지 않는다.
export function raycast(map: GridMap, x: number, y: number, phi: number, rangeMax: number): number | null {
    const cdx = Math.cos(phi)
    const sdy = Math.sin(phi)
    const [r0, c0] = worldToCell(map, [x, y])
    let r = r0, c = c0
    if (!inBounds(map, r, c)) return null
    if (occupiedAt(map, r, c)) return 0.0
    const [ox, oy] = map.origin
    const res = map.resolution
    const h = map.height

    let stepX: number, tMaxX: number
    if (cdx > 0.0) {
        stepX = 1
        tMaxX = (ox + (c + 1) * res - x) / cdx
    } else if (cdx < 0.0) {
        stepX = -1
        tMaxX = (ox + c * res - x) / cdx
    } else {
        stepX = 0
        tMaxX = Infinity
    }
    let stepY: number, tMaxY: number
    if (sdy > 0.0) {
        stepY = -1 // world y는 row 인덱스가 내려갈수록 커진다 — 행 r의 TOP 가장자리.
        tMaxY = (oy + (h - r) * res - y) / sdy
    } else if (sdy < 0.0) {
        stepY = 1
        tMaxY = (oy + (h - 1 - r) * res - y) / sdy
    } else {
        stepY = 0
        tMaxY = Infinity
    }
    const deltaX = cdx !== 0.0 ? res / Math.abs(cdx) : Infinity
    const deltaY = sdy !== 0.0 ? res / Math.abs(sdy) : Infinity

    for (;;) {
        let hitT: number
        if (tMaxX < tMaxY) {
            hitT = tMaxX
            if (hitT >= rangeMax) return null
            c += stepX
            tMaxX += deltaX
        } else {
            hitT = tMaxY
            if (hitT >= rangeMax) return null
            r += stepY
            tMaxY += deltaY
        }
        if (!inBounds(map, r, c)) return null
        if (occupiedAt(map, r, c)) return hitT
    }
}

// 세그먼트 from→lm가 점유 셀 정사각형과 만나지 않으면 보인다 (bbox 스킵 순서까지 미러).
export function landmarkVisible(map: GridMap, from: Point, lm: Point): boolean {
    const minX = Math.min(from[0], lm[0])
    const maxX = Math.max(from[0], lm[0])
    const minY = Math.min(from[1], lm[1])
    const maxY = Math.max(from[1], lm[1])
    const [ox, oy] = map.origin
    const res = map.resolution
    const h = map.height
    const w = map.width
    for (let r = 0; r < h; r++) {
        const yLo = oy + (h - 1 - r) * res
        const yHi = oy + (h - r) * res
        if (yLo > maxY || yHi < minY) continue
        for (let c = 0; c < w; c++) {
            const xLo = ox + c * res
            const xHi = ox + (c + 1) * res
            if (xLo > maxX || xHi < minX) continue
            if (occupiedAt(map, r, c) && segmentIntersectsRect(from, lm, [xLo, yLo, xHi, yHi])) {
                return false
            }
        }
    }
    return true
}

// 한 스텝의 관측 — 계약 순서로 드로한다.
function observe(rng: Rng, map: GridMap, gt: Pose, sensor: SensorConfig,
                 landmarks: Point[] | null): { scan?: Point[], obs?: LandmarkObs[] } {
    if (sensor.type === "beam") {
        const beams = sensor.beams as number
        const fov = (sensor.fov_deg as number) * Math.PI / 180.0
        const half = fov / 2.0
        const stepA = fov / (beams - 1)
        const scan: Point[] = []
        for (let i = 0; i < beams; i++) {
            const phi = gt[2] - half + i * stepA
            const rHit = raycast(map, gt[0], gt[1], phi, sensor.range_max)
            if (rHit === null) continue // miss는 점을 내지 않는다
            const rNoisy = rHit + rng.gaussian(0.0, sensor.sigma_range)
            const e: Point = [gt[0] + Math.cos(phi) * rNoisy, gt[1] + Math.sin(phi) * rNoisy]
            scan.push(robotToWorld(e, gt)) // 스캔 점은 로봇 프레임으로 저장된다
        }
        return {scan}
    }
    const obs: LandmarkObs[] = []
    const lms = landmarks as Point[]
    for (let lmId = 0; lmId < lms.length; lmId++) {
        const lm = lms[lmId]
        const dx = lm[0] - gt[0]
        const dy = lm[1] - gt[1]
        const dist = Math.sqrt(dx * dx + dy * dy)
        if (dist > sensor.range_max || !landmarkVisible(map, [gt[0], gt[1]], lm)) continue
        const rNoisy = dist + rng.gaussian(0.0, sensor.sigma_range)
        const bearingExact = wrap(Math.atan2(dy, dx) - gt[2])
        const bNoisy = wrap(bearingExact + rng.gaussian(0.0, sensor.sigma_bearing as number))
        obs.push({id: lmId, bearing: bNoisy, range: rNoisy})
    }
    return {obs}
}

// 시나리오에서 Step 스트림 전체를 조립한다 — python build_episode의 미러.
export function buildEpisode(map: GridMap, path: Point[], stepMeters: number,
                             sensor: SensorConfig, landmarks: Point[] | null,
                             sigmaXy: number, sigmaTheta: number, seed: number): Episode {
    const poses = resample(path, stepMeters)
    const rng = new Rng(seed)
    const steps: Step[] = []
    let uArriving: Twist | undefined = undefined
    const last = poses.length - 1
    for (let t = 0; t < poses.length; t++) {
        const {scan, obs} = observe(rng, map, poses[t], sensor, landmarks)
        steps.push({t, gt: poses[t], odom: uArriving, scan, obs})
        if (t < last) {
            const uExact = poseMinus(poses[t], poses[t + 1])
            const ex = rng.gaussian(0.0, sigmaXy)
            const ey = rng.gaussian(0.0, sigmaXy)
            const etheta = rng.gaussian(0.0, sigmaTheta)
            uArriving = [uExact[0] + ex, uExact[1] + ey, wrap(uExact[2] + etheta)]
        }
    }
    return {steps, landmarks, grid: map}
}
