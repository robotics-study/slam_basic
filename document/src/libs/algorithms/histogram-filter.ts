// python/slam/filtering/histogram_filter.py의 라이브 미러 — 이산 자세 공간(자유 셀 ×
// heading 격자) 위의 완전한 Bayes 필터. JS 산술은 libm과 ulp 단위로 다를 수 있어서
// (사이트 문서의 사실) 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 — 패리티
// 체커의 허용 오차(1e-9) 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다:
// 관측-major/state-minor 우도 누적(상태별 합승은 스캔 순서 = j 오름차순으로 동일하게
// 쌓인다), ascending readout, 첫 최대 우선 argmax. H 테이블은 JS에서 통째로 살리지 않고
// R 플랫 테이블 + 회전 인덱스((b + j − off) mod B)로 gather한다 — 같은 값, 1/360 메모리.
import {cellToWorld, GridMap, inBounds, occupiedAt, worldToCell} from "../grid"
import {poseCompose, wrap} from "../geometry"
import {buildEpisode, raycast} from "../sim"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Point, Pose, TraceEvent, Twist} from "../trace/types"

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// params 는 demo가 시나리오 센서로 주입한 그대로의 값(beam 시나리오는 beams/fov_deg/
// range_max/sigma_range, seed는 시나리오에서) — 엔진은 센서를 오직 이 파라미터로만 본다.
export function runHistogramFilter(grid: GridMap, scenario: ScenarioJson,
                                   params: Record<string, ParamValue>): TraceEvent[] {
    const pSlip = Number(params["p_slip"])
    const sigmaRange = Number(params["sigma_range"])
    const beams = Number(params["beams"])
    const fovDeg = Number(params["fov_deg"])
    const rangeMax = Number(params["range_max"])

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor, null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    // --- init_state: 균일 사전분포 + 격자 + DDA 테이블 (ascending (fi, k)) ---
    const h = grid.height, w = grid.width, res = grid.resolution
    // sim.ts의 observe()와 동일한 식 — 필터의 눈금이 스캔의 각도와 같은 방식으로 계산된다.
    const fov = fovDeg * Math.PI / 180.0
    const stepA = fov / (beams - 1)
    const half = fov / 2.0
    const bins = Math.round(2.0 * Math.PI / stepA)
    const off = Math.floor((beams - 1) / 2)
    if (!(Math.abs(bins * stepA - 2.0 * Math.PI) < 1e-9)) throw new Error("histogram_filter: heading lattice not closed")
    if (!(Math.abs(off * stepA - half) < 1e-9)) throw new Error("histogram_filter: heading lattice not aligned to beams")

    const freeR: number[] = [], freeC: number[] = []
    const cxArr: number[] = [], cyArr: number[] = []
    const fiOfCell = new Int32Array(h * w).fill(-1)
    for (let r = 0; r < h; r++) {
        for (let c = 0; c < w; c++) {
            if (occupiedAt(grid, r, c)) continue
            fiOfCell[r * w + c] = cxArr.length
            freeR.push(r)
            freeC.push(c)
            // 자유 셀의 world 중심 — 좌표 프레임은 격자 레이어만 소유한다 (cellToWorld).
            const center = cellToWorld(grid, [r, c])
            cxArr.push(center[0])
            cyArr.push(center[1])
        }
    }
    const nFree = cxArr.length
    const nStates = nFree * bins
    const miss = rangeMax + res

    // R: 자유 셀 × 격자 각도 하나의 DDA — python과 같은 (fi, k) 오름차순.
    const rFlat = new Float64Array(nStates)
    {
        let sIdx = 0
        for (let fi = 0; fi < nFree; fi++) {
            for (let k = 0; k < bins; k++) {
                const hit = raycast(grid, cxArr[fi], cyArr[fi], -Math.PI + k * stepA, rangeMax)
                rFlat[sIdx++] = hit === null ? miss : hit
            }
        }
    }

    let belief = new Float64Array(nStates).fill(1.0 / nStates)
    const poses: Pose[] = []

    // --- predict: child-of-movement + slip 혼합, slide semantics (python과 같은 순서) ---
    function predict(u: Twist): void {
        const newP = new Float64Array(nStates)
        for (let fi = 0; fi < nFree; fi++) {
            const cx = cxArr[fi], cy = cyArr[fi]
            const base = fi * bins
            for (let b = 0; b < bins; b++) {
                const p = belief[base + b]
                if (p === 0.0) continue // 정확히 0 — 더해져도 어떤 합도 바꿀 수 없다
                const moved = poseCompose([cx, cy, -Math.PI + b * stepA], u)
                const [qr, qc] = worldToCell(grid, [moved[0], moved[1]])
                let qb = Math.floor((moved[2] + Math.PI) / stepA + 0.5)
                if (qb >= bins) qb -= bins
                // slide semantics: 격자 밖/점유 셀로의 이동은 움직이지 않는다.
                let qCi = freeR[fi] * w + freeC[fi]
                if (inBounds(grid, qr, qc) && fiOfCell[qr * w + qc] >= 0) qCi = qr * w + qc
                const q = fiOfCell[qCi] * bins + qb
                newP[q] += (1.0 - pSlip) * p
                newP[base + b] += pSlip * p
            }
        }
        belief = newP
    }

    // --- update: prior와의 Bayes 곱 (log space). 상태별 합승은 스캔 점 순서(= j 오름차순)로
    // 쌓인다 — python의 observation-major 누적과 같은 수열. ---
    function updateScan(scan: Point[]): void {
        const kCount = scan.length
        const rArr = new Float64Array(kCount)
        const jArr = new Int32Array(kCount)
        for (let k = 0; k < kCount; k++) {
            const zx = scan[k][0], zy = scan[k][1]
            rArr[k] = Math.sqrt(zx * zx + zy * zy)
            const beta = Math.atan2(zy, zx)
            let j = Math.floor((beta + half) / stepA + 0.5)
            if (j < 0) j = 0
            if (j > beams - 1) j = beams - 1
            jArr[k] = j
        }
        const ll = new Float64Array(nStates)
        for (let k = 0; k < kCount; k++) {
            const rv = rArr[k]
            // b=0에서의 회전 인덱스 — b가 오를 때 m은 순환 증가한다 ((b + j − off) mod B).
            let m = jArr[k] - off
            if (m < 0) m += bins
            for (let fi = 0; fi < nFree; fi++) {
                const base = fi * bins
                let mm = m
                for (let b = 0; b < bins; b++) {
                    const d = (rv - rFlat[base + mm]) / sigmaRange
                    ll[base + b] += (-0.5 * d) * d
                    mm++
                    if (mm === bins) mm = 0
                }
            }
        }
        let mMax = Number.NEGATIVE_INFINITY
        for (let s = 0; s < nStates; s++) if (ll[s] > mMax) mMax = ll[s]
        const wList = new Float64Array(nStates)
        let total = 0.0
        for (let s = 0; s < nStates; s++) {
            const pPrior = belief[s]
            const wv = pPrior === 0.0 ? 0.0 : pPrior * Math.exp(ll[s] - mMax)
            wList[s] = wv
            total += wv
        }
        if (total !== 0.0) {
            for (let s = 0; s < nStates; s++) belief[s] = wList[s] / total
        }
    }

    // --- 이벤트 스트림 — python의 데모 드라이버와 같은 순서: run_started, 스텝마다
    // step_observed → pose_estimated → belief_updated, 마지막에 run_finished. ---
    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "histogram_filter",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
    })

    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.scan !== undefined ? {scan: step.scan} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})
        if (step.odom !== undefined) predict(step.odom)
        if (step.scan !== undefined && step.scan.length > 0) updateScan(step.scan)

        // readout (고정 순서): 상태 오름차순의 위치 평균 → 빈 주변변 argmax(첫 최대 우선)
        // → 모수 표준편차.
        let xHat = 0.0, yHat = 0.0
        const binMarginal = new Float64Array(bins)
        for (let fi = 0; fi < nFree; fi++) {
            const cx = cxArr[fi], cy = cyArr[fi]
            const base = fi * bins
            for (let b = 0; b < bins; b++) {
                const p = belief[base + b]
                xHat += p * cx
                yHat += p * cy
                binMarginal[b] += p
            }
        }
        let thetaHat = -Math.PI
        let best = Number.NEGATIVE_INFINITY
        for (let b = 0; b < bins; b++) {
            if (binMarginal[b] > best) {
                best = binMarginal[b]
                thetaHat = -Math.PI + b * stepA
            }
        }
        let varX = 0.0, varY = 0.0, varT = 0.0
        for (let fi = 0; fi < nFree; fi++) {
            const cx = cxArr[fi], cy = cyArr[fi]
            const base = fi * bins
            for (let b = 0; b < bins; b++) {
                const p = belief[base + b]
                const dx = cx - xHat
                const dy = cy - yHat
                const dt = wrap(-Math.PI + b * stepA - thetaHat)
                varX += p * (dx * dx)
                varY += p * (dy * dy)
                varT += p * (dt * dt)
            }
        }
        const pose: Pose = [xHat, yHat, thetaHat]
        const cov: [number, number, number] = [Math.sqrt(varX), Math.sqrt(varY), Math.sqrt(varT)]
        events.push({seq: seq++, event: "pose_estimated", t: step.t, pose, cov})

        // EVERY cell row-major ([row, col, p]); 점유 셀은 상태가 아니다 — 정확히 0.0.
        const cells: Array<[number, number, number]> = []
        for (let ci = 0; ci < h * w; ci++) {
            const fi = fiOfCell[ci]
            let pCell = 0.0
            if (fi >= 0) {
                const base = fi * bins
                for (let b = 0; b < bins; b++) pCell += belief[base + b]
            }
            cells.push([Math.floor(ci / w), ci % w, pCell])
        }
        events.push({seq: seq++, event: "belief_updated", t: step.t, cells})

        poses.push(pose)
    }

    const result: EstimateResult = {poses}
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
