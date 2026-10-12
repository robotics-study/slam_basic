// python/slam/filter_based/ekf_slam.py의 라이브 미러 — Extended Kalman Filter SLAM (Smith &
// Cheeseman, ICRA 1986), filter_based 갈래의 첫 회원: 지도가 추정기 안에 산다. 앞선 모든
// 갈래가 association을 주어진 것으로 소비했을 때 이 페이지는 관측이association 이후에 무엇을
// 하는지를 보여준다 — 상태를 증강한다. 상태 벡터는 x = [px, py, θ | l_0x, l_0y, l_1x, …]이고
// 하나의 조밀 공분산 Σ가 포즈와 랜드마크 전부를 함께 싣는다. JS 산술은 libm과 ulp 단위로
// 다를 수 있어서(사이트 문서의 사실) 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 —
// 패리티 체커의 허용 오차(1e-9) 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다:
// 오름차승 k 행렬곱, 상삼각만 계산하고 하삼각은 복사, Joseph 형식, 출생은 update가 아닌 증분.

import {GridMap} from "../grid"
import {poseCompose, robotToWorld, wrap} from "../geometry"
import {buildEpisode} from "../sim"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Pose, TraceEvent} from "../trace/types"

// 계약의 고정 순서 그대로인 조밀 행렬곱: out[i][j]는 a[i][k]·b[k][j]를 k 오름차순으로 +0.0에서
// 시작하는 누적기에 더한다. 어떤 항도 건너뛰지 않는다 — 합 자체들이 계약이다.
function matmul(a: number[][], b: number[][]): number[][] {
    const n = a.length, m = b[0].length
    const out: number[][] = []
    for (let i = 0; i < n; i++) {
        const rowA = a[i]
        const rowOut: number[] = []
        for (let j = 0; j < m; j++) {
            let acc = 0.0
            for (let k = 0; k < rowA.length; k++) acc += rowA[k] * b[k][j]
            rowOut.push(acc)
        }
        out.push(rowOut)
    }
    return out
}

// out[i][j] = Σ_k a[i][k]·h[j][k], k 오름차순 — 오른쪽 인자가 전치된 곱 (양쪽 갈래의 ΣH,
// predict의 A Fᵀ, 출생의 −G B⁻ᵀ).
function matmulAtB(a: number[][], h: number[][]): number[][] {
    const n = a.length, m = h.length
    const out: number[][] = []
    for (let i = 0; i < n; i++) {
        const rowA = a[i]
        const rowOut: number[] = []
        for (let j = 0; j < m; j++) {
            let acc = 0.0
            const rowH = h[j]
            for (let k = 0; k < rowA.length; k++) acc += rowA[k] * rowH[k]
            rowOut.push(acc)
        }
        out.push(rowOut)
    }
    return out
}

// 명시적 2×2 역행렬 — det과 수반행렬, 솔버 없음: det = m00·m11 − m01·m10.
function invert2(m: number[][]): number[][] {
    const det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    return [[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]]
}

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// t = 0은 선언된 게이지 포즈를 정확히 영 공분산으로 채택하고(게이지 관례), 이후 스텝은
// predict(포즈 합성 + FΣFᵀ + Q) 후 관측을 id 오름차순으로 소비한다: 알려진 slot은 Joseph
// 형식 표준 갱신, 알려지지 않은 id는 증분(평균 = 역투영 — 혁신이 구성상 0이므로 평균 보정이
// 없고, 공분산만 B⁻¹WB⁻ᵀ 블록 + 크로스 −GB⁻ᵀ로 자란다). 이벤트 순서는 pose_estimated(cov)
// 먼저, landmarks_updated(slot 순서, sx/sy 포함) 나중.
export function runEkfSlam(grid: GridMap, scenario: ScenarioJson,
                           params: Record<string, ParamValue>): TraceEvent[] {
    const x0 = Number(params["x0"]), y0 = Number(params["y0"])
    const thetaDeg = Number(params["theta_deg"])
    // Q — 필터가 스스로 선언하는 운동 노이즈(시나리오가 주입하는 값과 같은 값을 선언한다).
    const sigmaXy = Number(params["sigma_xy"]), sigmaTheta = Number(params["sigma_theta"])
    const qPos = sigmaXy * sigmaXy, qTheta = sigmaTheta * sigmaTheta
    // R — 관측 노이즈, 관측 공간에서 대각(거리 m, 방위각 rad — 단위가 다르므로 비등방).
    const sigmaRange = Number(params["sigma_range"]), sigmaBearing = Number(params["sigma_bearing"])
    const rRange = sigmaRange * sigmaRange, rBearing = sigmaBearing * sigmaBearing

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor,
        scenario.landmarks ?? null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    // 상태: x[0..2]는 포즈, 그다음 발견된 랜드마크마다 (x, y) 쌍 — 관측 순서. ids는 slot k →
    // 시나리오 landmark id(id가 순서 밖으로 올 때 slot ≠ id — 교육점이고 숨기지 않는다).
    let x: number[] = []
    let p: number[][] = [] // 조밀 공분산 — 같은 slot 레이아웃
    const ids: number[] = []
    const poses: Pose[] = []

    // predict: x̂ ← x ⊕ u(시뮬레이터가 GT를 미분할 쓴 그 pose_compose), Σ̄ = FΣFᵀ + Q.
    function predict(dxO: number, dyO: number, dThetaO: number): void {
        const px = x[0], py = x[1], th = x[2]
        const c = Math.cos(th), s = Math.sin(th)
        // J = ∂(x ⊕ u)/∂x를 사전 평균에서 — 고정 식.
        const j02 = -((s * dxO) + (c * dyO))
        const j12 = (c * dxO) - (s * dyO)
        // 포즈 삼조는 정본에 시뮬레이터가 쓴 것과 같은 pose_compose로 전파되고 랜드마크
        // slot은 정적 — 평균 성분은 정확한 복사 그대로 지나간다(F의 항등 블록이 평균에 한 일).
        const pNew = poseCompose([px, py, th], [dxO, dyO, dThetaO])
        const newX = [pNew[0], pNew[1], pNew[2]]
        for (let i = 3; i < x.length; i++) newX.push(x[i])
        x = newX
        const n = x.length
        const f: number[][] = Array.from({length: n}, () => new Array<number>(n).fill(0.0))
        f[0][0] = 1.0
        f[0][2] = j02
        f[1][1] = 1.0
        f[1][2] = j12
        f[2][2] = 1.0
        for (let i = 3; i < n; i++) f[i][i] = 1.0 // 정적 랜드마크 — 항등 행/열
        const a = matmul(f, p) // A = FΣ (오름차승 k; 곱해진 0 항은 아무것도 바꾸지 않는다)
        const pNext: number[][] = Array.from({length: n}, () => new Array<number>(n).fill(0.0))
        for (let i = 0; i < n; i++) { // Σ̄ = A·Fᵀ — 상삼각 오름차승 계산, 하삼각은 복사(정확)
            const rowA = a[i]
            for (let j = i; j < n; j++) {
                let acc = 0.0
                const rowF = f[j]
                for (let k = 0; k < n; k++) acc += rowA[k] * rowF[k]
                pNext[i][j] = acc
                pNext[j][i] = acc
            }
        }
        pNext[0][0] += qPos // Q는 포즈 대각에 slot 순서로 앉는다
        pNext[1][1] += qPos
        pNext[2][2] += qTheta
        p = pNext
    }

    // 하나의 관측, id 오름차순. 알려진 slot → 표준 EKF 갱신(Joseph 형식); 알려지지 않음 →
    // 증분(부정합 사전 분포의 극한 — 파일 머리말 참조).
    function observe(obsId: number, zRange: number, zBearing: number): void {
        const px = x[0], py = x[1], th = x[2]
        let slot = -1
        for (let k = 0; k < ids.length; k++) { // association은 주어진 것: 선형 id 탐색
            if (ids[k] === obsId) {
                slot = k
                break
            }
        }

        const n0 = x.length
        if (slot < 0) {
            // ---- 출생. 평균: 관측을 현재 포즈로 역투영 — 측정 모델의 정확한 역이라 이 관측의
            // 혁신이 구성상 정확히 0이고 평균 보정은 애초에 존재할 수 없다(알려지지 않은 랜드마크는
            // 포즈에 정보를 싣지 않는다 — 출생이 뜻하는 것이 그것이다).
            const zx = zRange * Math.cos(zBearing)
            const zy = zRange * Math.sin(zBearing)
            const w = robotToWorld([zx, zy], [px, py, th])
            const lx = w[0], ly = w[1]
            const dx = lx - px, dy = ly - py
            const rHat = Math.sqrt(dx * dx + dy * dy)
            const beta = Math.atan2(dy, dx)
            const cb = Math.cos(beta), sb = Math.sin(beta)
            const n = n0
            // H를 현재 평균에서 — 행 3.. 은 영(이 랜드마크는 아직 상태에 없다) — 그래서 출생은
            // 포즈를 움직일 수 없다.
            const h: number[][] = Array.from({length: n}, () => [0.0, 0.0])
            h[0][0] = -cb
            h[1][0] = -sb
            h[2][0] = 0.0
            h[0][1] = sb / rHat
            h[1][1] = -cb / rHat
            h[2][1] = -1.0
            const g = matmul(p, h) // G = ΣH — 갱신과 같은 고정 순서 곱
            const wMat = [[0.0, 0.0], [0.0, 0.0]]
            let w01 = 0.0
            for (let i = 0; i < n; i++) {
                wMat[0][0] += h[i][0] * g[i][0]
                w01 += h[i][0] * g[i][1]
                wMat[1][1] += h[i][1] * g[i][1]
            }
            wMat[0][1] = w01
            wMat[1][0] = w01 // 대칭은 복사로(정확)
            wMat[0][0] += rRange
            wMat[1][1] += rBearing
            // B = ∂h/∂landmark(det = 1/r̂ ≠ 0): 행 0은 거리 기울기, 행 1은 방위각 기울기.
            const bInv = invert2([[cb, sb], [-sb / rHat, cb / rHat]])
            // 새 대각 블록 B⁻¹WB⁻ᵀ(범용 곱; 상삼각 계산 후 하삼각 복사), 크로스 열 −GB⁻ᵀ —
            // t = 0에서는 Σ_pp가 아직 영이라 정확히 ±0.
            const tMat = matmul(bInv, wMat)
            const nb = matmulAtB(tMat, bInv)
            nb[1][0] = nb[0][1] // B⁻¹WB⁻ᵀ는 대칭 — 아래 항을 복사(정확)
            const cross = matmulAtB(g, bInv)
            // 상태를 키운다: 평균 쌍을 append하고 Σ를 열 한 쌍과 그 거울 행으로 키운다(복사가
            // 행렬을 정확히 대칭으로 유지한다).
            x.push(lx)
            x.push(ly)
            const grown: number[][] = []
            for (let i = 0; i < n; i++) {
                const row = p[i].slice()
                row.push(-cross[i][0])
                row.push(-cross[i][1])
                grown.push(row)
            }
            const colX: number[] = [], colY: number[] = []
            for (let i = 0; i < n; i++) {
                colX.push(grown[i][n]) // 방금 append한 크로스 열(slot n)
                colY.push(grown[i][n + 1])
            }
            grown.push([...colX, nb[0][0], nb[0][1]])
            grown.push([...colY, nb[1][0], nb[1][1]])
            p = grown
            ids.push(obsId)
            return
        }

        // ---- 재관측: 기존 slot에 표준 EKF 갱신.
        const c1 = 3 + 2 * slot
        const lx = x[c1], ly = x[c1 + 1]
        const dx = lx - px, dy = ly - py
        const rHat = Math.sqrt(dx * dx + dy * dy)
        const beta = Math.atan2(dy, dx)
        const cb = Math.cos(beta), sb = Math.sin(beta)
        const yR = zRange - rHat // 거리는 감지 않는다(wrap 없음)
        const yB = wrap(zBearing - wrap(beta - th)) // 방위각은 시뮬레이터처럼 감싼다
        const n = n0
        const h: number[][] = Array.from({length: n}, () => [0.0, 0.0])
        h[0][0] = -cb
        h[1][0] = -sb
        h[2][0] = 0.0
        h[0][1] = sb / rHat
        h[1][1] = -cb / rHat
        h[2][1] = -1.0
        h[c1][0] = cb
        h[c1][1] = -sb / rHat
        h[c1 + 1][0] = sb
        h[c1 + 1][1] = cb / rHat
        const g = matmul(p, h) // G = ΣH (n×2), 오름차승 순서
        const sMat = [[0.0, 0.0], [0.0, 0.0]]
        let s01 = 0.0
        for (let i = 0; i < n; i++) {
            sMat[0][0] += h[i][0] * g[i][0]
            s01 += h[i][0] * g[i][1]
            sMat[1][1] += h[i][1] * g[i][1]
        }
        sMat[0][1] = s01
        sMat[1][0] = s01 // 대칭은 복사로(정확)
        sMat[0][0] += rRange
        sMat[1][1] += rBearing
        const sInv = invert2(sMat)
        const kGain: number[][] = []
        for (let i = 0; i < n; i++) {
            const gi = g[i]
            kGain.push([gi[0] * sInv[0][0] + gi[1] * sInv[1][0],
                        gi[0] * sInv[0][1] + gi[1] * sInv[1][1]])
        }
        // 평균 갱신 — 성분별 인덱스 순서; θ는 자기 덧셈 뒤에 다시 감싼다.
        x[0] += kGain[0][0] * yR + kGain[0][1] * yB
        x[1] += kGain[1][0] * yR + kGain[1][1] * yB
        x[2] = wrap(x[2] + (kGain[2][0] * yR + kGain[2][1] * yB))
        for (let i = 3; i < n; i++) {
            x[i] += kGain[i][0] * yR + kGain[i][1] * yB
        }
        // Joseph 형식: Σ ← AΣAᵀ + KRKᵀ, A = I − KH(고정 오름차승 행렬곱; 상삼각 계산 후 복사).
        const aMat: number[][] = []
        for (let i = 0; i < n; i++) {
            const row: number[] = []
            for (let j = 0; j < n; j++) {
                let acc = 0.0
                for (let k = 0; k < 2; k++) acc += kGain[i][k] * h[j][k]
                row.push((i === j ? 1.0 : 0.0) - acc)
            }
            aMat.push(row)
        }
        const mMat = matmul(aMat, p) // M = AΣ (오름차승 k)
        const sigmaNew: number[][] = Array.from({length: n}, () => new Array<number>(n).fill(0.0))
        for (let i = 0; i < n; i++) {
            const rowM = mMat[i]
            const tI0 = kGain[i][0] * rRange // (KR)[i][0] — R은 대각, 정확한 곱들
            const tI1 = kGain[i][1] * rBearing
            for (let j = i; j < n; j++) {
                let acc = 0.0
                const rowJ = aMat[j]
                for (let k = 0; k < n; k++) acc += rowM[k] * rowJ[k]
                acc += tI0 * kGain[j][0] + tI1 * kGain[j][1]
                sigmaNew[i][j] = acc
                sigmaNew[j][i] = acc
            }
        }
        p = sigmaNew
    }

    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "ekf_slam",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
        ...(scenario.landmarks !== undefined ? {landmarks: scenario.landmarks} : {}),
    })

    let initialized = false
    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})
        if (!initialized) {
            // t = 0은 선언된 게이지 포즈를 정확히 영 공분산으로 채택한다 — 이 저장소 전체의
            // 게이지 관례: 출발 자세는 알려 있다.
            x = [x0, y0, thetaDeg * Math.PI / 180]
            p = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            initialized = true
        } else {
            if (step.odom === undefined) // 계약: t ≥ 1은 항상 명령을 싣고 온다
                throw new Error(`ekf_slam: step ${step.t} arrived without an odom command`)
            predict(step.odom[0], step.odom[1], step.odom[2])
        }
        if (step.obs !== undefined) {
            for (const o of step.obs) observe(o.id, o.range, o.bearing) // id 오름차순 — 고정 순서
        }
        const pose: Pose = [x[0], x[1], x[2]]
        poses.push(pose)
        const cov: [number, number, number] = [Math.sqrt(p[0][0]), Math.sqrt(p[1][1]), Math.sqrt(p[2][2])]
        events.push({seq: seq++, event: "pose_estimated", t: step.t, pose, cov})
        const estimated: Array<{ id: number; x: number; y: number; sx?: number; sy?: number }> = []
        for (let k = 0; k < ids.length; k++) {
            estimated.push({
                id: ids[k], x: x[3 + 2 * k], y: x[4 + 2 * k],
                sx: Math.sqrt(p[3 + 2 * k][3 + 2 * k]), sy: Math.sqrt(p[4 + 2 * k][4 + 2 * k]),
            })
        }
        events.push({seq: seq++, event: "landmarks_updated", t: step.t, estimated})
    }

    const result: EstimateResult = {poses, landmarks: []}
    for (let k = 0; k < ids.length; k++) {
        result.landmarks!.push({
            id: ids[k], x: x[3 + 2 * k], y: x[4 + 2 * k],
            sx: Math.sqrt(p[3 + 2 * k][3 + 2 * k]), sy: Math.sqrt(p[4 + 2 * k][4 + 2 * k]),
        })
    }
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
