// python/slam/filtering/particle_filter.py의 라이브 미러 — 연속 자세 (x, y, θ) 위의
// 부트스트랩 입자 필터. JS 산술은 libm과 ulp 단위로 다를 수 있어서(사이트 문서의 사실)
// 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 — 패리티 체커의 허용 오차(1e-9)
// 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다: 입자 오름차순 드로
// (가우시안 = 균일 2드로), 스캔 순서 우도 누적(max-shifted exp, 옴부 정렬), 체계적
// 리샘플의 strict `>` 워크와 마지막 무사용 u += 1/N 까지. Rng 는 splitmix64 BigInt 미러라
// 드로 자체는 비트 단위로 같다.
import {GridMap, worldToCell} from "../grid"
import {poseCompose, wrap} from "../geometry"
import {buildEpisode, raycast} from "../sim"
import {Rng} from "../rng"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Point, Pose, TraceEvent, Twist} from "../trace/types"

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// seed 는 plain 파라미터다(주입된 값 = 시나리오 seed 와 같은 수, 그러나 알고리즘 자신의
// 독립 스트림). 센서 필드는 알고리즘이 오직 range_max/sigma_range 로만 본다.
export function runParticleFilter(grid: GridMap, scenario: ScenarioJson,
                                  params: Record<string, ParamValue>): TraceEvent[] {
    const n = Number(params["n_particles"])
    const x0 = Number(params["x0"]), y0 = Number(params["y0"])
    const rangeMax = Number(params["range_max"]), sigmaRange = Number(params["sigma_range"])
    const sigmaXy = Number(params["sigma_xy"]), sigmaTheta = Number(params["sigma_theta"])

    // 알고리즘 자신의 스트림 — 시뮬레이터의 노이즈 스트림과 같은 값에서 시작하지만 독립.
    const rng = new Rng(Number(params["seed"]))
    const invN = 1.0 / n

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor, null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    let xs: number[] = [], ys: number[] = [], ths: number[] = [], ws: number[] = []
    let initialized = false

    // --- init: 출발 셀(x0/y0 가 속한 셀) 균일 × heading [−π, π) 전체 — 입자 오름차순 드로.
    function initParticles(): void {
        const [ox, oy] = grid.origin
        const res = grid.resolution
        const h = grid.height
        const [row0, col0] = worldToCell(grid, [x0, y0])
        for (let i = 0; i < n; i++) {
            xs.push(ox + (col0 + rng.uniform01()) * res)
            ys.push(oy + ((h - 1 - row0) + rng.uniform01()) * res)
            ths.push(rng.uniform01() * (2.0 * Math.PI) - Math.PI)
        }
        ws = new Array<number>(n).fill(invN) // 균일 사전분포 — 정확히 1/N
        initialized = true
    }

    // --- predict: x ⊕ (u + ε), 입자 오름차순 드로(가우시안마다 균일 두 번).
    function move(u: Twist): void {
        for (let i = 0; i < n; i++) {
            const ex = rng.gaussian(0.0, sigmaXy)
            const ey = rng.gaussian(0.0, sigmaXy)
            const et = rng.gaussian(0.0, sigmaTheta)
            const p = poseCompose([xs[i], ys[i], ths[i]], [u[0] + ex, u[1] + ey, u[2] + et])
            xs[i] = p[0]; ys[i] = p[1]; ths[i] = p[2]
        }
    }

    // --- update: 스캔 점마다 (r, β)를 스캔 순서로 재구성하고 입자 자체 자세에서 raycast
    // 기댓값의 가우시안을 곱한다(miss 는 sentinel range_max + res 그대로). max-shifted exp를
    // 오름차순으로 쌓아 오름차순 정규화; 합이 0이면 믿음이 그대로 남는다(문서화된 퇴화 케이스).
    function weight(scan: Point[]): void {
        const res = grid.resolution
        const sentinel = rangeMax + res
        const rs: number[] = [], bs: number[] = []
        for (const z of scan) {
            rs.push(Math.sqrt(z[0] * z[0] + z[1] * z[1]))
            bs.push(Math.atan2(z[1], z[0]))
        }
        let m = Number.NEGATIVE_INFINITY
        const ll: number[] = []
        for (let i = 0; i < n; i++) {
            const s = ths[i]
            let acc = 0.0
            for (let k = 0; k < rs.length; k++) {
                const hit = raycast(grid, xs[i], ys[i], s + bs[k], rangeMax)
                const e = hit === null ? sentinel : hit
                const d = (rs[k] - e) / sigmaRange
                acc += (-0.5 * d) * d
            }
            ll.push(acc)
            if (acc > m) m = acc // max 는 정확 — 순서에 무관하게(구성상)
        }
        let total = 0.0
        const weighted: number[] = []
        for (let i = 0; i < n; i++) {
            const wv = ws[i] * Math.exp(ll[i] - m)
            weighted.push(wv)
            total += wv
        }
        if (total !== 0.0) {
            for (let i = 0; i < n; i++) ws[i] = weighted[i] / total
        }
    }

    // --- resample: ESS = 1/Σw² 가 N/2 아래로 떨어지면 체계적 리샘플 — 단일 드로를
    // [0, 1/N)에 스케일하고 누적 가중치 워크는 strict `>`, 매 복사 후 u += 1/N(마지막은 무사용).
    function resampleIfEffective(): void {
        let essDen = 0.0
        for (let i = 0; i < n; i++) {
            const wv = ws[i]
            essDen += wv * wv
        }
        const ess = 1.0 / essDen
        if (ess < n / 2.0) {
            let u = rng.uniform01() * invN
            let c = ws[0], i = 0
            const nx: number[] = [], ny: number[] = [], nt: number[] = []
            for (let j = 0; j < n; j++) {
                while (i < n - 1 && u > c) {
                    i += 1
                    c += ws[i]
                }
                nx.push(xs[i]); ny.push(ys[i]); nt.push(ths[i])
                u += invN // 마지막 무사용 증분까지 — 고정 루프 형태
            }
            xs = nx; ys = ny; ths = nt
            ws = new Array<number>(n).fill(invN) // 리샘플 후 균일 — 정확히 1/N
        }
    }

    const poses: Pose[] = []
    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "particle_filter",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
    })

    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.scan !== undefined ? {scan: step.scan} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})
        if (!initialized) initParticles()
        if (step.odom !== undefined) move(step.odom)
        if (step.scan !== undefined && step.scan.length > 0) weight(step.scan)
        resampleIfEffective()

        // 판독(고정 순서): 가중 평균 x, y 를 오름차순으로; circular mean 헤딩; 그다음
        // 두 번째 오름차순 패스의 population 분산(θ 는 seam 을 통과해 wrap).
        let xHat = 0.0, yHat = 0.0, sSin = 0.0, sCos = 0.0
        for (let i = 0; i < n; i++) {
            const wv = ws[i]
            xHat += wv * xs[i]
            yHat += wv * ys[i]
            sSin += wv * Math.sin(ths[i])
            sCos += wv * Math.cos(ths[i])
        }
        const thetaHat = Math.atan2(sSin, sCos)
        let varX = 0.0, varY = 0.0, varT = 0.0
        for (let i = 0; i < n; i++) {
            const wv = ws[i]
            const dx = xs[i] - xHat
            const dy = ys[i] - yHat
            const dt = wrap(ths[i] - thetaHat)
            varX += wv * (dx * dx)
            varY += wv * (dy * dy)
            varT += wv * (dt * dt)
        }
        const pose: Pose = [xHat, yHat, thetaHat]
        const cov: [number, number, number] = [Math.sqrt(varX), Math.sqrt(varY), Math.sqrt(varT)]
        poses.push(pose)
        const cloud: Array<[number, number, number, number]> = []
        for (let i = 0; i < n; i++) cloud.push([xs[i], ys[i], ths[i], ws[i]])
        events.push({seq: seq++, event: "pose_estimated", t: step.t, pose, cov})
        events.push({seq: seq++, event: "particles_updated", t: step.t, particles: cloud})
    }

    const result: EstimateResult = {poses}
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
