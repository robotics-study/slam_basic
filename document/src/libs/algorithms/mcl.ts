// python/slam/filtering/mcl.py의 라이브 미러 — KLD-sampling MCL(Fox, Burgard,
// Dellaert & Thrun AAAI 1999 + Fox IJRR 2003). JS 산술은 libm과 ulp 단위로 다를 수
// 있어서(사이트 문서의 사실) 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 —
// 패리티 체커의 허용 오차(1e-9) 안에서 같다. 그래서 이 파일은 연산 순서를 그대로
// 미러한다: t=0 고정 사전분포는 입자 오름차순 드로, t≥1 KLD 루프는 표본을 하나씩 —
// 조상 워크(strict `>`), 가우시안 셋, x ⊕ (u + ε), 스캔 순서 우도, floor 빈 키.
import {GridMap, worldToCell} from "../grid"
import {poseCompose, wrap} from "../geometry"
import {buildEpisode, raycast} from "../sim"
import {Rng} from "../rng"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Point, Pose, TraceEvent, Twist} from "../trace/types"
import {invNormCdf} from "../stats"

// Fox 2003 식 (14) — n = χ²_{k−1,1−δ}/2ε 의 Wilson–Hilferty 폐형. k ≤ 1 은 퇴화
// 케이스(0.0: 빈 하나가 만드는 belief는 n_min 밖의 보장이 필요 없다). 좌결합 t·t·t.
export function kldBound(k: number, epsilon: number, zQ: number): number {
    if (k <= 1) return 0.0
    const nu = k - 1.0
    const q9 = 2.0 / (9.0 * nu)
    const t = (1.0 - q9) + zQ * Math.sqrt(q9)
    return (nu / (2.0 * epsilon)) * t * t * t
}

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
export function runMcl(grid: GridMap, scenario: ScenarioJson,
                       params: Record<string, ParamValue>): TraceEvent[] {
    const x0 = Number(params["x0"]), y0 = Number(params["y0"])
    const epsilon = Number(params["epsilon"]), delta = Number(params["delta"])
    const binXy = Number(params["bin_xy"]), binTheta = Number(params["bin_theta"])
    const nMin = Number(params["n_min"]), maxParticles = Number(params["max_particles"])
    const sigmaRange = Number(params["sigma_range"])
    const sigmaXy = Number(params["sigma_xy"]), sigmaTheta = Number(params["sigma_theta"])
    const rangeMax = Number(params["range_max"])

    // 상한의 양자화는 설정의 상수 — 여기서 한 번만 계산한다.
    const zQ = invNormCdf(1.0 - delta)

    // 알고리즘 자신의 스트림 — 시뮬레이터의 노이즈 스트림과 같은 값에서 시작하지만 독립.
    const rng = new Rng(Number(params["seed"]))
    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor, null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    let xs: number[] = [], ys: number[] = [], ths: number[] = [], ws: number[] = []
    let initialized = false

    // 스캔 점마다 (r, β) — 로봇 프레임 끝점의 극좌표. 고정 식: sqrt(zx²+zy²), atan2(zy,zx).
    function polarPoints(scan: Point[]): Point[] {
        const points: Point[] = []
        for (const z of scan) points.push([Math.sqrt(z[0] * z[0] + z[1] * z[1]), Math.atan2(z[1], z[0])])
        return points
    }

    // --- init: t=0는 고정 크기 사전분포(적응성은 첫 KLD 스텝부터) — 입자 오름차순 드로.
    function initParticles(): void {
        const [ox, oy] = grid.origin
        const res = grid.resolution
        const h = grid.height
        const [row0, col0] = worldToCell(grid, [x0, y0])
        for (let i = 0; i < maxParticles; i++) {
            xs.push(ox + (col0 + rng.uniform01()) * res)
            ys.push(oy + ((h - 1 - row0) + rng.uniform01()) * res)
            ths.push(rng.uniform01() * (2.0 * Math.PI) - Math.PI)
        }
        ws = new Array<number>(maxParticles).fill(1.0 / maxParticles) // 균일 — 정확히 1/N
        initialized = true
    }

    // --- t = 0 뿐: 모든 가중치에 exp(ll − max ll)를 곱하고 정규화(particle_filter와
    // 같은 고정 패턴 — miss는 sentinel range_max + res).
    function weight(points: Point[]): void {
        const sentinel = rangeMax + grid.resolution
        const n = ws.length
        let m = Number.NEGATIVE_INFINITY
        const ll: number[] = []
        for (let i = 0; i < n; i++) {
            let acc = 0.0
            for (const pt of points) {
                const hit = raycast(grid, xs[i], ys[i], ths[i] + pt[1], rangeMax)
                const e = hit === null ? sentinel : hit
                const d = (pt[0] - e) / sigmaRange
                acc += (-0.5 * d) * d
            }
            ll.push(acc)
            if (acc > m) m = acc // max는 정확 — 순서에 무관(구성상)
        }
        let total = 0.0
        const weighted: number[] = []
        for (let i = 0; i < n; i++) {
            const wv = ws[i] * Math.exp(ll[i] - m)
            weighted.push(wv)
            total += wv
        }
        if (total !== 0.0) { // 합이 0이면 믿음이 그대로 남는다(문서화된 퇴화 케이스)
            for (let i = 0; i < n; i++) ws[i] = weighted[i] / total
        }
    }

    // --- KLD-sampling 스텝 하나: 상한이 커버를 보증할 때까지(또는 상한선까지) 표본을
    // 하나씩 resample-move-reweight. 조상 워크는 strict `>` — multinomial 리샘플 그대로,
    // KLD는 언제 멈추는가만 바꾼다. 처음 밟은 빈은 k를 올리고 n_chi를 다시 계산한다.
    function kldUpdate(u: Twist, scan?: Point[]): void {
        const sentinel = rangeMax + grid.resolution
        const points = scan !== undefined && scan.length > 0 ? polarPoints(scan) : []
        const xOld = xs, yOld = ys, thOld = ths, wOld = ws
        const nOld = wOld.length
        const xn: number[] = [], yn: number[] = [], tn: number[] = [], lln: number[] = []
        const seen = new Set<string>() // 빈 키 (floor(x/bin_xy), floor(y/bin_xy), floor(θ/bin_θ))
        let k = 0, n = 0
        let m = Number.NEGATIVE_INFINITY
        let nChi = Number.POSITIVE_INFINITY
        while (true) {
            const u1 = rng.uniform01()
            let i = 0
            let c = wOld[0]
            while (i < nOld - 1 && u1 > c) { // strict `>` 워크 — 고정 순서
                i += 1
                c += wOld[i]
            }
            const ex = rng.gaussian(0.0, sigmaXy)
            const ey = rng.gaussian(0.0, sigmaXy)
            const et = rng.gaussian(0.0, sigmaTheta)
            const p = poseCompose([xOld[i], yOld[i], thOld[i]], [u[0] + ex, u[1] + ey, u[2] + et])
            let acc = 0.0
            for (const pt of points) {
                const hit = raycast(grid, p[0], p[1], p[2] + pt[1], rangeMax)
                const e = hit === null ? sentinel : hit
                const d = (pt[0] - e) / sigmaRange
                acc += (-0.5 * d) * d
            }
            xn.push(p[0]); yn.push(p[1]); tn.push(p[2]); lln.push(acc)
            if (acc > m) m = acc // max는 정확 — 순서에 무관(구성상)
            const key = `${Math.floor(p[0] / binXy)},${Math.floor(p[1] / binXy)},${Math.floor(p[2] / binTheta)}`
            if (!seen.has(key)) { seen.add(key); k += 1 }
            n += 1
            if (n >= nMin) nChi = kldBound(k, epsilon, zQ)
            if ((n >= nChi && n >= nMin) || n >= maxParticles) break
        }
        // 오래된 가중치는 이미 행동했다 — 조상 워크를 굴린 것이 그들이다.
        let total = 0.0
        const weighted: number[] = []
        for (let i = 0; i < n; i++) {
            const wv = Math.exp(lln[i] - m)
            weighted.push(wv)
            total += wv
        }
        const wNew: number[] = []
        for (let i = 0; i < n; i++) wNew.push(weighted[i] / total)
        xs = xn; ys = yn; ths = tn; ws = wNew
    }

    const poses: Pose[] = []
    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "mcl",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
    })

    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.scan !== undefined ? {scan: step.scan} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})
        if (!initialized) {
            initParticles()
            if (step.scan !== undefined && step.scan.length > 0) weight(polarPoints(step.scan))
        } else if (step.odom !== undefined) {
            kldUpdate(step.odom, step.scan)
        }

        // 판독(고정 순서, particle_filter 페이지와 동일): 가중 평균 x, y를 오름차순으로;
        // circular mean 헤딩; 그다음 두 번째 오름차순 패스의 population 분산.
        let xHat = 0.0, yHat = 0.0, sSin = 0.0, sCos = 0.0
        const n = ws.length
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
