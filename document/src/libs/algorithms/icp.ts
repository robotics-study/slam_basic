// python/slam/registration/icp.py의 라이브 미러 — 점-점 ICP 스캔 매칭(Besl & McKay
// 1992), registration 갈래의 첫 회원: 오도메트리 없이 직전 스캔→도착 스캔 페어링만으로
// 자세를 복원한다. JS 산술은 libm과 ulp 단위로 다를 수 있어서(사이트 문서의 사실)
// 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 — 패리티 체커의 허용 오차(1e-9)
// 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다: 소스 점을 스캔 순서로,
// 최근접 탐색은 오름차순 strict `<`(타이는 낮은 인덱스), d_max² 초과 쌍 폐기, 중심 정렬
// 합승 오름차순, atan2 하나, 갱신 후 max(|Δdx|,|Δdy|,|Δθ|) ≤ eps 고정점 판정.
import {GridMap} from "../grid"
import {poseCompose} from "../geometry"
import {buildEpisode} from "../sim"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Point, Pose, TraceEvent, Twist} from "../trace/types"

// ICP 해 하나: source를 target에 얹는 SE(2) 원소. 고정 순서 — 소스 점을 스캔 순서로
// 돌며 현재 추정으로 변환하고(코사인 먼저), 제곱 거리 최근접 대상점을 오름차순
// strict `<`로 찾고(타이는 낮은 인덱스), 그 거리가 d_max² 이하면 쌍을 보존(truncated
// least squares), 남은 쌍 위에서 폐형해(중심 정렬 + atan2 하나)를 반복. sqrt는 어디에도
// 없다 — 거리 비교는 전부 제곱 그대로. 빈 입력/전원 폐기는 정체(identity) 반환.
export function icpStep(source: Point[], target: Point[], dMax: number, eps: number, maxIters: number): Twist {
    if (source.length === 0 || target.length === 0) return [0.0, 0.0, 0.0]
    const dm2 = dMax * dMax // 제곱은 한 번 — 아래에 sqrt는 어디에도 없다
    let ax = 0.0, ay = 0.0, ang = 0.0
    for (let it = 0; it < maxIters; it++) {
        const ca = Math.cos(ang) // 코사인 먼저 — 고정 순서
        const sa = Math.sin(ang)
        const qx: number[] = [], qy: number[] = [], px: number[] = [], py: number[] = []
        for (const s of source) { // 오름차순 스캔 순서 — 고정
            const tx = ca * s[0] - sa * s[1] + ax
            const ty = sa * s[0] + ca * s[1] + ay
            // 제곱 거리 최근접: j=0을 먼저, 이후 오름차순 strict `<` — 타이 낮은 인덱스.
            let bestD = (tx - target[0][0]) * (tx - target[0][0]) + (ty - target[0][1]) * (ty - target[0][1])
            let bestJ = 0
            for (let j = 1; j < target.length; j++) {
                const dd = (tx - target[j][0]) * (tx - target[j][0]) + (ty - target[j][1]) * (ty - target[j][1])
                if (dd < bestD) { // strict — 타이에는 낮은 인덱스가 남는다
                    bestD = dd
                    bestJ = j
                }
            }
            if (bestD <= dm2) { // truncated least squares: d_max 넘으면 쌍이 없다
                qx.push(s[0]); qy.push(s[1])
                px.push(target[bestJ][0]); py.push(target[bestJ][1])
            }
        }
        const n = qx.length
        if (n === 0) return [ax, ay, ang] // 정렬할 것이 남지 않았다 — 추정을 그대로 실어 나른다
        let qxBar = 0.0, qyBar = 0.0, pxBar = 0.0, pyBar = 0.0
        for (let j = 0; j < n; j++) { // 오름차순 중심 합 — 고정
            qxBar += qx[j]; qyBar += qy[j]
            pxBar += px[j]; pyBar += py[j]
        }
        qxBar /= n; qyBar /= n
        pxBar /= n; pyBar /= n
        let num = 0.0, den = 0.0
        for (let j = 0; j < n; j++) { // 중심 잡힌 외적/내적 합, 오름차순 — 고정
            const zx = qx[j] - qxBar
            const zy = qy[j] - qyBar
            const wx = px[j] - pxBar
            const wy = py[j] - pyBar
            num += zx * wy - zy * wx
            den += zx * wx + zy * wy
        }
        // Besl & McKay의 SVD가 2D에서 접히는 자리: atan2 하나, 그리고 중심을 맞출 이동.
        const dNew = Math.atan2(num, den)
        const cr = Math.cos(dNew)
        const sr = Math.sin(dNew)
        const axNew = pxBar - (cr * qxBar - sr * qyBar)
        const ayNew = pyBar - (sr * qxBar + cr * qyBar)
        // 수렴: 세 성분의 max, 갱신 AFTER 판정.
        const moved = Math.max(Math.abs(axNew - ax), Math.abs(ayNew - ay), Math.abs(dNew - ang))
        ax = axNew; ay = ayNew; ang = dNew
        if (moved <= eps) break // "대응한 뒤 푼다"의 고정점
    }
    return [ax, ay, ang]
}

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// 이 알고리즘은 센서 파라미터를 선언하지 않는다 — 스캔 점은 그대로 소비되고, 게이지
// (x0/y0/theta_deg)가 절대 좌표의 선언이다. t=0은 선언된 게이지 포즈를 그대로 방출하고
// (절대 좌표는 측정이 아니라 선언이다), 이후 모든 스텝은 도착 스캔을 직전 스캔에 맞춰
// 복원한 twist를 합성한다. odom은 절대 소비되지 않는다 — 그것이 이 갈래다.
export function runIcp(grid: GridMap, scenario: ScenarioJson,
                       params: Record<string, ParamValue>): TraceEvent[] {
    const x0 = Number(params["x0"]), y0 = Number(params["y0"])
    const thetaDeg = Number(params["theta_deg"])
    const dMax = Number(params["d_max"]), eps = Number(params["eps"])
    const maxIters = Number(params["max_iters"])

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor, null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    let hasPose = false
    let pose: Pose = [0.0, 0.0, 0.0]
    let prev: Point[] = [] // 직전 스텝의 스캔(로봇 프레임) — 아직 없으면 빈 배열

    const poses: Pose[] = []
    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "icp",
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
            const twist = icpStep(step.scan !== undefined ? step.scan : [], prev, dMax, eps, maxIters)
            pose = poseCompose(pose, twist)
            if (step.scan !== undefined) prev = step.scan // 스캔 없는 스텝은 타건을 그대로 둔다
        }
        poses.push(pose)
        // cov 없음 — ICP는 불확실성 모델을 싣지 않는다(그 위가 filter_based 갈래의 일).
        events.push({seq: seq++, event: "pose_estimated", t: step.t, pose})
    }

    const result: EstimateResult = {poses}
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
