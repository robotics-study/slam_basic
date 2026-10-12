// python/slam/features/dbscan.py의 라이브 미러 — Density-Based Spatial Clustering of
// Applications with Noise (Ester, Kriegel, Sander & Xu, KDD 1996), features 갈래의 첫
// 회원. 이 갈래는 랜드마크가 어디서 오는지를 가르친다: filter_based 계열이 association을
// 주어진 것으로 소비할 때, 그 id가 여기서 태어난다. JS 산술은 libm과 ulp 단위로 다를 수
// 있어서(사이트 문서의 사실) 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 — 패리티
// 체커의 허용 오차(1e-9) 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다:
// 오름차순 인덱스 방문, 닫힌 공(dx²+dy² ≤ eps² — sqrt 없음), 자기 자신을 세는 min_pts,
// sticky 노이즈, 발견 순서 = id, append 순서 명시적 += 중심점.

import {GridMap} from "../grid"
import {poseCompose, robotToWorld} from "../geometry"
import {buildEpisode} from "../sim"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {ParamValue, Point, Pose, TraceEvent} from "../trace/types"

// 라벨 센티널 — 논문의 UNVISITED / NOISE를 int로 (클러스터 id는 k ≥ 0).
const UNVISITED = -2
const NOISE = -1

// 인덱스 j(오름차순, 자기 자신 포함) 중 dx² + dy² ≤ eps² — 닫힌 공을 제곱 거리로 비교하니
// sqrt가 어디에도 없다. 자기 자신은 항상 안쪽(0 ≤ eps²)이고 그래서 min_pts가 자기를 센다.
function neighborhood(points: Point[], i: number, epsSq: number): number[] {
    const xi = points[i][0], yi = points[i][1]
    const nb: number[] = []
    for (let j = 0; j < points.length; j++) { // 오름차순 — 고정
        const dx = points[j][0] - xi
        const dy = points[j][1] - yi
        if (dx * dx + dy * dy <= epsSq) nb.push(j)
    }
    return nb
}

// Ester Algorithm 1을 점 목록 위 인덱스 순서로. 클러스터는 발견 순서로 반환되고 멤버는
// append 순서(씨앗 먼저)로 앉는다. NOISE 라벨 점은 어디에도 나타나지 않는다. 씨앗은
// ExpandCluster보다 먼저 라벨링되고, core인 q는 자신이 아직 UNVISITED이므로 성장 목록에
// 자기를 다시 append 한다 — 다시 만난 중복 항목은 건너뛴다; 둘 다 언어 간 계약의 일부다.
export function dbscanCluster(points: Point[], eps: number, minPts: number): Point[][] {
    const n = points.length
    const labels: number[] = new Array(n).fill(UNVISITED)
    const epsSq = eps * eps
    const clusters: Point[][] = []
    for (let i = 0; i < n; i++) { // 오름차순 방문 순서 — 고정
        if (labels[i] !== UNVISITED) continue // 이미 멤버이거나 이미 sticky 노이즈
        const nb = neighborhood(points, i, epsSq)
        if (nb.length < minPts) {
            labels[i] = NOISE // 방문 시점에 드물면 → 영원히 노이즈(sticky)
            continue
        }
        const k = clusters.length // 발견 순서가 곧 id
        const members: Point[] = [points[i]] // 씨앗 먼저, append 순서
        clusters.push(members)
        labels[i] = k
        let j = 0
        while (j < nb.length) { // ExpandCluster: 성장하는 목록, 인덱스 포인터 — 고정
            const q = nb[j]
            j++
            if (labels[q] !== UNVISITED) continue // 이미 멤버(두 번일 수도)이거나 sticky 노이즈
            const qnb = neighborhood(points, q, epsSq)
            if (qnb.length >= minPts) { // q가 core → 그 근방이 전선을 확장한다
                for (const m of qnb) { // 오름차순 append; labels 는 처리 시점에 필터
                    if (labels[m] === UNVISITED) nb.push(m)
                }
            }
            labels[q] = k
            members.push(points[q])
        }
    }
    return clusters
}

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// 이 알고리즘은 센서 파라미터를 선언하지 않는다 — 게이지(x0/y0/theta_deg)와 군집 두 값만
// 소비한다. t=0은 선언된 게이지 포즈를 그대로 방출하고 이후 모든 스텝은 도착한 오도메트리를
// 합성한다(이 갈래는 오도메트리를 소비한다 — registration 갈래의 정반대). 스캔 점은 빔
// 순서로 world로 올려 누적에 추가되고 매 스텝 전체를 다시 군집한다. 이벤트 순서는
// pose_estimated 먼저, landmarks_updated 나중.
export function runDbscan(grid: GridMap, scenario: ScenarioJson,
                          params: Record<string, ParamValue>): TraceEvent[] {
    const x0 = Number(params["x0"]), y0 = Number(params["y0"])
    const thetaDeg = Number(params["theta_deg"])
    const eps = Number(params["eps"]), minPts = Number(params["min_pts"])

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor,
        scenario.landmarks ?? null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)

    let hasPose = false
    let pose: Pose = [0.0, 0.0, 0.0]
    const points: Point[] = [] // 누적 world 점 — 스테프 우선, 빔 오름차순
    const poses: Pose[] = []
    let landmarks: Array<{ id: number; x: number; y: number }> = []

    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "dbscan",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
        ...(scenario.landmarks !== undefined ? {landmarks: scenario.landmarks} : {}),
    })

    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.scan !== undefined ? {scan: step.scan} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})
        if (!hasPose) {
            // t = 0은 선언된 게이지 포즈를 채택한다 — math.radians와 같은 식.
            pose = [x0, y0, thetaDeg * Math.PI / 180]
            hasPose = true
        } else {
            if (step.odom === undefined) // 계약: t ≥ 1은 항상 명령을 싣고 온다
                throw new Error(`dbscan: step ${step.t} arrived without an odom command`)
            pose = poseCompose(pose, step.odom)
        }
        poses.push(pose)

        if (step.scan !== undefined) {
            for (const z of step.scan) points.push(robotToWorld(z, pose)) // 빔 오름차순 — 고정
        }
        const clusters = dbscanCluster(points, eps, minPts)
        const estimated: Array<{ id: number; x: number; y: number }> = []
        for (let k = 0; k < clusters.length; k++) { // 발견 순서가 곧 id
            const members = clusters[k]
            let mx = 0.0, my = 0.0
            const n = members.length
            for (const p of members) { mx += p[0]; my += p[1] } // append 순서 — 고정; 보정 합 아님
            estimated.push({id: k, x: mx / n, y: my / n})
        }
        landmarks = estimated
        // cov 없음 — dbscan은 불확실성 모델을 싣지 않는다(그 위가 filter_based 갈래의 일).
        events.push({seq: seq++, event: "pose_estimated", t: step.t, pose})
        events.push({seq: seq++, event: "landmarks_updated", t: step.t, estimated})
    }

    const result: EstimateResult = {poses, landmarks}
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
