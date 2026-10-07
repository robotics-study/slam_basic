// python/slam/filtering/grid_mapping.py의 라이브 미러 — 알려진 자세(무노이즈 오도메트리 +
// 선언된 출발 자세) 위의 log-odds 점유 격자. JS 산술은 libm과 ulp 단위로 다를 수 있어서
// (사이트 문서의 사실) 브라우저 실행은 Python/C++과 비트 단위로 같지 않다 — 패리티 체커의
// 허용 오차(1e-9) 안에서 같다. 그래서 이 파일은 연산 순서를 그대로 미러한다: 스캔 순서,
// 셀 근접→원거리 보행(tie 는 Y, delta 0 축은 스텝 안 함), 도착 순서 누적, touched 집합을
// row-major 정렬로 방출(인덱스 r·W+c 오름차순 = (row,col) 사전식과 동일).
import {GridMap, inBounds, worldToCell} from "../grid"
import {poseCompose, robotToWorld} from "../geometry"
import {buildEpisode} from "../sim"
import {evaluate, EstimateResult} from "../metrics"
import {ScenarioJson} from "../trace/load"
import {Cell, ParamValue, Point, Pose, TraceEvent} from "../trace/types"

// from_pt → to_pt 닫힌 선분이 지나는 셀들을 근접→원거리 순으로 — 마지막이 착지 셀(hit).
// core/sim raycast 와 같은 DDA 보행 규칙을 세그먼트 벡터 위에 그대로 쓴다: tMax는 다음 격자선
// 교차의 ray 파라미터에서 시작해 res/|d| 만큼 전진하고, delta 가 0인 축은 절대 스텝하지 않으며,
// tie(tMaxX == tMaxY)는 Y 를 택한다. 보행이 래스터를 벗어나면 마지막 경계 내 셀이 hit 을 흡수하고,
// 시작 셀 자체가 경계 밖이면 아무것도 갱신하지 않는다.
export function cellsAlong(grid: GridMap, fromPt: Point, toPt: Point): Cell[] {
    const [x0, y0] = fromPt
    const dx = toPt[0] - x0
    const dy = toPt[1] - y0
    const h = grid.height // row 인덱스는 아래로 커진다 — height 만 tMax 에 들어간다
    let rc = worldToCell(grid, [x0, y0])
    if (!inBounds(grid, rc[0], rc[1])) return [] // 시작이 래스터 밖 — 바깥엔 셀이 없다
    const target = worldToCell(grid, [toPt[0], toPt[1]])
    const cells: Cell[] = [[rc[0], rc[1]]]
    if (rc[0] === target[0] && rc[1] === target[1]) return cells // 퇴화 세그먼트 — 시작이 곧 hit

    const [ox, oy] = grid.origin
    const res = grid.resolution
    let stepX: number, tMaxX: number
    if (dx > 0.0) {
        stepX = 1
        tMaxX = (ox + (rc[1] + 1) * res - x0) / dx
    } else if (dx < 0.0) {
        stepX = -1
        tMaxX = (ox + rc[1] * res - x0) / dx
    } else {
        stepX = 0
        tMaxX = Infinity
    }
    let stepY: number, tMaxY: number
    if (dy > 0.0) {
        stepY = -1 // row 인덱스는 아래로 커진다
        tMaxY = (oy + (h - rc[0]) * res - y0) / dy
    } else if (dy < 0.0) {
        stepY = 1
        tMaxY = (oy + (h - 1 - rc[0]) * res - y0) / dy
    } else {
        stepY = 0
        tMaxY = Infinity
    }
    const deltaX = dx !== 0.0 ? res / Math.abs(dx) : Infinity
    const deltaY = dy !== 0.0 ? res / Math.abs(dy) : Infinity

    for (;;) {
        if (tMaxX < tMaxY) {
            rc = [rc[0], rc[1] + stepX]
            tMaxX += deltaX
        } else {
            rc = [rc[0] + stepY, rc[1]]
            tMaxY += deltaY
        }
        if (!inBounds(grid, rc[0], rc[1])) return cells // 래스터를 벗어났다 — 마지막 셀이 hit 를 흡수
        cells.push([rc[0], rc[1]])
        if (rc[0] === target[0] && rc[1] === target[1]) return cells
    }
}

// 데모와 같은 계약: (맵, 시나리오, run_started의 파라미터 맵) → 완전한 이벤트 스트림.
// 이 알고리즘은 센서 파라미터를 선언하지 않는다 — 빔의 수는 시나리오가 정하고, 추정기는
// 그냥 그 점을 소비한다. 앵커(x0/y0/theta_deg)만이 유일한 노브(p_hit 와 함께)다.
export function runGridMapping(grid: GridMap, scenario: ScenarioJson,
                               params: Record<string, ParamValue>): TraceEvent[] {
    const pH = Number(params["p_hit"])
    const x0 = Number(params["x0"])
    const y0 = Number(params["y0"])
    const thetaDeg = Number(params["theta_deg"])

    // 대칭 역모델의 단일 log-odds 크기 — hit +L, 통과 −L.
    const logit = Math.log(pH / (1 - pH))

    const episode = buildEpisode(grid, scenario.path, scenario.step_meters, scenario.sensor, null,
        scenario.odom_noise.sigma_xy, scenario.odom_noise.sigma_theta, scenario.seed)
    const w = grid.width
    const field = new Float64Array(grid.height * grid.width) // prior 0.0 — 미지는 절대 점유가 아니다

    const poses: Pose[] = []
    const events: TraceEvent[] = []
    let seq = 0
    events.push({
        seq: seq++, event: "run_started", algorithm: "grid_mapping",
        params: {...params}, seed: scenario.seed, sensor: scenario.sensor,
    })

    for (const step of episode.steps) {
        events.push({seq: seq++, event: "step_observed", t: step.t, gt: step.gt,
                     ...(step.odom !== undefined ? {odom: step.odom} : {}),
                     ...(step.scan !== undefined ? {scan: step.scan} : {}),
                     ...(step.obs !== undefined ? {obs: step.obs} : {})})

        // 자세는 주어진 것 — t=0 에는 선언된 앵커가 곧 자세이고, 이후 무노이즈 오도메트리를 적분한다.
        let pose: Pose
        if (step.odom === undefined) {
            pose = [x0, y0, thetaDeg * Math.PI / 180]
        } else {
            pose = poseCompose(poses[poses.length - 1], step.odom)
        }
        poses.push(pose)

        if (step.scan !== undefined) {
            // 스캔 순서(beam 오름차순) — 고정 누적 순서. touched 는 인덱스 집합이고
            // 방출은 row-major 정렬(인덱스 오름차순)이다.
            const touched = new Set<number>()
            for (const z of step.scan) {
                const wPt = robotToWorld(z, pose)
                const cells = cellsAlong(grid, [pose[0], pose[1]], wPt)
                if (cells.length === 0) continue
                for (let i = 0; i + 1 < cells.length; i++) { // 통과 → 자유 증거
                    const idx = cells[i][0] * w + cells[i][1]
                    field[idx] -= logit
                    touched.add(idx)
                }
                const hitIdx = cells[cells.length - 1][0] * w + cells[cells.length - 1][1] // 착지 → 점유 증거
                field[hitIdx] += logit
                touched.add(hitIdx)
            }
            events.push({seq: seq++, event: "pose_estimated", t: step.t, pose})
            const out: Array<[number, number, number]> = []
            for (const idx of [...touched].sort((a, b) => a - b)) {
                out.push([Math.floor(idx / w), idx % w, field[idx]])
            }
            events.push({seq: seq++, event: "map_updated", t: step.t, cells: out})
        } else { // scan 없음 (beam 시나리오에선 불가능하지만 정직하게)
            events.push({seq: seq++, event: "pose_estimated", t: step.t, pose})
        }
    }

    const result: EstimateResult = {
        poses,
        grid: {resolution: grid.resolution, origin: [grid.origin[0], grid.origin[1]], logOdds: field},
    }
    events.push({seq, event: "run_finished", metrics: evaluate(result, episode)})
    return events
}
