// python/slam/core/metrics.py의 미러 — 언어 공용 지표 계약. 데모가 run_finished로 실어
// 보내는 값들의 정의이며, 라이브 엔진이 같은 키·같은 순서(사전순 아님 — 계산 순서)로
// 지표를 채우는 근거다. 라이브 실행은 JS 산술이라 ulp 수준 차이는 허용 오차 계약(패리티
// 체커의 tol) 안에 있다; 정수 지표(IoU 카운트)는 여전히 사실상 exact하다.
import {GridMap} from "./grid";
import {poseMinus} from "./geometry";
import {Episode} from "./sim";
import {Point, Pose} from "./trace/types";

// 추정기가 뱉은 결과 — poses는 스텝별(인덱스 = t), landmarks는 선택, grid는 매핑 계열이
// 시나리오 래스터 위에 직접 짓는 log-odds 맵(같은 resolution/origin 계약).
export interface EstimateResult {
    poses: Pose[];
    landmarks?: Array<{ id: number; x: number; y: number; sx?: number; sy?: number }>;
    grid?: { resolution: number; origin: [number, number]; logOdds: Float64Array | number[] };
}

// ate_rmse — 전 스텝 위치 오차의 RMS (미터).
export function ateRmse(poses: Pose[], gts: Pose[]): number {
    if (poses.length !== gts.length) throw new Error("ATE needs one estimate per ground-truth step")
    let total = 0.0
    for (let t = 0; t < poses.length; t++) {
        const dx = poses[t][0] - gts[t][0]
        const dy = poses[t][1] - gts[t][1]
        total += dx * dx + dy * dy
    }
    return Math.sqrt(total / poses.length)
}

// rpe_rmse — 연속 스텝의 상대 자세 오차. 고정 규범: 노름은 sqrt(dx²+dy²+dθ²) — 미터와
// 라디안이 하나의 노름을 공유하는 것은 계약이지 물리가 아니다.
export function rpeRmse(poses: Pose[], gts: Pose[]): number {
    if (poses.length !== gts.length) throw new Error("RPE needs one estimate per ground-truth step")
    if (poses.length < 2) return 0.0
    let total = 0.0
    for (let t = 1; t < poses.length; t++) {
        const eHat = poseMinus(poses[t - 1], poses[t])
        const eGt = poseMinus(gts[t - 1], gts[t])
        const dx = eHat[0] - eGt[0]
        const dy = eHat[1] - eGt[1]
        const dt = eHat[2] - eGt[2]
        total += dx * dx + dy * dy + dt * dt
    }
    return Math.sqrt(total / (poses.length - 1))
}

// map_iou — 추정 log-odds 맵(l > 0이 점유; 미지는 절대 점유로 취급)과 GT 격자의 셀 IoU.
// 양쪽 모두 비어 있으면 1.0으로 정의한다.
export function mapIou(est: EstimateResult["grid"], gt: GridMap): number {
    if (!est) throw new Error("map_iou needs an estimated grid")
    if (est.resolution !== gt.resolution || est.origin[0] !== gt.origin[0] || est.origin[1] !== gt.origin[1]) {
        throw new Error("map_iou: estimate grid must share the ground-truth raster")
    }
    const n = gt.width * gt.height
    if (est.logOdds.length !== n) throw new Error("map_iou: estimate grid must share the ground-truth raster")
    let inter = 0, union = 0
    for (let i = 0; i < n; i++) {
        const estOcc = est.logOdds[i] > 0.0
        const gtOcc = gt.occupied[i]
        if (estOcc && gtOcc) inter++
        if (estOcc || gtOcc) union++
    }
    if (union === 0) return 1.0
    return inter / union
}

// landmark_rmse — 양쪽에 모두 있는 id에 대한 위치 RMSE; 하나도 없으면 생략.
export function landmarkRmse(estimated: Array<{ id: number; x: number; y: number }>,
                             gts: Point[]): number | null {
    let total = 0.0, n = 0
    for (const e of estimated) {
        if (e.id < gts.length) {
            const dx = e.x - gts[e.id][0]
            const dy = e.y - gts[e.id][1]
            total += dx * dx + dy * dy
            n++
        }
    }
    if (n === 0) return null
    return Math.sqrt(total / n)
}

// 한 실행의 지표 묶음 — python evaluate()와 같은 키, 같은 삽입 순서.
export function evaluate(result: EstimateResult, episode: Episode): Record<string, number> {
    const gts = episode.steps.map((s) => s.gt)
    const metrics: Record<string, number> = {
        ate_rmse: ateRmse(result.poses, gts),
        rpe_rmse: rpeRmse(result.poses, gts),
    }
    if (result.grid) metrics["map_iou"] = mapIou(result.grid, episode.grid)
    if (result.landmarks && episode.landmarks) {
        const lm = landmarkRmse(result.landmarks, episode.landmarks)
        if (lm !== null) metrics["landmark_rmse"] = lm
    }
    return metrics
}
