// trace 이벤트 열을 재생 가능한 씬으로 접는다 — tools/viz/replay.py의 build_scene 미러.
// 렌더러는 "seq 이하"의 이벤트만 그린다(누적 레이어). 스캔 점은 GT 자세로 world 복원,
// 랜드마크 관측은 (range, bearing)에서 world 점으로 복원해 그린다. 색 팔레트는 replay.py와
// 동일해야 GIF와 브라우저 재생이 같은 실행으로 읽힌다.
import {robotToWorld} from "../geometry";
import {Point, Pose, TraceEvent, Twist} from "./types";

// replay.py의 고정 팔레트 (테마와 무관 — GIF와 동일한 색이 계약).
export const GT_COLOR = "#0f172a"        // GT 트레일/로봇
export const EST_COLOR = "#0d9488"       // 추정 트레일/로봇
export const PARTICLE_COLOR = "#2563eb"  // 입자 히트맵 상한 (Blues)
export const LANDMARK_COLOR = "#ca8a04"  // 랜드마크 추정 +
export const LOOP_COLOR = "#dc2626"      // 루프 클로저 코드

// belief/map 히트의 하한 색 (replay의 colormap 시작값).
export const HEAT_LOW = "#f8fafc"

export interface StepFrame {
    seq: number;
    t: number;
    gt: Pose;
    // 스캔/랜드마크 관측을 그 스텝의 GT 자세로 world 복원한 점들.
    points: Point[];
}

export interface SlamScene {
    algorithm: string;
    gtLandmarks: Point[];
    steps: StepFrame[];
    posesEst: Array<{ seq: number; pose: Pose; cov?: [number, number, number] }>
    clouds: Array<{ seq: number; particles: Array<[number, number, number, number]> }>
    beliefs: Array<{ seq: number; cells: Array<[number, number, number]> }>
    landmarkFrames: Array<{ seq: number; estimated: Array<{ id: number; x: number; y: number; sx?: number; sy?: number }> }>
    mapDiffs: Array<{ seq: number; cells: Array<[number, number, number]> }>
    constraints: Array<{ seq: number; i: number; j: number; d: Twist; loop?: boolean }>
    trajectory: { poses: Pose[]; seq: number } | null;
    metrics?: Record<string, number>;
    // 이벤트 수 (= 컷오프 상한). 재생 슬라이더의 최대값.
    total: number;
}

// 스캔 점은 이미 로봇 프레임 — GT 자세로 world 복원만 한다. 랜드마크 관측은 극좌표를
// 직각으로 푸른다 (replay._restore_obs와 같은 식).
function restoreObs(gt: Pose, obs: Array<{ id: number; bearing: number; range: number }>): Point[] {
    return obs.map((o) => [gt[0] + o.range * Math.cos(gt[2] + o.bearing), gt[1] + o.range * Math.sin(gt[2] + o.bearing)])
}

export function buildScene(events: TraceEvent[]): SlamScene {
    const scene: SlamScene = {
        algorithm: "",
        gtLandmarks: [],
        steps: [],
        posesEst: [],
        clouds: [],
        beliefs: [],
        landmarkFrames: [],
        mapDiffs: [],
        constraints: [],
        trajectory: null,
        total: events.length,
    }
    for (const ev of events) {
        const seq = ev.seq
        switch (ev.event) {
            case "run_started":
                scene.algorithm = ev.algorithm ?? ""
                scene.gtLandmarks = ev.landmarks ?? []
                break
            case "step_observed": {
                const gt = ev.gt as Pose
                let points: Point[] = []
                if (ev.scan) {
                    // scan 은 로봇 프레임 — 계약의 역변환 robotToWorld 로 world 복원.
                    points = ev.scan.map((p) => robotToWorld(p, gt))
                } else if (ev.obs) {
                    points = restoreObs(gt, ev.obs)
                }
                scene.steps.push({seq, t: ev.t ?? 0, gt, points})
                break
            }
            case "pose_estimated":
                scene.posesEst.push({seq, pose: ev.pose as Pose, cov: ev.cov})
                break
            case "particles_updated":
                scene.clouds.push({seq, particles: ev.particles ?? []})
                break
            case "belief_updated":
                scene.beliefs.push({seq, cells: ev.cells ?? []})
                break
            case "landmarks_updated":
                scene.landmarkFrames.push({seq, estimated: ev.estimated ?? []})
                break
            case "map_updated":
                scene.mapDiffs.push({seq, cells: ev.cells ?? []})
                break
            case "constraint_added":
                scene.constraints.push({
                    seq, i: ev.i as number, j: ev.j as number, d: ev.d as Twist, loop: ev.loop,
                })
                break
            case "trajectory_found":
                scene.trajectory = {poses: ev.poses ?? [], seq}
                break
            case "run_finished":
                scene.metrics = ev.metrics
                break
        }
    }
    return scene
}

// hex (#rrggbb) 두 색의 선형 보간 — 히트맵/입자 색을 replay.py 팔레트에 맞춘다.
export function mixHex(a: string, b: string, t: number): string {
    const ch = (s: string, i: number) => parseInt(s.slice(i, i + 2), 16)
    const r = Math.round(ch(a, 1) + (ch(b, 1) - ch(a, 1)) * t)
    const g = Math.round(ch(a, 3) + (ch(b, 3) - ch(a, 3)) * t)
    const bl = Math.round(ch(a, 5) + (ch(b, 5) - ch(a, 5)) * t)
    return `rgb(${r},${g},${bl})`
}

// 색(#rrggbb 또는 rgb())에 알파를 섞는다. 히트맵/입자 레이어는 Shape 의 fill 속성 없이
// 셀마다 색을 바꿔 그린다 — Konva 는 shape 에 fill 속성이 없으면 fillStrokeShape 이
// 아무것도 채우지 않으므로, 투명도를 색 문자열로 직접 구워 ctx.fill() 로 채운다.
export function withAlpha(color: string, a: number): string {
    if (color.startsWith("#")) {
        const ch = (i: number) => parseInt(color.slice(i, i + 2), 16)
        return `rgba(${ch(1)},${ch(3)},${ch(5)},${a})`
    }
    return color.replace("rgb(", "rgba(").replace(")", `, ${a})`)
}
