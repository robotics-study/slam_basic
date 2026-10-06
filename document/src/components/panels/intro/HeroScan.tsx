import {useEffect, useMemo, useRef, useState} from "react";
import {Circle, Group, Layer, Line, Shape, Stage} from "react-konva";
import {GridMap} from "../../../libs/grid";
import {raycast} from "../../../libs/sim";
import {Point, Pose} from "../../../libs/trace/types";
import {useCanvasColors} from "../../../libs/useTheme";

// 히어로의 라이브 피겨 — 알고리즘 없이도 "SLAM이 무엇인가"를 보여 준다: 작은 격자 지도를
// 배경에 깔고, 로봇이 웨이포인트 경로를 따라 움직이며 각 스텝의 빔 스캔을 쏜다. 맞은 점이
// 쌓여 벽의 윤곽이 드러나는 순간 — 추정의 출발점 그 자체. (노이즈 없는 순수 레이캐스트;
// 알고리즘 데모는 각 알고리즘 페이지의 라이브 sandbox 가 한다.)
const MAP_ROWS = [
    "########################",
    "#......................#",
    "#..####....######......#",
    "#..####....######..##..#",
    "#..............##...#..#",
    "#..#####.......##...#..#",
    "#..#...............#...#",
    "#..#....#####......#...#",
    "#.................##...#",
    "########################",
]
const RES = 0.5

// 로봇이 걸어 다닐 웨이포인트 (셀 좌표가 아니라 world 미터 — res 로 환산해 잡는다).
const WAYPOINTS: Point[] = [
    [2 * RES + RES / 2, 6 * RES],
    [9 * RES, 1.5 * RES],
    [17 * RES, 3 * RES],
    [18 * RES, 7 * RES],
    [4 * RES, 7.5 * RES],
]

const BEAMS = 60
const FOV = Math.PI // 180도
const RANGE_MAX = 9

function buildMap(): GridMap {
    const height = MAP_ROWS.length
    const width = MAP_ROWS[0].length
    const occupied: boolean[] = new Array(width * height).fill(false)
    for (let r = 0; r < height; r++) {
        for (let c = 0; c < width; c++) occupied[r * width + c] = MAP_ROWS[r][c] === "#"
    }
    return {name: "hero", width, height, resolution: RES, origin: [0, 0], occupied}
}

// 노이즈 없는 스캔 — sim.raycast 와 동일한 DDA (sigma 0 이면 그대로의 결과).
function scanAt(map: GridMap, pose: Pose): Point[] {
    const pts: Point[] = []
    const half = FOV / 2
    for (let i = 0; i < BEAMS; i++) {
        const phi = pose[2] - half + (FOV / (BEAMS - 1)) * i
        const hit = raycast(map, pose[0], pose[1], phi, RANGE_MAX)
        if (hit === null) continue
        pts.push([pose[0] + Math.cos(phi) * hit, pose[1] + Math.sin(phi) * hit])
    }
    return pts
}

// 등아크 재샘플 (sim.resample 과 같은 규칙의 단순 버전 — 히어로 전용).
function resampleHero(path: Point[], stepMeters: number): Pose[] {
    const segLen: number[] = []
    let total = 0.0
    for (let k = 0; k < path.length - 1; k++) {
        const l = Math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
        segLen.push(l)
        total += l
    }
    const pts: Point[] = []
    for (let s = 0.0; s < total; s += stepMeters) {
        let acc = 0.0, chosen: Point = path[path.length - 1]
        for (let k = 0; k < segLen.length; k++) {
            if (s - acc <= segLen[k]) {
                const frac = (s - acc) / segLen[k]
                chosen = [
                    path[k][0] + frac * (path[k + 1][0] - path[k][0]),
                    path[k][1] + frac * (path[k + 1][1] - path[k][1]),
                ]
                break
            }
            acc += segLen[k]
        }
        pts.push(chosen)
    }
    const poses: Pose[] = []
    for (let k = 0; k < pts.length; k++) {
        const theta = k + 1 < pts.length
            ? Math.atan2(pts[k + 1][1] - pts[k][1], pts[k + 1][0] - pts[k][0])
            : poses[poses.length - 1][2]
        poses.push([pts[k][0], pts[k][1], theta])
    }
    return poses
}

const HeroScan = () => {
    const colors = useCanvasColors()
    const map = useMemo(buildMap, [])
    const cell = 18
    const stageW = Math.round(cell * map.width)
    const stageH = Math.round(cell * map.height)

    const poses = useMemo(() => resampleHero(WAYPOINTS, 0.35), [])
    // 각 스텝의 스캔을 미리 계산해 두고 (노이즈 없음 — 결정적), 프레임마다 하나씩 드러낸다.
    const scans = useMemo(() => poses.map((p) => scanAt(map, p)), [poses, map])

    const [step, setStep] = useState(0)
    const playing = useRef(true)
    useEffect(() => {
        const timer = window.setInterval(() => {
            setStep((s) => (playing.current ? (s + 1) % (poses.length + 24) : s))
        }, 90)
        return () => window.clearInterval(timer)
    }, [poses.length])

    const toXY = (p: Point): [number, number] =>
        [(p[0] - map.origin[0]) / map.resolution * cell,
         (map.height - (p[1] - map.origin[1]) / map.resolution) * cell]

    const wallLayer = useMemo(() => (
        <Shape listening={false} sceneFunc={(ctx, shape) => {
            for (let r = 0; r < map.height; r++) {
                for (let c = 0; c < map.width; c++) {
                    if (!map.occupied[r * map.width + c]) continue
                    ctx.beginPath()
                    ctx.rect(c * cell, r * cell, cell, cell)
                    ctx.fillStrokeShape(shape)
                }
            }
        }} fill={colors.text} opacity={0.14}/>
    ), [map, cell, colors.text])

    const cur = poses[Math.min(step, poses.length - 1)]
    const hitPts: number[] = []
    for (let i = 0; i <= Math.min(step, scans.length - 1); i++) {
        for (const p of scans[i]) hitPts.push(...toXY(p))
    }

    return (
        <Stage width={stageW} height={stageH} className="rounded-lg overflow-hidden w-fit"
               onMouseEnter={() => { playing.current = false }}
               onMouseLeave={() => { playing.current = true }}>
            <Layer>
                {wallLayer}
                {/* 쌓여 가는 스캔 점 — 벽의 윤곽이 드러난다 */}
                <Shape listening={false} sceneFunc={(ctx, shape) => {
                    for (let i = 0; i + 1 < hitPts.length; i += 2) {
                        ctx.beginPath()
                        ctx.arc(hitPts[i], hitPts[i + 1], Math.max(1.4, cell * 0.1), 0, Math.PI * 2)
                        ctx.fillStrokeShape(shape)
                    }
                }} fill={colors.muted} opacity={0.8}/>
                {/* 현재 스캔 팬 — 로봇에서 맞은 점으로 */}
                {step < scans.length && (() => {
                    const [gx, gy] = toXY([cur[0], cur[1]])
                    const rays = scans[Math.min(step, scans.length - 1)].map((p) => ({g: [gx, gy] as number[], p: toXY(p)}))
                    return (
                        <Shape listening={false} stroke={colors.accent2} opacity={0.35} strokeWidth={0.6}
                               sceneFunc={(ctx, shape) => {
                                   for (const r of rays) {
                                       ctx.beginPath()
                                       ctx.moveTo(r.g[0], r.g[1])
                                       ctx.lineTo(r.p[0], r.p[1])
                                       ctx.fillStrokeShape(shape)
                                   }
                               }}/>
                    )
                })()}
                {/* GT 경로 (옅은 점선) + 로봇 */}
                <Line listening={false} points={poses.flatMap((p) => toXY([p[0], p[1]]))}
                      stroke={colors.muted} opacity={0.35} strokeWidth={1} dash={[4, 4]}/>
                {(() => {
                    const [x, y] = toXY([cur[0], cur[1]])
                    const tip = toXY([cur[0] + Math.cos(cur[2]), cur[1] + Math.sin(cur[2])])
                    return (
                        <Group listening={false}>
                            <Circle listening={false} x={x} y={y} radius={cell * 0.34} fill={colors.accent}
                                    stroke={colors.bg} strokeWidth={1.5}/>
                            <Line listening={false} points={[x, y, tip[0], tip[1]]} stroke={colors.accent}
                                  strokeWidth={1.6} lineCap="round"/>
                        </Group>
                    )
                })()}
            </Layer>
        </Stage>
    )
}

export default HeroScan
