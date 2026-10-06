import {Fragment, useMemo, useRef} from "react";
import {Circle, Group, Layer, Line, Shape, Stage} from "react-konva";
import Konva from "konva";
import {GridMap, worldToCellFloat} from "../../libs/grid";
import {
    EST_COLOR, GT_COLOR, HEAT_LOW, LANDMARK_COLOR, LOOP_COLOR, mixHex, PARTICLE_COLOR, SlamScene,
    withAlpha,
} from "../../libs/trace/timeline";
import {Point, Pose} from "../../libs/trace/types";
import {useCanvasColors} from "../../libs/useTheme";

// SLAM 씬 렌더러 — tools/viz/replay.py 의 draw() 미러. 표시 좌표는 셀 단위
// (u = (x-ox)/res, v = h-(y-oy)/res)이고 레이어 순서·색·투명도가 GIF와 같다:
// GT 격자 → belief/map 히트 → GT 트레일+로봇 → 스캔 팬(최신 스텝만) → 추정 트레일+
// 오차 타원 → 입자 구름(w·N 컬러맵) → 랜드마크 → 배치 궤적과 제약 간선.
export interface GridCanvasProps {
    map: GridMap;
    // 가장 긴 변의 픽셀 크기. 셀 크기는 여기서 유도된다.
    panel: number;
    scene?: SlamScene;
    // 이 seq 이하의 이벤트만 그린다 (누적 레이어). 생략이면 전부.
    cutoff?: number;
    // sandbox 상호작용 — 핸들러가 있을 때만 셀 페인팅이 활성화된다.
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
}

const sigmoid = (l: number): number => 1 / (1 + Math.exp(-l))

const GridCanvas = ({map, panel, scene, cutoff = Infinity, onPaintCell}: GridCanvasProps) => {
    const colors = useCanvasColors();
    const cell = panel / Math.max(map.width, map.height);
    const stageW = Math.round(cell * map.width);
    const stageH = Math.round(cell * map.height);

    // world 점 → 픽셀 (replay 의 disp 와 같은 변환).
    const toXY = (p: Point): [number, number] => {
        const [rf, cf] = worldToCellFloat(map, p)
        return [cf * cell, rf * cell]
    }

    // 점유 셀 — 맵이 바뀔 때만 재렌더된다.
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
        }} fill={colors.text} opacity={0.78}/>
    ), [map, cell, colors.text])

    // belief 히트맵: cutoff 이하 마지막 belief_updated 한 장 (전체 그리드 확률).
    const beliefSeq = scene ? scene.beliefs.filter((b) => b.seq <= cutoff).at(-1)?.seq : undefined
    const beliefLayer = useMemo(() => {
        if (!scene || beliefSeq === undefined) return null
        const frame = scene.beliefs.find((b) => b.seq === beliefSeq)!
        return (
            <Shape key={beliefSeq} listening={false} sceneFunc={(ctx) => {
                for (const [r, c, p] of frame.cells) {
                    ctx.fillStyle = withAlpha(
                        mixHex(HEAT_LOW, PARTICLE_COLOR, Math.min(1, Math.max(0, p))), 0.55)
                    ctx.beginPath()
                    ctx.rect(c * cell, r * cell, cell, cell)
                    ctx.fill()
                }
            }}/>
        )
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [beliefSeq, cell])

    // 로그오즈 지도: cutoff 이하 diff 를 누적해 sigmoid(l) 히트로 그린다 (매핑 갈래).
    const mapDiffSeq = scene ? scene.mapDiffs.filter((m) => m.seq <= cutoff).at(-1)?.seq : undefined
    const mapLayer = useMemo(() => {
        if (!scene || mapDiffSeq === undefined) return null
        const acc = new Map<number, number>()
        for (const diff of scene.mapDiffs) {
            if (diff.seq > mapDiffSeq) break
            for (const [r, c, l] of diff.cells) acc.set(r * map.width + c, l)
        }
        return (
            <Shape key={mapDiffSeq} listening={false} sceneFunc={(ctx) => {
                for (const [idx, l] of acc) {
                    const r = Math.floor(idx / map.width), c = idx % map.width
                    ctx.fillStyle = withAlpha(mixHex(HEAT_LOW, "#7c3aed", sigmoid(l)), 0.55)
                    ctx.beginPath()
                    ctx.rect(c * cell, r * cell, cell, cell)
                    ctx.fill()
                }
            }}/>
        )
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [mapDiffSeq, cell, map.width])

    // GT 트레일(점선) — 컷오프까지 누적.
    const gtVisible = scene ? scene.steps.filter((s) => s.seq <= cutoff) : []
    const gtTrail = useMemo(() => {
        if (!scene || gtVisible.length === 0) return null
        const pts: number[] = []
        for (const s of gtVisible) pts.push(...toXY([s.gt[0], s.gt[1]]))
        return <Line listening={false} points={pts} stroke={GT_COLOR} opacity={0.55}
                     strokeWidth={Math.max(1, cell * 0.08)} dash={[cell * 0.28, cell * 0.22]}
                     lineCap="round"/>
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [scene, gtVisible.length, cell])

    const lastStep = gtVisible.at(-1)
    const lastEst = scene ? scene.posesEst.filter((p) => p.seq <= cutoff).at(-1) : undefined

    // 스캔 팬/관측 점: 최신 가시 스텝만 (역사는 히트 레이어가 이미 담고 있다). 빔은
    // 로봇에서 점으로, 점은 작은 점으로 — replay 와 같은 색.
    const scanLayer = useMemo(() => {
        if (!lastStep || lastStep.points.length === 0) return null
        const [gx, gy] = toXY([lastStep.gt[0], lastStep.gt[1]])
        const pts = lastStep.points.map((p) => toXY(p))
        return (
            <Fragment>
                <Shape listening={false} stroke="#94a3b8" opacity={0.5}
                       strokeWidth={Math.max(0.4, cell * 0.035)} sceneFunc={(ctx, shape) => {
                           for (const [u, v] of pts) {
                               ctx.beginPath()
                               ctx.moveTo(gx, gy)
                               ctx.lineTo(u, v)
                               ctx.fillStrokeShape(shape)
                           }
                       }}/>
                <Shape listening={false} fill="#334155" sceneFunc={(ctx, shape) => {
                    for (const [u, v] of pts) {
                        ctx.beginPath()
                        ctx.arc(u, v, Math.max(1.2, cell * 0.07), 0, Math.PI * 2)
                        ctx.fillStrokeShape(shape)
                    }
                }}/>
            </Fragment>
        )
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [lastStep, cell])

    // 추정 트레일 — 필터 계열의 단계별 자세.
    const estVisible = scene ? scene.posesEst.filter((p) => p.seq <= cutoff) : []
    const estTrail = useMemo(() => {
        if (!scene || estVisible.length === 0) return null
        const pts: number[] = []
        for (const p of estVisible) pts.push(...toXY([p.pose[0], p.pose[1]]))
        return <Line listening={false} points={pts} stroke={EST_COLOR} opacity={0.95}
                     strokeWidth={Math.max(1.2, cell * 0.1)} lineCap="round"/>
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [scene, estVisible.length, cell])

    // 입자 구름(최신 가시 클라우드): 색은 w·N 컬러맵 — 지배적 입자는 짙고 죽은 입자는
    // HEAT_LOW 로 사라진다 (replay 의 Blues 매핑 미러).
    const cloudSeq = scene ? scene.clouds.filter((c) => c.seq <= cutoff).at(-1)?.seq : undefined
    const cloudLayer = useMemo(() => {
        if (!scene || cloudSeq === undefined) return null
        const cloud = scene.clouds.find((c) => c.seq === cloudSeq)!.particles
        const n = cloud.length || 1
        const dots = cloud.map(([x, y, , w]) => ({xy: toXY([x, y]), a: Math.min(1, w * n)}))
        return (
            <Shape key={cloudSeq} listening={false} sceneFunc={(ctx) => {
                for (const d of dots) {
                    ctx.fillStyle = withAlpha(mixHex(HEAT_LOW, PARTICLE_COLOR, d.a), 0.9)
                    ctx.beginPath()
                    ctx.arc(d.xy[0], d.xy[1], Math.max(1.4, cell * 0.09), 0, Math.PI * 2)
                    ctx.fill()
                }
            }}/>
        )
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [cloudSeq, cell])

    // 랜드마크: GT × 마크(항상 전부 — 시나리오 진실) + 최신 추정 +(σ 에러바).
    const lmFrame = scene ? scene.landmarkFrames.filter((f) => f.seq <= cutoff).at(-1) : undefined
    const landmarkLayer = useMemo(() => {
        if (!scene || !lmFrame) return null
        const gtMarks = scene.gtLandmarks.map((p) => toXY(p))
        const est = lmFrame.estimated.map((e) => ({
            xy: toXY([e.x, e.y]),
            sx: e.sx !== undefined ? (e.sx / map.resolution) * cell : undefined,
            sy: e.sy !== undefined ? (e.sy / map.resolution) * cell : undefined,
        }))
        return (
            <Fragment>
                {gtMarks.length > 0 && (
                    <Shape listening={false} stroke="#64748b" strokeWidth={Math.max(1.2, cell * 0.09)}
                           sceneFunc={(ctx, shape) => {
                               const s = Math.max(3, cell * 0.26)
                               for (const [u, v] of gtMarks) {
                                   ctx.beginPath()
                                   ctx.moveTo(u - s / 2, v - s / 2); ctx.lineTo(u + s / 2, v + s / 2)
                                   ctx.moveTo(u + s / 2, v - s / 2); ctx.lineTo(u - s / 2, v + s / 2)
                                   ctx.fillStrokeShape(shape)
                               }
                           }}/>
                )}
                {est.length > 0 && (
                    <Fragment>
                        <Shape listening={false} stroke={LANDMARK_COLOR} opacity={0.7}
                               strokeWidth={Math.max(0.8, cell * 0.05)} sceneFunc={(ctx, shape) => {
                                   for (const e of est) {
                                       if (e.sx === undefined || e.sy === undefined) continue
                                       ctx.beginPath()
                                       ctx.moveTo(e.xy[0] - e.sx, e.xy[1]); ctx.lineTo(e.xy[0] + e.sx, e.xy[1])
                                       ctx.moveTo(e.xy[0], e.xy[1] - e.sy); ctx.lineTo(e.xy[0], e.xy[1] + e.sy)
                                       ctx.fillStrokeShape(shape)
                                   }
                               }}/>
                        <Shape listening={false} stroke={LANDMARK_COLOR} strokeWidth={Math.max(1.4, cell * 0.1)}
                               sceneFunc={(ctx, shape) => {
                                   const s = Math.max(3, cell * 0.28)
                                   for (const e of est) {
                                       ctx.beginPath()
                                       ctx.moveTo(e.xy[0] - s / 2, e.xy[1]); ctx.lineTo(e.xy[0] + s / 2, e.xy[1])
                                       ctx.moveTo(e.xy[0], e.xy[1] - s / 2); ctx.lineTo(e.xy[0], e.xy[1] + s / 2)
                                       ctx.fillStrokeShape(shape)
                                   }
                               }}/>
                    </Fragment>
                )}
            </Fragment>
        )
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [scene, lmFrame?.seq, cell, map.resolution])

    // 배치 궤적(graph_based): 이벤트 seq 에 도달하면 전체 폴리라인이 나타나고, 제약 간선은
    // 양 끝 노드가 화면에 존재할 때만 그린다 (odometry 엣지는 옅게, loop 는 빨간 파선).
    const trajVisible = scene?.trajectory && scene.trajectory.seq <= cutoff ? scene.trajectory : null
    const graphLayer = useMemo(() => {
        if (!scene || !trajVisible) return null
        const poses = trajVisible.poses
        const linePts: number[] = []
        for (const p of poses) linePts.push(...toXY([p[0], p[1]]))
        const edges = scene.constraints
            .filter((c) => c.seq <= cutoff && c.i < poses.length && c.j < poses.length)
            .map((c, i) => ({
                key: i,
                pts: [...toXY([poses[c.i][0], poses[c.i][1]]), ...toXY([poses[c.j][0], poses[c.j][1]])],
                loop: c.loop,
            }))
        return (
            <Fragment>
                <Line listening={false} points={linePts} stroke={EST_COLOR} opacity={0.95}
                      strokeWidth={Math.max(1.2, cell * 0.1)} lineCap="round"/>
                {edges.map((e) => (
                    <Line key={`c${e.key}`} listening={false} points={e.pts}
                          stroke={e.loop === undefined ? "#94a3b8" : LOOP_COLOR}
                          opacity={e.loop === undefined ? 0.5 : 0.9}
                          strokeWidth={e.loop === undefined ? Math.max(0.6, cell * 0.05) : Math.max(1.2, cell * 0.09)}
                          dash={e.loop === undefined ? undefined : [cell * 0.3, cell * 0.24]}
                          lineCap="round"/>
                ))}
            </Fragment>
        )
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [scene, trajVisible?.seq, cutoff, cell])

    // 로봇 마커 — 점 + heading 화살표 (화살표 길이는 world 1 m).
    const robotMarker = (pose: Pose, color: string, key: string) => {
        const [x, y] = toXY([pose[0], pose[1]])
        const r = cell * 0.3
        const tip = toXY([pose[0] + Math.cos(pose[2]), pose[1] + Math.sin(pose[2])])
        return (
            <Group key={key} listening={false}>
                <Circle listening={false} x={x} y={y} radius={r} fill={color}
                        stroke={colors.bg} strokeWidth={Math.max(1, r * 0.25)}/>
                <Line listening={false} points={[x, y, tip[0], tip[1]]} stroke={color}
                      strokeWidth={Math.max(1, cell * 0.1)} lineCap="round"/>
            </Group>
        )
    }

    // 벽 페인팅 (sandbox): 첫 셀의 반전값을 붓 값으로 드래그 내내 유지한다.
    const paintValue = useRef<boolean | null>(null)
    const cellAt = (stage: Konva.Stage | null): [number, number] | null => {
        const pos = stage?.getPointerPosition()
        if (!pos) return null
        const c = Math.floor(pos.x / cell)
        const r = Math.floor(pos.y / cell)
        if (r < 0 || r >= map.height || c < 0 || c >= map.width) return null
        return [r, c]
    }

    return (
        <Stage width={stageW} height={stageH}
               className="bg-surface border border-border rounded-lg overflow-hidden w-fit"
               onPointerDown={(e) => {
                   if (!onPaintCell) return
                   const c = cellAt(e.target.getStage())
                   if (!c) return
                   paintValue.current = !map.occupied[c[0] * map.width + c[1]]
                   onPaintCell(c[0], c[1], paintValue.current)
               }}
               onPointerMove={(e) => {
                   if (!onPaintCell || paintValue.current === null) return
                   const c = cellAt(e.target.getStage())
                   if (c) onPaintCell(c[0], c[1], paintValue.current)
               }}
               onPointerUp={() => { paintValue.current = null }}
               onPointerLeave={() => { paintValue.current = null }}>
            <Layer>
                {wallLayer}
                {beliefLayer}
                {mapLayer}
                {scanLayer}
                {gtTrail}
                {estTrail}
                {cloudLayer}
                {landmarkLayer}
                {graphLayer}
                {lastStep && robotMarker(lastStep.gt, GT_COLOR, "gt")}
                {lastEst && robotMarker([lastEst.pose[0], lastEst.pose[1], lastEst.pose[2]], EST_COLOR, "est")}
            </Layer>
        </Stage>
    )
}

export default GridCanvas
