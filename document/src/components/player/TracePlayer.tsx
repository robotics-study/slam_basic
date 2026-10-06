import {ReactNode, useEffect, useMemo, useState} from "react";
import {GridMap} from "../../libs/grid";
import {SlamScene} from "../../libs/trace/timeline";
import GridCanvas from "../2d/GridCanvas";
import {useTr} from "../../libs/i18n";

// SLAM 재생은 단일 위상이다 — 이벤트 스트림이 곧 애니메이션. tick 은 "보이는 이벤트 수"
// (컷오프)이고, 자동 재생은 이벤트 전체를 고정 시간(PLAY_MS)에 압축해 같은 체감 속도를
// 만든다. ⏮/⏭ 은 정지 상태에서 이벤트 하나씩 밟는다 — 관측이 들어오고 추정이 갱신되는
// 순서를 직접 관찰하는 것이 이 사이트의 학습 단위다.
const PLAY_MS = 3000;
const TICK_MS = 30;

interface TracePlayerProps {
    map: GridMap;
    scene: SlamScene;
    autoPlay?: boolean;
    panel?: number;
    // 플레이어 아래 추가 컨트롤 (sandbox의 프리셋/파라미터 칩 등).
    footer?: ReactNode;
    // 셀 페인팅 핸들러 — 있을 때만 격자 편집이 활성화된다 (sandbox 의 벽 그리기).
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
}

const Btn = ({onClick, label, children}: {
    onClick: () => void; label: string; children: ReactNode
}) => (
    <button type="button" onClick={onClick} aria-label={label}
            className="px-1.5 py-1 rounded border border-border hover:bg-surface leading-none">
        {children}
    </button>
)

const TracePlayer = ({map, scene, autoPlay = true, panel = 340, footer, onPaintCell}: TracePlayerProps) => {
    const t = useTr()
    const total = scene.total
    const [tick, setTick] = useState(autoPlay ? 0 : total)
    const [playing, setPlaying] = useState(autoPlay)
    const finished = tick >= total

    // 이벤트 스트림 전체를 고정 창에 압축 — 이벤트 수와 무관하게 체감 속도 동일.
    useEffect(() => {
        if (!playing || finished) return
        const perTick = Math.max(1, Math.round(total / (PLAY_MS / TICK_MS)))
        const timer = window.setInterval(() => {
            setTick((v) => Math.min(total, v + perTick))
        }, TICK_MS)
        return () => window.clearInterval(timer)
    }, [playing, finished, total])

    // 스트림이 끝나면 자동 재생을 멈춘다.
    useEffect(() => {
        if (playing && tick >= total) setPlaying(false)
    }, [playing, tick, total])

    const replay = () => {
        setTick(0)
        setPlaying(true)
    }
    const stepBy = (d: number) => {
        setPlaying(false)
        setTick((v) => Math.max(0, Math.min(total, v + d)))
    }

    // 상태 라인: 마지막 가시 스텝의 t 와 (run_finished 가 보이면) 지표.
    const currentT = useMemo(() => {
        let last: number | null = null
        for (const s of scene.steps) if (s.seq <= tick) last = s.t
        return last
    }, [scene, tick])
    const metricsShown = scene.metrics !== undefined && scene.total > 0 && tick >= total

    return (
        <div className="flex flex-col gap-2 items-center">
            <GridCanvas map={map} panel={panel} scene={scene} cutoff={tick} onPaintCell={onPaintCell}/>

            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: panel}}>
                <Btn onClick={() => stepBy(-1)} label={t("step back one event", "이벤트 하나 뒤로")}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                        <path d="M7 5h2v14H7zM19 5v14l-8-7 8-7z"/>
                    </svg>
                </Btn>
                {playing
                    ? <Btn onClick={() => setPlaying(false)} label={t("pause", "일시정지")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M7 5h4v14H7zM13 5h4v14h-4z"/>
                        </svg>
                    </Btn>
                    : <Btn onClick={finished ? replay : () => setPlaying(true)} label={t("play", "재생")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M8 5v14l11-7z"/>
                        </svg>
                    </Btn>}
                <Btn onClick={() => stepBy(1)} label={t("step forward one event", "이벤트 하나 앞으로")}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                        <path d="M5 5l8 7-8 7V5zM15 5h2v14h-2z"/>
                    </svg>
                </Btn>
                <input type="range" min={0} max={total} value={Math.min(tick, total)}
                       onChange={(e) => {
                           setPlaying(false)
                           setTick(parseInt(e.target.value))
                       }}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("timeline — scrub the event stream", "타임라인 — 이벤트 스트림을 스크럽")}/>
            </div>

            <div className="text-xs text-muted text-center tabular-nums">
                {t("step", "스텝")}{" "}
                <span className="font-semibold" style={{color: "var(--accent)"}}>{currentT ?? "–"}</span>
                {" · "}{t("events", "이벤트")}{" "}
                <span className="font-semibold">{Math.min(tick, total)}/{total}</span>
                {metricsShown && (
                    <>
                        {" · "}
                        {Object.entries(scene.metrics!).map(([k, v]) => (
                            <span key={k} className="ml-1 font-mono">
                                {k} <span className="font-semibold">{v.toFixed(4)}</span>
                            </span>
                        ))}
                    </>
                )}
            </div>

            {footer}
        </div>
    )
}

export default TracePlayer
