import {ReactNode, useEffect, useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../CanvasFigure";
import TracePlayer from "../player/TracePlayer";
import {loadScenario, LoadedScenario, ScenarioJson} from "../../libs/trace/load";
import {buildScene} from "../../libs/trace/timeline";
import {GridMap} from "../../libs/grid";
import {ParamValue, TraceEvent} from "../../libs/trace/types";
import {useTr} from "../../libs/i18n";

// 라이브 sandbox — 페이지의 알고리즘을 브라우저에서 직접 돌린다. 저장소의 Python/C++
// 구현과 필드 단위로 동일한 미러 엔진(libs/algorithms)이 그대로 실행되고, recorded trace
// 재생은 어디에도 없다. 셀을 드래그해 벽을 그리면 스캔이 바뀌고 재실행이 즉시 돈다.
// 파라미터 칩은 configs/<section>/<algo>.yaml 의 선언과 같은 키를 같은 값 사다리에서 고른다.

export interface ScenarioPreset {
    // maps/scenarios/<slug>.yaml 슬러그 (버튼 라벨과 동일).
    name: string;
}

export interface ParamChip {
    key: string;
    label?: string;
    values: Array<number | boolean | string>;
}

// 데모 드라이버와 같은 주입 계약 (demos/demo_common.py): 선언된 센서 파라미터는
// 시나리오의 센서 블록에서, seed 는 시나리오의 seed 로 채워진다 — 기본값은 자리채우기일
// 뿐이고, 없는 필드를 선언한 것은 조용한 기본값이 아니라 오류다. (seed 는 확률적 알고리즘만 선언한다.)
const SENSOR_KEYS = ["beams", "fov_deg", "range_max", "sigma_range", "sigma_bearing"] as const

function injectSensor(defaults: Record<string, ParamValue>, scenario: ScenarioJson): Record<string, ParamValue> {
    const out = {...defaults}
    if ("seed" in defaults) out["seed"] = scenario.seed
    for (const key of SENSOR_KEYS) {
        if (key in defaults) {
            const v = scenario.sensor[key]
            if (v === undefined) throw new Error(`scenario sensor lacks "${key}" declared by the algorithm`)
            out[key] = v
        }
    }
    return out
}

export interface SandboxProps {
    presets: ScenarioPreset[];
    // 라이브 엔진 — (맵, 시나리오, 파라미터) → trace 이벤트. 알고리즘 페이지가 모듈 상수로 넘긴다.
    run: (grid: GridMap, scenario: ScenarioJson, params: Record<string, ParamValue>) => TraceEvent[];
    // 데모 기본 파라미터 (configs yaml 의 기본값과 동일).
    params: Record<string, ParamValue>;
    // 클릭마다 사다리를 순환하는 파라미터 칩 — 없는 알고리즘은 칩 없이 프리셋만.
    chips?: ParamChip[];
    label: string;
}

export const SandboxScene = ({presets, run, params: defaultParams, chips = [], label}: SandboxProps) => {
    const t = useTr()
    const [presetName, setPresetName] = useState(presets[0].name)
    const [scenario, setScenario] = useState<ScenarioJson | null>(null)
    // 편집된 맵 (벽 페인팅) — 프리셋 전환 시 시나리오와 함께 되돌아온다.
    const [grid, setGrid] = useState<GridMap | null>(null)
    const [params, setParams] = useState<Record<string, ParamValue>>(defaultParams)
    const [error, setError] = useState<string | null>(null)
    // 리셋은 presetName 이 이미 같아도 로드를 다시 촉발해야 하므로 별도 nonce 를 돈다.
    const [nonce, setNonce] = useState(0)

    useEffect(() => {
        let cancelled = false
        const name = presets.find((p) => p.name === presetName)?.name ?? presets[0].name
        loadScenario(name).then((l: LoadedScenario) => {
            if (cancelled) return
            // 편집 대상 사본 (occupied 복사). 파라미터는 주입 계약대로 재구성 —
            // 프리셋마다 센서 기본값이 다르다 (corridor02 의 range_max 2.5 등).
            setGrid({...l.grid, occupied: [...l.grid.occupied]})
            setScenario(l.scenario)
            setParams(injectSensor(defaultParams, l.scenario))
            setError(null)
        }).catch((e: unknown) => {
            if (!cancelled) setError(e instanceof Error ? e.message : String(e))
        })
        return () => {
            cancelled = true
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [presetName, nonce])

    // 라이브: 맵/시나리오/파라미터가 바뀔 때마다 엔진을 다시 돌리고 재생이 0부터 돈다.
    // 엔진이 던지는 거부(센서 타입 불일치 등)는 크래시가 아니라 error 카드다.
    const scene = useMemo(() => {
        if (!scenario || !grid) return null
        try {
            return buildScene(run(grid, scenario, params))
        } catch (e) {
            setError(e instanceof Error ? e.message : String(e))
            return null
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [scenario, grid, params])

    // 벽 페인팅 — 셀을 드래그하면 격자가 바뀌고 scene 이 다시 돈다.
    const paintCell = (row: number, col: number, occupied: boolean) => {
        setGrid((prev) => {
            if (!prev) return prev
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }

    // 파라미터 칩 클릭 — 사다리를 한 칸 순환.
    const cycleParam = (chip: ParamChip) => setParams((prev) => {
        const cur = prev[chip.key]
        const i = chip.values.findIndex((v) => v === cur)
        const next = chip.values[(i + 1) % chip.values.length]
        return {...prev, [chip.key]: next}
    })

    const controls: ReactNode = (
        <div className="flex flex-col items-center gap-1.5 text-xs text-muted">
            <div className="flex items-center justify-center gap-1.5 flex-wrap">
                {presets.map((p) => (
                    <button key={p.name} type="button" onClick={() => setPresetName(p.name)}
                            className="px-2 py-0.5 rounded border font-mono"
                            style={p.name === presetName
                                ? {borderColor: "var(--accent)", color: "var(--accent)", fontWeight: 600}
                                : undefined}>
                        {p.name}
                    </button>
                ))}
            </div>
            {chips.length > 0 && (
                <div className="flex items-center justify-center gap-1.5 flex-wrap">
                    {chips.map((chip) => (
                        <button key={chip.key} type="button" onClick={() => cycleParam(chip)}
                                aria-label={t(`cycle ${chip.key}`, `${chip.key} 순환 변경`)}
                                className="px-1.5 py-0.5 rounded border border-border font-mono tabular-nums hover:bg-surface">
                            {`${chip.label ?? chip.key}=${String(params[chip.key])}`}
                        </button>
                    ))}
                </div>
            )}
            <div className="flex items-center justify-center gap-1.5">
                <button type="button" onClick={() => setNonce((n) => n + 1)}
                        aria-label={t("reset the sandbox", "sandbox 초기화")}
                        className="px-2 py-0.5 rounded border border-border hover:bg-surface inline-flex items-center gap-1">
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor"
                         strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                        <path d="M3 12a9 9 0 1 0 3-6.7"/>
                        <path d="M3 4v5h5"/>
                    </svg>
                </button>
            </div>
            <div className="text-xs text-muted text-center">
                {t("drag cells to draw walls — the scan changes and the estimator re-runs from step 0",
                   "셀을 드래그해 벽을 그리면 스캔이 바뀌고 추정이 스텝 0부터 다시 돌아갑니다")}
            </div>
        </div>
    )

    if (error) {
        return (
            <div className="rounded-xl border border-border bg-surface p-4 text-sm text-muted text-center">
                {t("engine refused this run", "엔진이 이 실행을 거부했습니다")}:{" "}
                <span className="font-mono">{error}</span>
            </div>
        )
    }

    return (
        <CanvasFigure label={label} tight modal={scene && grid ? (
            <TracePlayer map={grid} scene={scene} panel={modalCanvasSize(1).width}/>
        ) : undefined}>
            {scene && grid
                ? <TracePlayer map={grid} scene={scene} footer={controls} onPaintCell={paintCell}/>
                : <div style={{minHeight: 340}} className="grid place-items-center text-sm text-muted">…</div>}
        </CanvasFigure>
    )
}
