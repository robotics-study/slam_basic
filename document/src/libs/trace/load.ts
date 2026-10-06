// 웹 패널이 쓰는 로더 — tools/web_export가 수출한 JSON(data/maps, data/scenarios)을
// 라이브 sandbox와 parity 체크어가 같은 입력으로 읽는다. 알고리즘 실행 자체는 브라우저
// 엔진(libs/algorithms)이 하고, recorded trace는 parity 검증의 기준 자료로만 쓰인다.
import {GridMap, GridMapJson, parseGridMap} from "../grid";
import {resolvePath} from "../url";
import {Point, SensorConfig} from "./types";

export async function loadGridMap(name: string): Promise<GridMap> {
    const res = await fetch(resolvePath(`data/maps/${name}.json`))
    if (!res.ok) throw new Error(`fetch failed: data/maps/${name}.json (${res.status})`)
    return parseGridMap(await res.json() as GridMapJson)
}

// 시나리오 JSON (maps/scenarios/<slug>.yaml의 그대로의 수출본).
export interface ScenarioJson {
    map: string;                       // "../grid/<name>.yaml" — 슬러그만 뽑아 쓴다.
    path: Point[];                     // GT 웨이포인트 폴리라인 (world 좌표)
    step_meters: number;
    sensor: SensorConfig;
    landmarks?: Point[];
    odom_noise: { sigma_xy: number; sigma_theta: number };
    seed: number;
}

export interface LoadedScenario {
    slug: string;
    scenario: ScenarioJson;
    grid: GridMap;
}

// 시나리오 슬러그 → (파싱된 시나리오, 그 yaml이 가리키는 맵).
export async function loadScenario(slug: string): Promise<LoadedScenario> {
    const res = await fetch(resolvePath(`data/scenarios/${slug}.json`))
    if (!res.ok) throw new Error(`fetch failed: data/scenarios/${slug}.json (${res.status})`)
    const scenario = await res.json() as ScenarioJson
    // "../grid/office01.yaml" → "office01"
    const mapName = scenario.map.split("/").pop()!.replace(/\.ya?ml$/, "")
    const grid = await loadGridMap(mapName)
    return {slug, scenario, grid}
}
