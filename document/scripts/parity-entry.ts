// check-engine-parity.mjs 가 esbuild 로 번들하는 진입점 — 웹 라이브 엔진 전부를 재수출한다.
// 알고리즘이 추가되면 run<algo>(grid, scenario, params) → TraceEvent[] 를 여기에 export 한다
// (params 는 run_started 의 그것 — seed 포함. 데모와 동일한 계약).
export {parseGridMap} from "../src/libs/grid";
