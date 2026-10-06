// 웹 라이브 TS 엔진과 저장소 python demo 의 정밀 대조.
// public/data/traces/<algo>/<scenario>.py.jsonl.gz 에서 run_started(params·seed 포함)를
// 읽고, 같은 입력(시나리오 JSON + 격자 + 파라미터)으로 TS 엔진을 돌려 run_finished 의
// 지표를 비교한다. JS 의 Math.* 는 CPython math(libm) 와 ulp 단위로 다를 수 있어서
// 부동소수 지표는 허용 오차 비교가 설계 사실이다 (기본 tol: 1e-9 — 정수 지표는 어떤
// tol 보다 훨씬 크게 갈라지므로 여전히 사실상 exact). 알고리즘이 추가되면 RUNNERS 에
// run<algo>(grid, scenario, params) → TraceEvent[] 를 등록하고 CHECKS 에 시나리오를 적는다.
// 실행: node scripts/check-engine-parity.mjs (esbuild 로 번들).
import {execFileSync} from "node:child_process";
import {gunzipSync} from "node:zlib";
import {mkdtempSync, readFileSync, rmSync} from "node:fs";
import {tmpdir} from "node:os";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";
import {createRequire} from "node:module";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = mkdtempSync(join(tmpdir(), "parity-"));
const bundle = join(outDir, "engines.cjs");
execFileSync(join(root, "node_modules", ".bin", "esbuild"), [
    join(root, "scripts", "parity-entry.ts"),
    "--bundle", "--format=cjs", "--platform=node", `--outfile=${bundle}`,
    "--loader:.ts=ts",
], {stdio: "pipe"});
const engines = createRequire(import.meta.url)(bundle);

// 시나리오 JSON은 브라우저 로더(libs/trace/load.ts)가 읽는 그 파일 그대로 — parity 는
// 같은 입력을 쓴다는 계약 자체가 검사 대상이다. map 필드("../grid/<name>.yaml")에서 맵
// 슬러그를 뽑아 격자를 만든다.
const loadScenario = (slug) => {
    const scenario = JSON.parse(
        readFileSync(join(root, "public", "data", "scenarios", `${slug}.json`), "utf-8"));
    const mapName = scenario.map.split("/").pop().replace(/\.ya?ml$/, "");
    const grid = engines.parseGridMap(JSON.parse(
        readFileSync(join(root, "public", "data", "maps", `${mapName}.json`), "utf-8")));
    return {scenario, grid};
};

const loadTrace = (algo, slug) =>
    gunzipSync(readFileSync(join(root, "public", "data", "traces", algo, `${slug}.py.jsonl.gz`)))
        .toString("utf-8").trim().split("\n").map((l) => JSON.parse(l));

const finalOf = (events) => events[events.length - 1];

// 알고리즘 slug → TS 엔진 러너. 시그니처는 데모와 같은 계약:
//   run(grid, scenario, params) → TraceEvent[]
// (params 는 python demo 가 run_started 에 실어 보낸 파라미터 맵 그대로 — stochastic
// 알고리즘의 seed 포함. 시나리오 seed 는 demo 가 주입하므로 여기선 건드리지 않는다.)
const RUNNERS = {histogram_filter: engines.runHistogramFilter};

// algo × scenario 조합. metricKeys 를 생략하면 python trace 의 모든 지표 키를 기본 tol 로
// 비교한다. 시나리오 슬러그는 public/data/scenarios/<slug>.json 의 그 이름이다.
const CHECKS = [{algo: "histogram_filter", scenarios: ["corridor01_back_and_forth", "corridor02_ambiguous"]}];

const DEFAULT_TOL = 1e-9;

let failures = 0;
for (const check of CHECKS) {
    for (const slug of check.scenarios) {
        let events;
        try {
            events = loadTrace(check.algo, slug);
        } catch {
            continue;   // 해당 시나리오의 trace 미탑재 — 검사 대상 아님
        }
        const expected = finalOf(events);
        const started = events[0];
        const {scenario, grid} = loadScenario(slug);
        const got = finalOf(RUNNERS[check.algo](grid, scenario, started.params ?? {}));

        const problems = [];
        const keys = check.metricKeys
            ?? Object.keys(expected.metrics ?? {}).map((key) => ({key, tol: DEFAULT_TOL}));
        for (const {key, tol} of keys) {
            const a = got.metrics?.[key];
            const b = expected.metrics?.[key];
            if (a === undefined || b === undefined || Math.abs(a - b) > tol) {
                problems.push(`${key} ${a} != ${b}`);
            }
        }
        const tag = `${check.algo} × ${slug}`;
        if (problems.length) {
            failures++;
            console.log(`FAIL ${tag}: ${problems.join("; ")}`);
        } else {
            console.log(`ok   ${tag}`);
        }
    }
}
rmSync(outDir, {recursive: true, force: true});
if (failures) {
    console.error(`\n${failures} parity failure(s)`);
    process.exit(1);
}
console.log(CHECKS.length ? "\nall engines match the repository demos" : "no engine checks registered yet — nothing to compare");
