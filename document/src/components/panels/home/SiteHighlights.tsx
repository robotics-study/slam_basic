import {useTr} from "../../../libs/i18n";

// 홈에서 "이 사이트가 무엇인가"를 사실만으로 세우는 카드 줄.
const SiteHighlights = () => {
    const t = useTr()

    const cards: Array<{kicker: string; title: string; desc: string}> = [
        {
            kicker: t("one recursion", "하나의 재귀"),
            title: t("The same filter, every branch", "갈래마다 같은 필터"),
            desc: t(
                "Every estimator here is the same predict–update recursion wearing different " +
                "clothes: a histogram over cells, particles carrying per-landmark Kalman " +
                "filters, log-odds grids grown on particles, pose graphs that marginalize the " +
                "map out. The genealogy is one axis — what carries the belief.",
                "여기 있는 모든 추정기는 같은 predict–update 재귀가 옷만 바꿔 입은 모습입니다. " +
                "셀 위의 히스토그램, 랜드마크마다 Kalman 필터를 매단 입자, 입자 위에서 자라는 " +
                "log-odds 격자, 지도를 소거해 버리는 pose graph까지. 계보의 축은 하나예요 — " +
                "belief를 무엇이 나르는가.",
            ),
        },
        {
            kicker: t("proofs", "증명"),
            title: t("Every property, proven", "모든 성질에 증명"),
            desc: t(
                "Convergence, consistency, and complexity claims come with step-by-step proofs, " +
                "not hand-waving — why the histogram filter is exact but exponential, why " +
                "Rao-Blackwellization is exact conditioning, why a graph's information matrix " +
                "stays sparse.",
                "수렴·일관성·복잡도 주장은 말로 얼버무리지 않고 단계별 증명으로 뒷받침합니다. " +
                "histogram filter가 정확한데 왜 지수적인지, Rao-Blackwell화가 왜 정확한 조건부인지, " +
                "그래프의 정보 행렬이 왜 희소로 남는지에 대한 증명까지요.",
            ),
        },
        {
            kicker: t("live demos", "라이브 데모"),
            title: t("Draw a wall. Watch the belief change.", "벽을 그리면 belief가 바뀝니다"),
            desc: t(
                "Every page runs the estimator live in your browser — the exact mirror of the " +
                "Python/C++ engine. Draw walls, move the waypoints, retune the sensor noise and " +
                "the algorithm's parameters: every edit re-runs the whole episode and replays it " +
                "step by step — scan fan, belief heat, particle cloud, landmark estimates, loop " +
                "closures, ATE/RPE at the end.",
                "모든 페이지에서 추정기를 브라우저에서 직접 돌립니다. 엔진은 Python/C++ 구현과 " +
                "필드 단위로 동일한 미러예요. 벽을 그리고, 웨이포인트를 옮기고, 센서 노이즈와 파라미터를 " +
                "조정하면 매 편집이 에피소드 전체를 다시 돌려 스텝별로 재생합니다 — 레이저 부채꼴, " +
                "belief 열지도, 입자 구름, 랜드마크 추정, 루프 클로저, 마지막의 ATE/RPE까지.",
            ),
        },
        {
            kicker: t("full source", "전체 소스"),
            title: t("Read the real implementation", "실제 구현을 그대로 읽기"),
            desc: t(
                "Each page ends with the complete C++ and Python source that the explanations " +
                "describe. The live sandbox engine is a third mirror of the same estimator, and all " +
                "three emit field-for-field identical traces on every scenario, verified on every build.",
                "각 페이지 끝에는 설명이 가리키는 C++·Python 구현 전체가 그대로 붙어 있습니다. " +
                "라이브 sandbox 엔진은 같은 추정기의 세 번째 미러이고, 셋 모두 모든 시나리오에서 " +
                "필드 단위로 동일한 trace를 방출합니다. 빌드마다 검증됩니다.",
            ),
        },
    ]

    return (
        <div className="grid gap-4 sm:grid-cols-2 mb-12">
            {cards.map((c) => (
                <div key={c.title}
                     className="flex flex-col gap-2 rounded-[var(--radius)] border border-border bg-surface p-5 shadow-card">
                    <span className="text-xs font-bold uppercase tracking-wider"
                          style={{color: "var(--accent)"}}>{c.kicker}</span>
                    <span className="font-semibold" style={{fontSize: "1.02rem"}}>{c.title}</span>
                    <p className="m-0 text-sm text-muted leading-relaxed">{c.desc}</p>
                </div>
            ))}
        </div>
    )
}

export default SiteHighlights
