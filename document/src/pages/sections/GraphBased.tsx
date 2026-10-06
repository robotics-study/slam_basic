import {Layer, Shape, Stage} from "react-konva";
import {T, useLang, useTr} from "../../libs/i18n";
import {BlockMath} from "../../components/math/Tex";
import Terms from "../../components/math/Terms";
import CanvasFigure from "../../components/CanvasFigure";
import {useCanvasColors} from "../../libs/useTheme";

// graph_based 갈래 소개 — 측정을 제약으로 모은다. 정적 피겨: 키프레임 노드들, odometry 체인
// (옅은 실선), 루프 클로저(빨간 파선).

const NODES: Array<[number, number]> = [[24, 86], [58, 50], [96, 34], [140, 40], [172, 66], [150, 96]];
// odometry 체인: 연속 노드 간. 루프: 마지막 → 첫 (노드를 다시 관측).
const ODOM_EDGES: Array<[number, number]> = [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5]];

const GraphFigure = () => {
    const colors = useCanvasColors();
    const lang = useLang().lang;
    const t = (en: string, ko: string) => (lang === "ko" ? ko : en);
    return (
        <div className="flex flex-wrap items-start justify-center gap-6 my-5">
            <figure className="m-0">
                <Stage width={200} height={130} listening={false}>
                    <Layer>
                        {/* odometry 체인 — 인접 노드 간 제약 */}
                        <Shape listening={false} stroke={colors.muted} opacity={0.55} strokeWidth={1.4}
                               sceneFunc={(ctx, shape) => {
                                   for (const [a, b] of ODOM_EDGES) {
                                       ctx.beginPath();
                                       ctx.moveTo(NODES[a][0], NODES[a][1]);
                                       ctx.lineTo(NODES[b][0], NODES[b][1]);
                                       ctx.fillStrokeShape(shape);
                                   }
                               }}/>
                        {/* 루프 클로저 — 빨간 파선 */}
                        <Shape listening={false} stroke="#dc2626" opacity={0.9} strokeWidth={1.8}
                               sceneFunc={(ctx, shape) => {
                                   const [a, b] = [NODES[5], NODES[0]];
                                   ctx.beginPath();
                                   ctx.moveTo(a[0], a[1]);
                                   ctx.lineTo(b[0], b[1]);
                                   ctx.fillStrokeShape(shape);
                               }} dash={[6, 4]}/>
                        {/* 노드 */}
                        <Shape listening={false} fill={colors.accent} sceneFunc={(ctx, shape) => {
                            for (const [x, y] of NODES) {
                                ctx.beginPath();
                                ctx.arc(x, y, 5.5, 0, Math.PI * 2);
                                ctx.fillStrokeShape(shape);
                            }
                        }}/>
                    </Layer>
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("pose nodes, odometry edges (faint), one loop closure (red dashed)",
                       "자세 노드들, odometry 간선(옅은 것), 루프 클로저 하나(빨간 파선)")}
                </figcaption>
            </figure>
        </div>
    );
};

// graph_based 갈래 소개 페이지.
const GraphBased = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    A filter commits: it folds each measurement into the state once and forgets the measurement.
                    The graph branch refuses to forget — every pose visited stays a variable, every measurement
                    becomes an edge, and at the end everything is solved together. This is smoothing where filtering
                    was, and it changes what a loop closure <em>is</em>: not a lucky correction, an ordinary edge
                    between two old variables.
                </p>}
                ko={<p>
                    필터는 약속한다: 관측을 상태에 한 번 접어 넣고 관측 자체는 잊는다. 그래프 갈래는 잊기를 거부한다 —
                    지나간 모든 자세가 변수로 남고, 모든 관측이 간선이 되고, 마지막에 전부 함께 풀린다. 필터가 있던
                    자리의 smoothing이고, 그래서 루프 클로저가 <em>무엇</em>인지도 바뀐다: 운 좋은 정정이 아니라, 오래된
                    변수 둘 사이의 평범한 간선이다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    Given odometry edges and re-observation constraints, find the trajectory that best satisfies all
                    of them at once — a least-squares problem in information form:
                </p>}
                ko={<p>
                    odometry 간선과 재관측 제약이 주어지면, 그 전부를 동시에 가장 잘 만족하는 궤적을 찾는다 — 정보
                    형식의 최소제곱 문제다:
                </p>}
            />
            <BlockMath math="x^{*}=\arg\min_x \sum_{(i,j)\in E}\bigl\|z_{ij}-h(x_i,x_j)\bigr\|_{{\Omega}_{ij}}^{2}"/>
            <Terms items={[
                ["x_i, x_j", "노드 — 각 스텝의 자세 변수."],
                ["z_{ij}", "간선의 관측: 노드 i 프레임에서 재고정된 상대 자세."],
                ["\\Omega_{ij}", "그 제약의 정보 행렬 — 신뢰가 강할수록 간선이 단단하다."],
            ]}/>

            <CanvasFigure label={"The graph: nodes are poses, edges are constraints"} tight>
                <GraphFigure/>
            </CanvasFigure>

            <h2>{t("From Filter to Graph", "필터에서 그래프로")}</h2>
            <T
                en={<p>
                    An EKF-SLAM update is already a graph operation in disguise — the Kalman gain is one block-row of
                    an information solve that keeps every past pose. The graph branch simply stops hiding it: keep all
                    the pose variables, and either marginalize landmarks out into pose-pose constraints (Sparse Pose
                    Adjustment — the landmark lives on only as a measurement subspace) or keep them as variables too
                    (full graph SLAM). Sparsity is the payoff: each constraint touches two nodes, so the information
                    matrix stays sparse and Cholesky does what the filter could not.
                </p>}
                ko={<p>
                    EKF-SLAM의 갱신은 이미 변장한 그래프 연산이다 — Kalman gain은 과거 자세를 전부 보존하는 정보
                    풀이의 행 하나일 뿐. 그래프 갈래는 그것을 그냥 숨기지 않는다: 모든 자세 변수를 그대로 두고,
                    랜드마크를 pose-pose 제약으로 주변화하든(Sparse Pose Adjustment — 랜드마크는 측정 부분공간으로만
                    남는다) 역시 변수로 둔다(완전 그래프 SLAM). 대가는 희소성이다: 각 제약이 노드 둘만 건드리니 정보
                    행렬은 희소하고, 필터가 못 하던 것을 Cholesky가 해 낸다.
                </p>}
            />

            <h2>{t("Loop Closure in Practice", "실제의 루프 클로저")}</h2>
            <T
                en={<p>
                    A graph is only as honest as its edges, and the edge that saves a trajectory from drift is the
                    one that says "you are here again". Cartographer (Hess et al. 2016) is this branch assembled into
                    a running system: scans accumulate into submaps, loop closure becomes branch-and-bound search over
                    past submap poses — an exhaustive correlation with pruning — and every accepted match enters as
                    exactly the constraint above. The registration branch's scan matching is the leaf of that tree;
                    this page's machinery is the trunk.
                </p>}
                ko={<p>
                    그래프는 간선만큼만 정직하고, 궤적을 드리프트에서 구하는 간선은 "너 여기 다시 왔어"라고 말하는
                    간선이다. Cartographer(Hess et al. 2016)는 이 갈래가 실행 시스템으로 조립된 것이다: 스캔이 submap에
                    쌓이고, 루프 클로저는 과거 submap 자세들에 대한 branch-and-bound 탐색 — 가지치기 있는 완전 상관 — 이
                    되며, 승인된 일치마다 위와 똑같은 제약으로 들어간다. registration 갈래의 스캔 매칭이 그 트리의 잎이고,
                    이 페이지의 기계가 줄기다.
                </p>}
            />
        </>
    )
}

export default GraphBased
