import {Layer, Shape, Stage} from "react-konva";
import {T, useLang, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import CanvasFigure from "../../components/CanvasFigure";
import {useCanvasColors} from "../../libs/useTheme";

// registration 갈래 소개 — 바퀴 없는 오도메트리. 정적 피겨: 같은 벽의 두 스캔이 어긋난 채로
// (왼쪽) 만나고, 강체 변환을 풀면(오른쪽) 겹친다.

// 작은 벽 조각 점구름 (정적). 왼쪽 패널은 map 점(점선 호)과 scan 점(실선 호)이 어긋나 있고,
// 오른쪽은 정렬 후 — 같은 점이 겹친다.
const MAP_PTS: Array<[number, number]> = [
    [14, 30], [20, 22], [28, 16], [37, 12], [47, 10], [57, 10], [67, 12], [76, 16],
];
const SCAN_PTS: Array<[number, number]> = MAP_PTS.map(([x, y]) => [x + 9, y + 6]);

function cloudLayer(pts: Array<[number, number]>, color: string, alpha: number) {
    return (
        <Shape listening={false} fill={color} opacity={alpha} sceneFunc={(ctx, shape) => {
            for (const [x, y] of pts) {
                ctx.beginPath();
                ctx.arc(x, y, 2.6, 0, Math.PI * 2);
                ctx.fillStrokeShape(shape);
            }
        }}/>
    )
}

const AlignFigure = () => {
    const colors = useCanvasColors();
    // 피겨 안의 캡션도 언어 전환을 따른다.
    const lang = useLang().lang;
    const t = (en: string, ko: string) => (lang === "ko" ? ko : en);
    return (
        <div className="flex flex-wrap items-start justify-center gap-6 my-5">
            <figure className="m-0">
                <Stage width={92} height={54} listening={false}>
                    <Layer>
                        {cloudLayer(MAP_PTS, colors.muted, 0.8)}
                        {cloudLayer(SCAN_PTS, colors.accent, 0.95)}
                    </Layer>
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("two scans, misaligned", "어긋난 두 스캔")}
                </figcaption>
            </figure>
            <figure className="m-0">
                <Stage width={92} height={54} listening={false}>
                    <Layer>
                        {cloudLayer(MAP_PTS, colors.muted, 0.8)}
                        {cloudLayer(MAP_PTS, colors.accent, 0.95)}
                    </Layer>
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("after solving the rigid transform", "강체 변환을 푼 뒤")}
                </figcaption>
            </figure>
        </div>
    );
};

// registration 갈래 소개 페이지.
const Registration = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every branch above this one rides on odometry — wheels that slip, wheels that drift. This
                    branch is the other way to move: take two scans of the same wall and solve for the rigid
                    transform between them. No wheel anywhere in the chain. The LiDAR-inertial members of the
                    filter_based branch run on exactly this underneath — their "odometry" <em>is</em> registration.
                </p>}
                ko={<p>
                    이 갈래 위의 모든 갈래는 오도메트를 탄다 — 미끄러지고 드리프트하는 바퀴를. 이 갈래는
                    움직이는 다른 방법이다: 같은 벽의 두 스캔을 받아 그 사이의 강체 변환을 푼다. 사슬 어디에도
                    바퀴가 없다. filter_based 갈래의 라이다-관성 회원들이 속으로 바로 이것 위에 서 있다 —
                    그들의 "오도메트리"는 곧 <em>등록</em>이다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    Given a reference point cloud (the map, or the previous scan){" "}
                    <InlineMath math="\mathcal{M}"/> and a moving scan <InlineMath math="\mathcal{S}"/>, find the
                    rigid motion <InlineMath math="T \in SE(2)"/> that best aligns them — "best" being some sum of
                    squared residuals over correspondences. The output is a pose difference, so this is odometry:
                    feed it scan after scan and you integrate a trajectory with no wheel in sight.
                </p>}
                ko={<p>
                    기준 점구름(지도, 또는 이전 스캔) <InlineMath math="\mathcal{M}"/>과 움직이는 스캔{" "}
                    <InlineMath math="\mathcal{S}"/>이 주어지면 대응 위 잔차 제곱합을 최소화하는 강체 운동{" "}
                    <InlineMath math="T \in SE(2)"/>를 찾는다. 출력이 자세 차이니 이건 오도메트리다: 스캔을
                    계속 먹이면 바퀴 없이 궤적이 적분된다.
                </p>}
            />

            <CanvasFigure label={"Registration = solve for the transform between point clouds"} tight>
                <AlignFigure/>
            </CanvasFigure>

            <h2>{t("Why Without Odometry", "왜 바퀴 없는 오도메트리인가")}</h2>
            <T
                en={<p>
                    Wheel odometry is a dead reckoning: every slip error integrates into the trajectory forever.
                    A scanner that measures geometry doesn't drift like that — its estimate of "how far did I move"
                    comes from the walls themselves, not from counting wheel revolutions. The cost is honest too:
                    registration needs structure in the scene (a featureless corridor gives it nothing to lock on),
                    and it degrades where geometry repeats.
                </p>}
                ko={<p>
                    바퀴 오도메트는 사산법이다: 미끄러짐 오차가 매 스텝 궤적에 영구히 적분된다. 형상을 재는
                    스캐너는 그렇게 드리프트하지 않는다 — "얼마나 움직였나"의 추정이 바퀴 회전 수가 아니라 벽
                    자체에서 나온다. 대가도 정직하다: 등록은 장면에 구조를 필요로 하고(구조 없는 통로는 고정할
                    게 없다), 형상이 반복되는 곳에서 성능이 준다.
                </p>}
            />

            <h2>{t("Correspondence, then Alignment", "대응, 그리고 정렬")}</h2>
            <T
                en={<p>
                    Both members alternate the same two steps until convergence. Step one: given a pose guess, find
                    correspondences — nearest point for ICP, containing cell for NDT. Step two: with correspondences
                    fixed, the best rigid transform has a closed form (2D: centroid alignment plus a single rotation).
                    The correspondence step is what makes it iterative and what makes the initial guess matter; the
                    alignment step is why the answer is exact given its input.
                </p>}
                ko={<p>
                    두 회원 모두 같은 두 단계를 수렴까지 번갈아 돈다. 첫 단계: 자세 추측이 주어지면 대응을 찾는다 —
                    ICP는 최근접 점, NDT는 점을 담는 셀. 두 번째 단계: 대응을 고정하면 최선의 강체 변환은 닫힌 형태가 있다
                    (2D에서는 중심점 정렬에 회전 하나). 대응 단계가 이것을 반복적으로 만들고 초기 추측을 중요하게 만든다.
                    정렬 단계가 입력이 주어졌을 때 답이 정확한 이유이고.
                </p>}
            />

            <h2>{t("From Points to Distributions", "점에서 분포로")}</h2>
            <T
                en={<ul>
                    <li><strong>ICP</strong> (Besl &amp; McKay 1992) — point-to-point correspondence, closed-form
                        rigid solve, iterate. The reference every later registration is measured against.</li>
                    <li><strong>NDT</strong> (Biber &amp; Strasser 2003) — glue a Gaussian to each cell and score the
                        scan against the distributions: the objective stops being nearest-neighbor spikes and becomes
                        smooth in the pose, which is what makes it behave under sparse scans.</li>
                </ul>}
                ko={<ul>
                    <li><strong>ICP</strong>(Besl &amp; McKay 1992) — 점-점 대응, 닫힌 형태의 강체 풀이, 반복. 모든
                        후속 등록이 이것을 기준으로 측정되는 기준점.</li>
                    <li><strong>NDT</strong>(Biber &amp; Strasser 2003) — 셀마다 Gaussian을 붙이고 스캔을 분포들에
                            대해 채점한다: 목적함수가 최근접 점의 가시밭에서 자세에 매끄러운 곡선이 되고, 그래서
                            드문드문한 스캔에서도 다룬다.</li>
                </ul>}
            />
        </>
    )
}

export default Registration
