import {Layer, Shape, Stage} from "react-konva";
import {T, useLang, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import CanvasFigure from "../../components/CanvasFigure";
import {useCanvasColors} from "../../libs/useTheme";

// features 갈래 소개 — 랜드마크는 어디서 오는가. 정적 피겨: 한 스캔의 점들이 eps 반지름으로
// 클러스터되어 id 있는 랜드마크 후보가 되는 순간.

// 가상의 스캔 점들 (픽셀 좌표) — 두 무더기와 잡점 하나.
const POINTS: Array<[number, number]> = [
    [26, 18], [30, 22], [24, 25], [29, 14],   // cluster 0
    [70, 40], [76, 36], [72, 46], [78, 44], [74, 30], // cluster 1
    [52, 12],                                  // 잡점 (혼자)
];

const ClusterFigure = () => {
    const colors = useCanvasColors();
    const lang = useLang().lang;
    const t = (en: string, ko: string) => (lang === "ko" ? ko : en);
    return (
        <div className="flex flex-wrap items-start justify-center gap-6 my-5">
            <figure className="m-0">
                <Stage width={100} height={60} listening={false}>
                    <Layer>
                        <Shape listening={false} sceneFunc={(ctx, shape) => {
                            // eps 원 두 개 (클러스터 반지름) — core point 판정의 무대.
                            for (const [cx, cy] of [[27, 20], [74, 39]] as Array<[number, number]>) {
                                ctx.beginPath();
                                ctx.arc(cx, cy, 13, 0, Math.PI * 2);
                                ctx.fillStrokeShape(shape);
                            }
                        }} stroke={colors.muted} opacity={0.5} dash={[4, 3]}/>
                        <Shape listening={false} fill={colors.text} sceneFunc={(ctx, shape) => {
                            for (const [x, y] of POINTS) {
                                ctx.beginPath();
                                ctx.arc(x, y, 2.6, 0, Math.PI * 2);
                                ctx.fillStrokeShape(shape);
                            }
                        }} opacity={0.9}/>
                    </Layer>
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("one scan's points + the eps ball", "한 스캔의 점들 + eps 반지름")}
                </figcaption>
            </figure>
        </div>
    );
};

// features 갈래 소개 페이지.
const Features = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The landmark branches above this one are honest: they consume{" "}
                    <InlineMath math="(id, \beta, r)"/> observations with association <em>given</em>. But an id
                    list has to come from somewhere. A raw scan is hundreds of anonymous points that change every
                    step — a landmark is what survives when you cluster those points into stable candidates and
                    hand each cluster one identity. This branch is that extraction, done deterministically so the
                    ids stay identical across languages and runs.
                </p>}
                ko={<p>
                    이 갈래 위의 랜드마크 계열은 정직하다: association을 <em>주어진 것</em>으로 소비한다 —{" "}
                    <InlineMath math="(id, \beta, r)"/>. 그런데 그 id 목록은 어딘가에서 와야 한다. 날스캔은
                    스텝마다 바뀌는 수백 개의 무명 점이고, 랜드마크는 그 점들을 안정적인 후보로 클러스터링하고
                    각 클러스터에 정체성 하나를 넘길 때 남는 것이다. 이 갈래는 그 추출이고, id가 언어와 실행을
                    넘어 동일하게 남도록 결정적으로 수행한다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    Input: one scan — a list of hit points in the robot frame, anonymous and dense. Output: a list
                    of candidate landmarks with stable identities, so that the same physical corner observed at{" "}
                    <InlineMath math="t"/> and <InlineMath math="t+50"/> carries the same id. Stability is the whole
                    problem; clustering is just how you get it.
                </p>}
                ko={<p>
                    입력: 스캔 하나 — 로봇 프레임의 무명 점 목록, 빽빽하고 이름 없다. 출력: 안정적인 정체성을 가진
                    후보 랜드마크 목록. 같은 물리적 모서리가 <InlineMath math="t"/>와 <InlineMath math="t+50"/>에서
                    같은 id를 taşı는 것. 안정성이 문제 전체이고, 클러스터링은 그것을 얻는 수단일 뿐이다.
                </p>}
            />

            <CanvasFigure label={"Density over one scan: two clusters survive, the lone point does not"} tight>
                <ClusterFigure/>
            </CanvasFigure>

            <h2>{t("Why Landmarks at All", "왜 랜드마크인가")}</h2>
            <T
                en={<p>
                    A grid map carries every hit; a landmark list carries only what repeats. The filter_based branch
                    pays per landmark, not per cell — its state stays small precisely because someone upstream
                    decided which points deserved an id. That decision is a modeling choice with teeth: cluster too
                    eagerly and ids flicker between steps (association lies), cluster too greedily and corners merge
                    into one smeared landmark. The determinism contract — expand core points strictly in point-index
                    order — exists so two languages extract the <em>same</em> landmarks from the same scan.
                </p>}
                ko={<p>
                    격자 지도는 맞은 것 전부를 싣고, 랜드마크 목록은 반복되는 것만 싣는다. filter_based 갈래는
                    셀당 아니라 랜드마크당 비용을 낸다 — 상류에서 어떤 점이 id를 받을 자격이 있는지 누군가 정했기
                    때문에 상태가 작은 것이다. 그 결정은 이빨을 가진 모델링 선택이다: 너무 쉽게 클러스터하면 id가
                    스텝마다 깜빡여 association이 거짓말하고, 너무 탐욕스럽게 클러스터하면 모서리들이 하나의 번진
                    랜드마크로 합쳐진다. core 점을 점 인덱스 순서대로만 확장한다는 결정성 계약은 두 언어가 같은
                    스캔에서 <em>같은</em> 랜드마크를 뽑아내게 하려고 존재한다.
                </p>}
            />

            <h2>{t("Density Clustering", "밀도 클러스터링")}</h2>
            <T
                en={<p>
                    The extractor is DBSCAN (Ester et al., KDD 1996; the survey by Kriegel, Schubert &amp; Zimek is
                    the readable tour): a point is <em>core</em> when its <InlineMath math="\varepsilon"/>-ball holds
                    at least <InlineMath math="minPts"/> points, clusters grow from core points through density
                    reachability, and noise is whatever no cluster claimed. No centroid iteration, no preset count —
                    the cluster count falls out of the geometry, which is exactly what a landmark list needs. The
                    algorithm page derives it; this page exists so the landmark branches never have to pretend ids
                    fall from the sky.
                </p>}
                ko={<p>
                    추출기는 DBSCAN(Ester et al., KDD 1996; 읽기 좋은 안내는 Kriegel, Schubert &amp; Zimek의 서베이):
                    어떤 점의 <InlineMath math="\varepsilon"/> 공이 <InlineMath math="minPts"/>개 이상의 점을 담으면
                    그 점은 <em>core</em>이고, 클러스터는 core 점에서 밀도 도달성으로 자라고, 노이즈는 어느 클러스터도
                    가져가지 않은 것이다. 중심 반복 없고, 개수 지정 없다 — 클러스터 수가 기하에서 떨어져 나오는데,
                    바로 랜드마크 목록이 필요로 하는 것이다. 알고리즘 페이지가 그것을 유도하고, 이 페이지는 랜드마크
                    갈래들이 id가 하늘에서 떨어진 척하지 않게 하려고 존재한다.
                </p>}
            />
        </>
    )
}

export default Features
