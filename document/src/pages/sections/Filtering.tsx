import {Layer, Shape, Stage} from "react-konva";
import {T, useLang, useTr} from "../../libs/i18n";
import {BlockMath, InlineMath} from "../../components/math/Tex";
import Terms from "../../components/math/Terms";
import CanvasFigure from "../../components/CanvasFigure";
import {HEAT_LOW, PARTICLE_COLOR, mixHex, withAlpha} from "../../libs/trace/timeline";
import {useCanvasColors} from "../../libs/useTheme";

// filtering 갈래 소개 — SLAM 의 두 절반을 분리해 가르치는 뿌리 갈래. 정적 피겨 하나:
// 같은 미지의 벽 앞에서 왼쪽은 알려진 지도, 가운데는 자세의 이산 belief(히스토그램),
// 오른쪽은 같은 문제에 대한 표본 표현(입자).

// 고정 픽셀 격자 (# = 점유) — corridor01 과 같은 구조: 균일한 한 셀 두께의 경계와
// 중앙 행에 문이 난 기둥. 세 패널이 같은 맵을 공유해야 "같은 문제, 다른 belief 표현"이
// 눈에 들어온다 — 그래서 belief 는 MAP 에서 파생한다 (따로 쓰지 않아 어긋날 수 없다).
// 좌표 계약: 셀은 [row, col] (row 0 = 최상단), 입자는 [col, row] — 같은 읽기 순서.
const MAP: string[] = [
    "#############",
    "#...#...#...#",
    "#...#...#...#",
    "#...........#",
    "#...#...#...#",
    "#...#...#...#",
    "#############",
];

// 자세 가설이 몰려 있는 핫스팟 자유 셀 ([row, col]) — belief 열과 입자 무더기가 같은
// 곳을 가리킨다. 벽을 핫스팟에 두면 안 된다: 자세는 벽에 있을 수 없다.
const HOT_CELLS: Array<[number, number]> = [[2, 10], [3, 10], [4, 10]];

// 히스토그램 패널의 셀 확률: 핫스팟 0.9, 나머지 자유 셀은 옅게 (0.05) — MAP 에서 파생.
const beliefAt = (r: number, c: number): number =>
    HOT_CELLS.some(([hr, hc]) => hr === r && hc === c) ? 0.9 : 0.05;

// 입자 패널: 같은 핫스팟 주위의 표본 — [col, row], MAP 과 같은 읽기 순서 (row 0 = 위).
const PARTICLES: Array<[number, number]> = [
    [9.7, 2.8], [10.4, 3.6], [10.1, 4.3], [10.8, 2.5], [9.4, 3.4], [10.2, 2.9],
];

const BeliefVsParticles = () => {
    const colors = useCanvasColors();
    // 피겨 안의 캡션도 언어 전환을 따른다.
    const lang = useLang().lang;
    const t = (en: string, ko: string) => (lang === "ko" ? ko : en);
    const cell = 16;
    const w = MAP[0].length * cell;
    const h = MAP.length * cell;

    const gridLayer = (belief: boolean) => (
        <Layer>
            <Shape listening={false} sceneFunc={(ctx) => {
                for (let r = 0; r < MAP.length; r++) {
                    for (let c = 0; c < MAP[0].length; c++) {
                        const occ = MAP[r][c] === "#";
                        if (!belief && !occ) continue;
                        // 히스토그램 패널: 벽은 세 패널 공통의 지도 색, 자유 셀에만 확률 열을
                        // 얹는다 — 라이브 재생의 belief 히트맵과 같은 색 (HEAT_LOW → PARTICLE).
                        ctx.fillStyle = occ
                            ? withAlpha(colors.text, 0.78)
                            : withAlpha(mixHex(HEAT_LOW, PARTICLE_COLOR, beliefAt(r, c)), 0.55);
                        ctx.beginPath();
                        ctx.rect(c * cell, r * cell, cell, cell);
                        ctx.fill();
                    }
                }
            }}/>
        </Layer>
    );

    return (
        <div className="flex flex-wrap items-start justify-center gap-6 my-5">
            <figure className="m-0">
                <Stage width={w} height={h} listening={false}>
                    {gridLayer(false)}
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("the map (known here)", "지도 (여기서는 알려짐)")}
                </figcaption>
            </figure>
            <figure className="m-0">
                <Stage width={w} height={h} listening={false}>
                    {gridLayer(true)}
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("belief over pose: one cell per hypothesis", "자세 belief: 가설 하나당 셀")}
                </figcaption>
            </figure>
            <figure className="m-0">
                <Stage width={w} height={h} listening={false}>
                    {gridLayer(false)}
                    <Shape listening={false} fill={PARTICLE_COLOR} opacity={0.9} sceneFunc={(ctx, shape) => {
                        // 입자는 [col, row]: x 가 열 (col), y 가 행 (row, 위가 0) — 격자와 같은 축.
                        for (const [x, y] of PARTICLES) {
                            ctx.beginPath();
                            ctx.arc(x * cell, y * cell, 3.4, 0, Math.PI * 2);
                            ctx.fillStrokeShape(shape);
                        }
                    }}/>
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("belief over pose: samples", "자세 belief: 표본")}
                </figcaption>
            </figure>
        </div>
    );
};

// filtering 갈래 소개 페이지 — 알고리즘 각론 전에 "이 갈래가 무엇인가"를 가르친다.
const FilteringEstimation = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    A robot that knows where it is has already solved half of SLAM. This branch is
                    that half — and its mirror: estimating the thing you don't drive. One state, one
                    motion model, one noisy sensor stream; the difference between localization and
                    mapping is only <em>what you call unknown</em>. Both halves are the same recursion,
                    so we teach the recursion first on a pose, then on a map.
                </p>}
                ko={<p>
                    자신이 어디 있는지 아는 로봇은 이미 SLAM의 절반을 푼 것이다. 이 갈래가 그 절반이고,
                    동시에 그 거울상이다 — 운전하지 않는 것을 추정하는 일. 상태 하나, 운동 모델 하나,
                    노이즈 있는 센서 스트림 하나가 같고, localization과 mapping의 차이는{" "}
                    <em>무엇을 미지라고 부르느냐</em>뿐이다. 두 절반은 같은 재귀라서, 재귀를 먼저 자세에
                    가르치고 나서 지도에 가르친다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    The state is the pose <InlineMath math="x_t = (x, y, \theta)"/> in world coordinates.
                    Every step the robot receives a noisy motion command{" "}
                    <InlineMath math="u_t = (\Delta x, \Delta y, \Delta\theta)"/> in its own frame and a
                    sensor reading: either a beam scan (endpoints of laser hits, robot frame) or point
                    landmark observations <InlineMath math="(id, \beta, r)"/>. The Markov assumption is the
                    entire contract — everything the estimate needs from the past fits in{" "}
                    <InlineMath math="x_{t-1}"/>:
                </p>}
                ko={<p>
                    상태는 world 좌표의 자세 <InlineMath math="x_t = (x, y, \theta)"/>다. 스텝마다 로봇은
                    자기 프레임의 노이즈 있는 명령 <InlineMath math="u_t = (\Delta x, \Delta y, \Delta\theta)"/>와
                    센서 관측을 받는다: 빔 스캔(레이저가 맞은 끝점들, 로봇 프레임) 또는 랜드마크 관측{" "}
                    <InlineMath math="(id, \beta, r)"/>. Markov 가정이 계약 전체다 — 과거에서 필요한 것은
                    전부 <InlineMath math="x_{t-1}"/>에 들어 있다:
                </p>}
            />
            <BlockMath math="p(x_t \mid x_{0:t-1}, z_{1:t}, u_{1:t}) = p(x_t \mid x_{t-1}, z_t)"/>
            <Terms items={[
                ["x_t", <>자세 <InlineMath math="(x,y,\theta)"/> — 미지.</>],
                ["z_t", "관측 (빔 스캔 또는 랜드마크 목록)."],
                ["u_t", "도착한 오도메트리 명령 — GT 이동에 노이즈를 더한 값."],
            ]}/>

            <h2>{t("The Recursive Bayes Filter", "재귀 Bayes 필터")}</h2>
            <T
                en={<p>
                    One belief <InlineMath math="bel(x_t)"/>, two lines of recursion. Predict by pushing the
                    belief through the motion model, update by multiplying in the sensor likelihood and
                    renormalizing:
                </p>}
                ko={<p>
                    하나의 belief <InlineMath math="bel(x_t)"/>, 두 줄의 재귀. 예측은 belief를 운동 모델로
                    밀어 넣고, 갱신은 센서 우도를 곱해 정규화한다:
                </p>}
            />
            <BlockMath math="bel(x_t)=\eta\;p(z_t\mid x_t)\!\int p(x_t\mid x_{t-1},u_t)\,bel(x_{t-1})\,dx_{t-1}"/>
            <Terms items={[
                ["bel(x_t)", "현재 belief — 표현이 갈래를 가른다 (히스토그램이냐 표본이냐)."],
                ["p(z_t\\mid x_t)", "센서 모델. 빔은 거리 노이즈, 랜드마크는 bearing/range 노이즈."],
                ["\\eta", "정규화 상수."],
            ]}/>
            <T
                en={<p>
                    The integral is where the branches split. When the pose is discretized into cells, the
                    integral becomes a sum over neighbors and the belief is exact — that is the histogram
                    filter. When it can't be, you carry samples instead of numbers: draw{" "}
                    <InlineMath math="M"/> poses from the motion model, weight each by how well its predicted
                    reading matches <InlineMath math="z_t"/>, resample when the weights collapse. That is the
                    particle filter, and on a known map it has a proper name — Monte Carlo Localization.
                </p>}
                ko={<p>
                    갈래가 갈리는 지점이 적분부다. 자세를 셀로 이산화하면 적분이 이웃에 대한 합으로 바뀌고
                    belief는 정확해진다 — histogram filter다. 그럴 수 없으면 숫자 대신 표본을 싣는다: 운동
                    모델에서 <InlineMath math="M"/>개의 자세를 그리고, 예측 관측이{" "}
                    <InlineMath math="z_t"/>와 얼마나 잘 맞는지로 가중치를 매기고, 가중치가 무너지면
                    리샘플링한다. particle filter이고, 알려진 지도 위에서 하면 이름이 있다 — Monte Carlo
                    Localization.
                </p>}
            />

            <CanvasFigure label={"Belief over a pose: cells vs samples"} tight>
                <BeliefVsParticles/>
            </CanvasFigure>

            <h2>{t("Two Halves of SLAM", "둘로 갈라진 SLAM")}</h2>
            <T
                en={<p>
                    Run the same recursion with the map as the unknown and you get mapping: each cell keeps
                    its own independent belief, log-odds is the numerically honest way to carry it, and the
                    update inverts a sensor model into one number per hit. The other half — estimating pose
                    while the map is built <em>from</em> your own estimates — is where the filter_based and
                    graph_based branches take over. This branch keeps the halves apart on purpose: you cannot
                    fuse what you haven't seen separately.
                </p>}
                ko={<p>
                    같은 재귀를 지도가 미지일 때 돌리면 mapping이 된다: 셀마다 독립 belief를 들고, log-odds가
                    그것을 올리는 수치적으로 정직한 방식이고, 갱신은 센서 모델을 hit당 숫자 하나로 뒤집는다.
                    나머지 절반 — 지도가 <em>자기 추정에서</em> 지어지는 동안 자세를 추정하는 쪽 — 은 filter_based와
                    graph_based 갈래가 이어받는다. 이 갈래는 두 절반을 일부러 분리한다: 따로 본 것을 합칠 수 없다.
                </p>}
            />

            <h2>{t("What Is Written, and What Comes Next", "집필된 것과 그다음")}</h2>
            <T
                en={<ul>
                    <li><strong>Histogram Filter</strong> — the recursion with a discrete pose space: exact,
                        exhaustive, exponential. The only member whose belief is the distribution itself.</li>
                    <li><strong>Log-Odds Grid Mapping</strong> — the other half: known pose, unknown map, one
                        independent log-odds cell per pixel (Moravec/Elfes, in Thrun's textbook form).</li>
                    <li><strong>Particle Filter</strong> — importance sampling and resampling with no map at all;
                        the bootstrap filter on the pose itself.</li>
                    <li><strong>MCL</strong> — the particle filter on a known map, plus KLD-sampling's adaptive
                        sample count (Pfaff et al.).</li>
                </ul>}
                ko={<ul>
                    <li><strong>Histogram Filter</strong> — 이산 자세 공간에서의 재귀: 정확하고, 완전하고, 지수 폭발.
                        belief가 곧 분포인 유일한 회원.</li>
                    <li><strong>Log-Odds Grid Mapping</strong> — 나머지 절반: 자세는 알려짐, 지도가 미지. 픽셀마다
                        독립 log-odds 셀(Moravec/Elfes, Thrun 교과서 형식).</li>
                    <li><strong>Particle Filter</strong> — 지도 없이 importance sampling과 리샘플링만; 자세 자체에
                        대한 bootstrap 필터.</li>
                    <li><strong>MCL</strong> — 알려진 지도 위의 입자 필터, 그리고 KLD-sampling의 적응형 표본 수
                        (Pfaff et al.).</li>
                </ul>}
            />
        </>
    )
}

export default FilteringEstimation
