import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import {SandboxScene} from "../../../components/panels/Sandbox";
import {runGridMapping} from "../../../libs/algorithms/grid-mapping";
import pyImpl from "../../../../../python/slam/filtering/grid_mapping.py?raw";
import cppImpl from "../../../../../cpp/include/slam/filtering/grid_mapping.hpp?raw";
import cppSrc from "../../../../../cpp/src/filtering/grid_mapping.cpp?raw";

const REPO = "https://github.com/robotics-study/slam_basic"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const GridMapping = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the other half of SLAM. The histogram filter spent a whole belief vector on the
                    ROBOT while the map sat there, known. Here the roles swap: the pose is GIVEN — noiseless
                    odometry integrated onto a declared start pose — and the belief lives in the MAP. Each cell
                    carries one scalar probability of being occupied, updated every time a beam passes through
                    it or lands on it. The cells are treated as independent (that naive independence assumption
                    IS the model), which collapses the joint posterior over millions of cells into millions of
                    independent scalar recursions. This is Moravec's 1988 artificial landmarks and Elfes' 1989
                    occupancy grid, in the log-odds form the textbook teaches — and it is exactly what the
                    filter_based branch will later bolt onto a pose estimator: nothing here changes except where
                    the poses come from.
                </p>}
                ko={<p>
                    이것은 SLAM 의 다른 절반이다. histogram filter 는 belief 벡터 전체를 로봇에게 쓰고 지도는
                    알려진 것으로 놓아두었다. 여기서 역할이 바뀐다: 자세는 주어진 것 — 무노이즈 오도메트리를
                    선언된 출발 자세에 적분하고 — belief 는 지도 쪽에 산다. 셀마다 점유 확률이라는 스칼라 하나를
                    올려 두고, 빔이 그 셀을 통과하거나 착지할 때마다 갱신한다. 셀들은 독립으로 다룬다(그 naive
                    independence 가 바로 이 모델이다). 그래서 수백만 셀의 결합 사후분포가 수백만 개의 독립 스칼라
                    재귀로 무너진다. Moravec 의 1988년 artificial landmarks 와 Elfes 의 1989 occupancy grid 를
                    교과서가 가르치는 log-odds 형태로 만든 것이고, filter_based 갈래가 나중에 자세 추정기에
                    얹어낼 것이 정확히 이것이다: 자세가 어디서 오는지를 빼면 아무것도 바뀌지 않는다.
                </p>}
            />

            <h2>{t("From Bayes per Cell to Log-Odds", "셀별 Bayes 에서 log-odds 로")}</h2>
            <T
                en={<>
                    <p>
                        A cell is a binary random variable: occupied or free. The general recursion from the
                        previous page applies to each one, and independence does all the collapsing — every
                        other cell marginalizes away, and a static map kills the motion term entirely:
                    </p>
                    <BlockMath math="bel_t(m) = \eta\; p(z_t \mid m)\; bel_{t-1}(m)"/>
                    <p>
                        A product of likelihood ratios with a renormalization at every step is exactly what log
                        space turns into additions. Define the odds{" "}
                        <InlineMath math="o = bel/(1-bel)"/> and its logarithm — the <strong>log-odds</strong>{" "}
                        <InlineMath math="l = \log o"/> — and each observation becomes one addition (proof: two
                        lines, expanded below). The prior is <InlineMath math="bel = 0.5"/>, i.e.{" "}
                        <InlineMath math="l = 0"/>: every cell starts neutral, and a cell counts as occupied iff{" "}
                        <InlineMath math="l > 0"/>. <strong>Unknown is not occupied</strong>: a never-beamed cell
                        keeps its prior forever, and that honesty is half the lesson of this page — the map only
                        claims what beams actually reached.
                    </p>
                </>}
                ko={<>
                    <p>
                        셀 하나는 이진 확률변수다: 점유 또는 자유. 앞 페이지의 일반 재귀가 각각에 적용되고, 독립이
                        모든 축소를 대신한다 — 다른 셀은 전부 주변부로 사라지고, 정적인 지도는 운동 항을 아예 죽인다:
                    </p>
                    <BlockMath math="bel_t(m) = \eta\; p(z_t \mid m)\; bel_{t-1}(m)"/>
                    <p>
                        매 스텝 정규화를 동반하는 우도의 곱 — 정확히 로그 공간이 덧셈으로 바꾸는 것이다. odds{" "}
                        <InlineMath math="o = bel/(1-bel)"/>와 그 로그, 즉 <strong>log-odds</strong>{" "}
                        <InlineMath math="l = \log o"/>를 정의하면 관측마다 덧셈 한 번이 된다(증명은 두 줄, 아래에서
                        전개). prior는 <InlineMath math="bel = 0.5"/>, 즉 <InlineMath math="l = 0"/>: 모든 셀은
                        중립에서 시작하고, 셀은 <InlineMath math="l > 0"/>일 때만 점유로 취급된다.{" "}
                        <strong>미지는 점유가 아니다</strong>: 한 번도 맞지 않은 셀은 prior를 영원히 지키고, 그
                        정직함이 이 페이지 교훈의 절반이다 — 지도는 빔이 실제로 닿은 것만 주장한다.
                    </p>
                </>}
            />
            <Proof title={t("Theorem (the update is an addition)", "정리 (갱신은 덧셈 한 번이다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> For a binary cell with the static transition above and
                            likelihood ratio{" "}
                            <InlineMath math="\Lambda_t = p(z_t \mid occupied)/p(z_t \mid free)"/>, the odds
                            update is exactly <InlineMath math="l_t = l_{t-1} + \log \Lambda_t"/>.
                        </p>
                        <p>
                            <strong>Proof.</strong> Write the recursion for both values of{" "}
                            <InlineMath math="m"/> and divide — the normalizer <InlineMath math="\eta"/> is the
                            same constant in both branches, so it cancels:
                        </p>
                        <BlockMath math="\frac{bel_t(1)}{bel_t(0)} = \frac{p(z_t \mid 1)}{p(z_t \mid 0)} \cdot \frac{bel_{t-1}(1)}{bel_{t-1}(0)},\qquad o_t = \Lambda_t\, o_{t-1}"/>
                        <p>
                            Take <InlineMath math="\log"/> of both sides:{" "}
                            <InlineMath math="l_t = l_{t-1} + \log \Lambda_t"/>. The renormalization never had to
                            be computed — that is the whole trick, and why nobody ships probability products
                            directly. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 위의 정적 전이를 가진 이진 셀과 우도비{" "}
                            <InlineMath math="\Lambda_t = p(z_t \mid occupied)/p(z_t \mid free)"/> 에 대해 odds 갱신은
                            정확히 <InlineMath math="l_t = l_{t-1} + \log \Lambda_t"/> 이다.
                        </p>
                        <p>
                            <strong>증명.</strong> <InlineMath math="m"/>의 두 값에 대해 재귀를 각각 쓰고 나누면 된다 —
                            정규화 인자 <InlineMath math="\eta"/>는 양쪽에서 같은 상수라서 약분된다:
                        </p>
                        <BlockMath math="\frac{bel_t(1)}{bel_t(0)} = \frac{p(z_t \mid 1)}{p(z_t \mid 0)} \cdot \frac{bel_{t-1}(1)}{bel_{t-1}(0)},\qquad o_t = \Lambda_t\, o_{t-1}"/>
                        <p>
                            양변에 <InlineMath math="\log"/>를 취하면{" "}
                            <InlineMath math="l_t = l_{t-1} + \log \Lambda_t"/>. 정규화를 애초에 계산할 필요가 없었다 —
                            이것이 트릭 전체이고, 아무도 확률 곱을 그대로 실어 나르지 않는 이유다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("A Beam Point Is Two Updates", "빔 점은 두 개의 갱신이다")}</h2>
            <T
                en={<>
                    <p>
                        What is the likelihood ratio of one scan point? The sensor says two things at once: the
                        cell where the beam <em>landed</em> was occupied (probability{" "}
                        <InlineMath math="p_{hit}"/>, it stopped there), and every cell the beam{" "}
                        <em>passed through</em> on the way was free (probability{" "}
                        <InlineMath math="p_{free}"/>). A missed beam emits no point at all, so it updates{" "}
                        <em>nothing</em> — misses are honest here; this model never pushes a cell toward free
                        because a beam flew past into the void.
                    </p>
                    <p>
                        The repo's inverse model is symmetric by contract:{" "}
                        <InlineMath math="p_{free} = p_{hit}"/>, so both increments carry one magnitude and the
                        whole algorithm fits in one addition per cell:
                    </p>
                    <BlockMath math="L = \log \frac{p_{hit}}{1 - p_{hit}},\qquad l \mathrel{+}= +L \text{ (landing)},\qquad l \mathrel{-}= L \text{ (pass-through)}"/>
                    <Terms items={[
                        ["p_{hit}", <>the symmetric inverse model's single probability — the config enforces{" "}
                            <InlineMath math="p_{hit} > 0.5"/>, because at 0.5 the log-odds is 0 and nothing ever updates</>],
                        ["L", <>one magnitude, two signs: landing adds it, passing subtracts it. Since every update is ±L with a single positive L,{" "}
                            <InlineMath math="\operatorname{sign}(l) = \operatorname{sign}(\#hits - \#passes)"/> — the binary map does not depend on{" "}
                            <InlineMath math="p_{hit}"/> at all, only the confidence behind each cell does</>],
                        ["noise", <>a noisy endpoint landing one cell short of the wall is not special-cased: that free cell collects −L from every beam passing through it toward real walls, so a few stray hits never flip it</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        점 하나의 우도는 무엇인가? 센서는 두 가지를 동시에 말한다: 빔이 <em>착지한</em> 셀은 점유였다
                        (확률 <InlineMath math="p_{hit}"/>, 거기서 멈췄다), 그리고 도중에 <em>통과한</em> 모든 셀은
                        자유였다(확률 <InlineMath math="p_{free}"/>). 맞지 않은 빔은 점을 아예 내지 않으므로{" "}
                        <em>아무것도</em> 갱신하지 않는다 — miss 는 여기서 정직하고, 이 모델은 빔이 허공으로 지나갔다고
                        셀을 자유 쪽으로 밀지 않는다.
                    </p>
                    <p>
                        이 저장소의 역모델은 계약상 대칭이다: <InlineMath math="p_{free} = p_{hit}"/> — 그래서 두
                        증분은 하나의 크기만 들고 알고리즘 전체가 셀마다 덧셈 한 번으로 줄어든다:
                    </p>
                    <BlockMath math="L = \log \frac{p_{hit}}{1 - p_{hit}},\qquad l \mathrel{+}= +L \text{ (landing)},\qquad l \mathrel{-}= L \text{ (pass-through)}"/>
                    <Terms items={[
                        ["p_{hit}", <>대칭 역모델의 단일 확률 — 설정은 <InlineMath math="p_{hit} > 0.5"/>를 강제한다; 0.5 이하면 log-odds 가 0 이 되어 아무것도 갱신되지 않는다</>],
                        ["L", <>하나의 크기, 두 부호: 착지는 더하고 통과가 뺀다. 모든 갱신이 하나의 양수 L 에 대해 ±L 이므로{" "}
                            <InlineMath math="\operatorname{sign}(l) = \operatorname{sign}(\#hits - \#passes)"/> — 이진 지도는{" "}
                            <InlineMath math="p_{hit}"/>에 전혀 의존하지 않고, 셀 뒤의 확신만 달라진다</>],
                        ["noise", <>벽보다 한 셀 손전에 착지하는 노이즈 섞인 끝점은 특별 취급하지 않는다: 그 자유 셀은 진짜 벽을 향해 지나가는 모든 빔에서 −L 을 수집하므로, 드문드문 맞는 hit 이 뒤집지 못한다</>],
                    ]}/>
                </>}
            />

            <h2>{t("The Ray Walks the Grid", "광선은 격자를 걸어간다")}</h2>
            <T
                en={<>
                    <p>
                        Turning a scan point into cells is the same DDA walk the simulator's raycast uses — but
                        run on the raw segment vector, not on an angle. Converting the endpoint back to an angle
                        and re-raycasting would quantize it onto beam centers and destroy the sub-cell landing
                        precision that makes wall faces sharp here. The rules are exactly the raycast's:{" "}
                        <InlineMath math="t_{max}"/> starts at the ray parameter of the next grid-line crossing
                        and advances by <InlineMath math="res/|d|"/>; an axis whose delta is 0 never steps; a
                        tie takes Y. Two edge cases are part of the contract: a segment that starts off-raster
                        updates nothing, and a walk that leaves the raster before reaching the endpoint's cell
                        hands the hit to its last in-bounds cell — beyond the raster no cell exists to update.
                    </p>
                </>}
                ko={<p>
                    스캔 점을 셀로 바꾸는 것은 시뮬레이터 raycast 와 같은 DDA 보행이다 — 다만 각도가 아니라 날것
                    세그먼트 벡터 위에서 한다. 착지점을 각도로 되돌려 다시 쏘면 빔 중심에 양자화되어 여기서 벽면을
                    선명하게 만드는 셀 미만 착지 정밀도를 망가뜨린다. 규칙은 raycast 와 정확히 같다:{" "}
                    <InlineMath math="t_{max}"/>는 다음 격자선 교차의 ray 파라미터에서 시작해{" "}
                    <InlineMath math="res/|d|"/>만큼 전진하고, delta 가 0인 축은 절대 스텝하지 않으며, 동률은 Y 를
                    택한다. 두 경계 케이스가 계약에 포함된다: 래스터 밖에서 시작하는 세그먼트는 아무것도 갱신하지
                    않고, 착지 셀에 닿기 전에 래스터를 벗어나는 보행은 마지막 경계 내 셀에 hit 을 넘긴다 — 래스터
                    바깥에는 갱신할 셀이 존재하지 않으니까.
                </p>}
            />

            <h2>{t("What No Beam Can Reach", "어떤 빔도 닿을 수 없는 곳")}</h2>
            <T
                en={<>
                    <p>
                        A beam stops at the first occupied cell it enters — that is what a scan point{" "}
                        <em>is</em>. So the interior of a thick wall, and any corner sealed on all sides, can
                        never accumulate evidence: those cells keep their prior <InlineMath math="l = 0"/> forever
                        and stay unknown. The demo's IoU sits near but below 1 for exactly this reason — an honest
                        ceiling, not a bug. A map that claims the inside of a wall it could never see would be
                        lying with confidence.
                    </p>
                    <p>
                        And the pose really is given here: the anchor equals the ground-truth start bit-for-bit
                        and the odometry noise is 0, so <InlineMath math="ate_{rmse} \approx 10^{-16}"/> — the
                        residual blur in the map comes only from range noise moving landings by a cell. What makes
                        a real mapping-while-localizing run smear walls — drifting poses feeding wrong segment
                        endpoints into the field — is exactly what the filter_based branch adds on top of this:
                        the same field, per particle, with the pose finally estimated.
                    </p>
                </>}
                ko={<>
                    <p>
                        빔은 들어간 첫 점유 셀에서 멈춘다 — 스캔 점이란 곧 그것이다. 그래서 두꺼운 벽의 내부와 사방이
                        막힌 모서리는 영원히 증거를 쌓을 수 없다: 그 셀들은 prior <InlineMath math="l = 0"/>을 평생
                        지키고 미지로 남는다. demo 의 IoU 가 1 근처에서 멈추는 이유가 정확히 이것이다 — 버그가 아니라
                        정직한 상한이다. 볼 수 없는 벽의 안쪽까지 주장하는 지도는 자신 있게 거짓말하는 지도다.
                    </p>
                    <p>
                        그리고 자세는 여기서 정말로 주어진다: 앵커가 정본 출발점과 비트 단위로 같고 오도메트리 노이즈가
                        0 이므로 <InlineMath math="ate_{rmse} \approx 10^{-16}"/> — 지도에 남는 흔적 흐림은 착지점이
                        셀 하나 밀리는 거리 노이즈에서만 온다. 진짜 매핑-while-국소화가 벽을 번지게 만드는 것, 즉
                        드리프트하는 자세가 틀린 세그먼트 끝점을 필드에 쏟아붓는 현상은 filter_based 갈래가 이 위에
                        얹는 것이다: 같은 필드를, 이번엔 입자마다, 그리고 마침내 추정된 자세와 함께.
                    </p>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The whole estimator is one loop with a fixed operation order — scan order, cells near→far per
                    walk, accumulation in arrival order, touched set emitted row-major sorted — pinned so two
                    languages reproduce it bit for bit.
                </p>}
                ko={<p>
                    추정기 전체는 고정 연산 순서를 가진 루프 하나다 — 스캔 순서, 보행당 근접→원거리 셀, 도착 순서
                    누적, touched 집합을 row-major 정렬로 방출 — 두 언어가 비트 단위로 재현하도록 못 박혀 있다.
                </p>}
            />
            <Pseudocode code={`init: l[cell] ← 0 for every cell                          # prior = probability 0.5 (unknown)

update(z₁ … z_K at pose x_t):                                   # t = 0: the declared anchor IS the pose
    pose ← anchor ⊕ odom                                        # noiseless integration — ATE ≈ 0 by contract
    for each point z in scan order:                             # beam ascending
        w ← robot_to_world(z, pose)                             # endpoint back to world frame
        cells ← walk(pose.point → w)                            # DDA below, near → far
        for every cell before the last:  l[cell] −= L           # pass-through → free evidence
        l[last cell] += L                                       # landing     → occupied evidence
    emit pose; emit touched cells row-major sorted

walk(a, b):                                                     # raycast's step rule on the raw segment
    (r,c) ← cell(a);  if off-raster: return []                  # nothing exists out there
    target ← cell(b);  cells ← [(r,c)]
    if (r,c) == target: return cells                            # degenerate — the start IS the hit
    tMax_x, tMax_y ← ray parameter of next grid-line crossing   # delta-0 axis never steps; tie → Y
    loop: step the smaller-tMax axis by one cell                # advance that tMax by res/|d|
          off-raster → return cells (last in-bounds cell absorbs the hit)
          on target  → return cells`}/>
            <T
                en={<ol>
                    <li><strong>Prior.</strong> Every cell starts at log-odds 0 — probability 0.5, and{" "}
                        <InlineMath math="l > 0"/> is the only occupancy decision there is.</li>
                    <li><strong>Pose is integrated, not estimated</strong>: t = 0 takes the declared anchor
                        (<InlineMath math="(x_0, y_0, \theta_0)"/>), later steps compose it with the scenario's
                        noiseless twist. This page is about the map; the pose chapter comes in filter_based.</li>
                    <li><strong>Per point</strong>: the endpoint goes back to world frame with the estimated pose
                        (the exact inverse of the simulator's world→robot), and the segment walk lists every cell
                        it crosses, nearest first.</li>
                    <li>The walk is the raycast's DDA on the raw vector — same tie rule (<InlineMath math="Y"/>
                        wins), an axis with delta 0 never steps, degenerate segments are their own hit.</li>
                    <li><strong>Emission</strong>: pose first, then only the cells whose log-odds changed this
                        step, sorted row-major — sparse by construction, identical order in every language.</li>
                </ol>}
                ko={<ol>
                    <li><strong>Prior.</strong> 모든 셀은 log-odds 0에서 시작한다 — 확률 0.5이고,{" "}
                        <InlineMath math="l > 0"/>이 존재하는 유일한 점유 판정이다.</li>
                    <li><strong>자세는 추정하지 않고 적분한다</strong>: t = 0은 선언된 앵커{" "}
                        (<InlineMath math="(x_0, y_0, \theta_0)"/>)를 쓰고, 이후 스텝은 시나리오의 무노이즈 twist로
                        합성한다. 이 페이지는 지도에 관한 페이지이고, 자세 이야기는 filter_based 에서 온다.</li>
                    <li><strong>점마다</strong>: 끝점을 추정 자세로 world 프레임에 되돌리고(시뮬레이터 world→robot 의
                        정확한 역), 세그먼트 보행이 지나는 모든 셀을 근접 순으로 나열한다.</li>
                    <li>보행은 날것 벡터 위의 raycast DDA 다 — 같은 동률 규칙(Y 우선), delta 가 0인 축은 절대
                        스텝하지 않는다, 퇴화 세그먼트는 자기 자신이 hit 이다.</li>
                    <li><strong>방출</strong>: 자세가 먼저, 그다음 이 스텝에 log-odds 가 바뀐 셀만, row-major
                        정렬로 — 구조적으로 희소하고 모든 언어에서 같은 순서다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox below runs this exact estimator live in your browser — the same operation
                        order as the repository's Python and C++ (the parity checker compares their metrics down
                        to <InlineMath math="10^{-9}"/>). Drag cells to draw walls: the scan changes and the
                        estimator re-runs from step 0. The preset is a zigzag-and-spiral tour of the office map
                        that sweeps every face from both directions; it starts facing due west, which is why the
                        anchor heading is exactly <InlineMath math="\pi"/>. Watch the two halves of the lesson:
                        the estimated trail rides ground truth to machine epsilon (<InlineMath math="ate_{rmse} \approx 10^{-16}"/>),
                        while the map fills in face by face — and the sealed interiors stay blank, which is why{" "}
                        <InlineMath math="map\_iou"/> lands near 0.93, not 1.
                    </p>
                    <p>
                        The only chip is <code>p_hit</code>. Try it and watch the metrics: they do not move at
                        all — with a symmetric model every update is ±L for one positive L, so the sign of each
                        cell depends only on (#hits − #passes). What the chip changes is the confidence behind
                        each cell, i.e. how saturated the heat map looks (<InlineMath math="\operatorname{sigmoid}(l)"/>,
                        where <InlineMath math="L = \log\frac{p}{1-p}"/> is 2.2 by default and the chip cycles it
                        through 4.6 and 0.4). The binary map is invariant; only the confidence saturates.
                    </p>
                </>}
                ko={<>
                    <p>
                        아래 sandbox 는 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++ 과 같은 연산
                        순서이고(parity 체커가 지표를 <InlineMath math="10^{-9}"/>까지 비교한다). 셀을 드래그해 벽을
                        그리면 스캔이 바뀌고 추정이 스텝 0에서 다시 돈다. 프리셋은 오피스 맵을 지그재그+나선으로 훑으며
                        모든 면을 양쪽에서 스캔하는 투어이고, 정확히 서쪽으로 향하며 시작하니 앵커 heading 이 정확히{" "}
                        <InlineMath math="\pi"/>다. 교훈의 두 절을 지켜보라: 추정 트레일은 기계 정밀도까지 정답을 타고
                        (<InlineMath math="ate_{rmse} \approx 10^{-16}"/>), 지도는 면별로 채워지는데 — 막힌 내부와
                        막힌 모서리는 계속 비어 있고, 그래서 <InlineMath math="map\_iou"/>가 1이 아니라 0.93 근처에
                        앉는다.
                    </p>
                    <p>
                        유일한 칩은 <code>p_hit</code> 이다. 만져보고 지표가 움직이지 않는 것을 봐라 — 대칭 모델에서는 모든
                        갱신이 하나의 양수 L 에 대해 ±L 이므로 셀의 부호는 (hit 수 − 통과 수)에만 의존한다. 칩이 바꾸는 것은
                        셀 뒤의 확신, 즉 히트맵이 얼마나 포화되어 보이는지다 — <InlineMath math="L = \log\frac{p}{1-p}"/>가
                        기본 2.2 에서 칩 사이클 순서로 4.6, 0.4 가 될 때의{" "}
                        <InlineMath math="\operatorname{sigmoid}(l)"/> 이다. 이진 지도는 불변이고, 포화되는 것은 확신뿐이다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "office01_tour"}]}
                run={runGridMapping}
                params={{p_hit: 0.9, x0: 5.875, y0: 4.375, theta_deg: 180}}
                chips={[{key: "p_hit", values: [0.6, 0.9, 0.99]}]}
                label={t(
                    "grid_mapping live — heat is sigmoid(log-odds); drag to draw walls and watch the tour fill them in",
                    "grid_mapping 라이브 — 열기는 sigmoid(log-odds); 드래그로 벽을 그리고 투어가 채워지는 것을 보라",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other
                    operation for operation: scan order, near→far walk, arrival-order accumulation, row-major
                    sorted emission, scalar <code>log</code> on both sides. The estimator emits one event stream{" "}
                    (<code>run_started</code>, <code>step_observed → pose_estimated → map_updated</code> per step,{" "}
                    <code>run_finished</code>) and the browser engine replays that contract with identical metrics
                    — which is what lets you play with the live sandbox above and trust it.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: 스캔 순서,
                    근접→원거리 보행, 도착 순서 누적, row-major 정렬 방출, 양쪽 다 스칼라 <code>log</code>.
                    추정기는 하나의 이벤트 스트림(<code>run_started</code>, 스텝마다{" "}
                    <code>step_observed → pose_estimated → map_updated</code>, <code>run_finished</code>)을
                    내보내고 브라우저 엔진은 그 계약을 같은 지표로 재생한다 — 그래서 위 라이브 sandbox 를 믿고
                    가지고 놀 수 있다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/filtering/grid_mapping.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/filtering/grid_mapping.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/filtering/grid_mapping.hpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/include/slam/filtering/grid_mapping.hpp`,
                            },
                            {
                                name: "cpp/src/filtering/grid_mapping.cpp",
                                code: cppSrc,
                                href: `${REPO}/blob/main/cpp/src/filtering/grid_mapping.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The estimator sources, embedded from the repository — the demo driver and trace contract live in demos/ and core/",
                    "저장소에서 그대로 embed 한 추정기 소스 — demo 드라이버와 trace 계약은 demos/와 core/에 있다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    H. P. Moravec,{" "}
                    <em>Artificial Landmarks for Mobile Robots: A Probabilistic Approach</em>, Proc. IEEE Int.
                    Conf. on Robotics and Automation (ICRA), 1988 — the original occupancy-grid idea: each cell
                    an independent binary variable updated by sonar returns.
                </li>
                <li>
                    A. Elfes,{" "}
                    <em>Occupancy Grids: A Stochastic Technique for Robot Perception and Navigation</em>,
                    Technical Report CMU-RI-TR-90-17, Carnegie Mellon University, 1989/1990 — the occupancy grid
                    as a robot perception representation, in the same per-cell independent form.
                </li>
                <li>
                    S. Thrun, W. Burgard, D. Fox,{" "}
                    <a href="https://mitpress.mit.edu/9780262201629/probabilistic-robotics" target="_blank"
                       rel="noopener noreferrer">
                        <em>Probabilistic Robotics</em>
                    </a>,
                    MIT Press, 2005 — Ch. 9: the log-odds form above and the generalized inverse sensor model
                    this page simplifies to its symmetric single-magnitude case.
                </li>
            </ol>
        </>
    )
}

export default GridMapping
