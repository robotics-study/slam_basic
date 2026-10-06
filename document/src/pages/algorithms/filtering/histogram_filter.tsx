import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import {SandboxScene} from "../../../components/panels/Sandbox";
import {runHistogramFilter} from "../../../libs/algorithms/histogram-filter";
import pyImpl from "../../../../../python/slam/filtering/histogram_filter.py?raw";
import cppImpl from "../../../../../cpp/include/slam/filtering/histogram_filter.hpp?raw";
import cppSrc from "../../../../../cpp/src/filtering/histogram_filter.cpp?raw";

const REPO = "https://github.com/robotics-study/slam_basic"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const HistogramFilter = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the Bayes filter with nothing hidden. The previous page ran the same
                    recursion on paper; here the belief stops being a symbol and becomes an actual
                    table: discretize the pose into cells × headings, carry one probability per
                    state, and update every single entry at every step. No sampling, no weights to
                    resample, no particles to lose — exhaustive enumeration, exact up to the
                    discretization itself. This is where the genealogy starts: Cowgill's 1970
                    grid-localization machine, later the textbook "histogram filter" of Thrun,
                    Burgard &amp; Fox. Everything that comes after in this branch relaxes one of its
                    assumptions and pays for it — particles drop the finite state space, landmark
                    filters drop the grid.
                </p>}
                ko={<p>
                    이것은 아무것도 숨긴 것이 없는 Bayes 필터다. 앞 페이지(재귀 Bayes 필터)가 같은
                    재귀를 종이 위에서 돌렸다면, 여기서는 belief가 기호가 아니라 실제 표(table)가
                    된다. 자세를 셀 × heading으로 이산화하고, 상태마다 확률 하나씩 올려 매 스텝
                    전부 갱신한다. 샘플링도, 리샘플링할 가중치도, 잃어버릴 입자도 없다 — 완전한
                    열거이고, 이산화 자체 외의 근사는 없다. 계보의 출발점이 여기다: Cowgill의 1970년
                    격자 국소화 기계가 나중에 Thrun·Burgard·Fox 교과서의 "histogram filter"가 되었다.
                    이 갈래에서 이후에 오는 것들은 전부 가정 하나를 완화하고 그 대가를 치른다 —
                    입자는 유한 상태 공간을 버리고, 랜드마크 필터는 격자를 버린다.
                </p>}
            />

            <h2>{t("From the Recursion to a Table", "재귀에서 표로")}</h2>
            <T
                en={<>
                    <p>
                        The state is a pose, but now discretized: position lives on the map's free
                        cells, and heading lives on an angular lattice whose step equals the
                        sensor's own beam spacing <InlineMath math="w"/>. A state is a pair{" "}
                        <InlineMath math="s = (f, b)"/> — free cell <InlineMath math="f"/>, lattice
                        angle <InlineMath math="\theta_b = -\pi + b\,w"/> — and the belief is a
                        vector with one number per state. Occupied cells are not states at all:
                        their mass is exactly 0 forever, which is why free-cell indexing exists.
                    </p>
                    <BlockMath math="bel(x_t) = \eta\; p(z_t \mid x_t)\!\sum_{s' \in S} p(x_t \mid u_t, s')\, bel(s')"/>
                    <p>
                        The integral of the general recursion became this finite sum — that is the
                        whole trick, and it buys exactness. <strong>Predict</strong> pushes every
                        state's mass through the commanded twist: each hypothesis at{" "}
                        <InlineMath math="(f, \theta_b)"/> composes with <InlineMath math="u"/>, gets
                        quantized back onto the lattice (nearest center, ties up), and is mixed with a{" "}
                        <InlineMath math="p_{\text{slip}}"/> chance of having stayed put. That single
                        scalar is the entire motion uncertainty in this model — odometry noise below
                        one cell cannot cross a cell boundary, so it is literally invisible to a
                        discrete state. <strong>Update</strong> multiplies the prior by the
                        likelihood and renormalizes; doing that product in log space (subtracting{" "}
                        <InlineMath math="m = \max_s L(s)"/> before exponentiating) keeps 45 scans'
                        worth of multiplied probabilities from underflowing.
                    </p>
                </>}
                ko={<>
                    <p>
                        상태는 자세지만, 이제 이산화된다: 위치는 맵의 자유 셀 위에, heading은 센서
                        빔 간격과 같은 눈금 <InlineMath math="w"/>를 가진 각도 격자 위에 산다. 상태는{" "}
                        <InlineMath math="s = (f, b)"/> 쌍 — 자유 셀 <InlineMath math="f"/>, 격자 각도{" "}
                        <InlineMath math="\theta_b = -\pi + b\,w"/> — 그리고 belief는 상태마다 숫자
                        하나를 담은 벡터다. 점유 셀은 아예 상태가 아니다: 질량은 영원히 정확히 0이고,
                        그래서 자유 셀 인덱싱이 존재한다.
                    </p>
                    <BlockMath math="bel(x_t) = \eta\; p(z_t \mid x_t)\!\sum_{s' \in S} p(x_t \mid u_t, s')\, bel(s')"/>
                    <p>
                        일반 재귀의 적분이 이 유한 합으로 바뀌었다 — 이것이 트릭 전체이고, 정확함을
                        산다. <strong>Predict</strong>는 각 상태의 질량을 명령된 twist로 밀어 넣는다:{" "}
                        <InlineMath math="(f, \theta_b)"/>의 가설이 <InlineMath math="u"/>와 합성되고,
                        격자로 되돌아 양자화되며(최근접 중심, 동률은 올림),{" "}
                        <InlineMath math="p_{\text{slip}}"/>만큼 제자리에 남을 확률과 섞인다. 이 스칼라
                        하나가 이 모델의 불확실성 전부다 — 셀 크기 미만인 오도메트리 노이즈는 셀 경계를
                        건널 수 없으므로 이산 상태에는 문자 그대로 보이지 않는다. <strong>Update</strong>는
                        prior에 우도를 곱해 정규화하고, 그 곱셈을 로그 공간에서 한다(거듭제곱 전{" "}
                        <InlineMath math="m = \max_s L(s)"/>를 빼서) — 45번 스캔이 쌓여도 곱해진 확률이
                        언더플로하지 않게 한다.
                    </p>
                </>}
            />

            <h2>{t("A Beam Scan as a Likelihood", "빔 스캔을 우도로")}</h2>
            <T
                en={<>
                    <p>
                        The sensor fires beams and returns hit endpoints in the robot frame. What is
                        the likelihood of one point <InlineMath math="z = (r, \beta)"/> given a state?
                        Quantize its bearing with the same rule the sensor used,{" "}
                        <InlineMath math="j = \mathrm{round}((\beta + \tfrac{fov}{2})/w)"/>, and ask:
                        if the robot really were at state <InlineMath math="s"/> facing beam{" "}
                        <InlineMath math="j"/>, what range would come back? That expected range is a
                        table lookup — one DDA raycast per (free cell, lattice angle), cached once at
                        init:
                    </p>
                    <BlockMath math="H[j][s] = R\big[f(s)\,\big]\big[(b + j - off) \bmod B\big],\qquad p(z \mid s) = \mathcal{N}(r;\, H[j][s],\, \sigma^2)"/>
                    <Terms items={[
                        ["R[f][k]", <>the precomputed table: raycast from cell center <InlineMath math="f"/>'s center at lattice angle <InlineMath math="\theta_k"/>; a miss stores the sentinel <InlineMath math="range_{max} + res"/></>],
                        ["B,\\ w,\\ off", <>lattice size <InlineMath math="B = 2\pi/w"/> and offset <InlineMath math="off\,w = fov/2"/> — the lattice is the beam grid extended to a full circle</>],
                        ["(b + j - off) \\bmod B", <>rotation as index arithmetic: facing heading <InlineMath math="b"/>, beam <InlineMath math="j"/> reads the table entry taken at absolute angle <InlineMath math="\theta_b + \text{beam angle}"/></>],
                    ]}/>
                    <p>
                        Building <InlineMath math="H"/> is data movement, not geometry — it is the same{" "}
                        <InlineMath math="R"/> gathered through a rotation identity. And a missed beam
                        carries no point at all: misses are honest here, contributing nothing to{" "}
                        <InlineMath math="L"/>, while a state that predicts a hit where the ground truth
                        missed eats the full Gaussian penalty of the sentinel value. That asymmetry is
                        what discriminates between hypotheses.
                    </p>
                    <p>
                        Why does the heading lattice use the sensor's own angular step? So the true
                        pose is exactly representable: when the robot faces a lattice angle standing at
                        a cell center, its beams fire along table angles and the likelihood reduces to
                        pure range noise. A coarse uniform heading grid cannot do this — quantizing{" "}
                        <InlineMath math="\theta"/> by 20° flips grazing beams against any range
                        precision, and no Bayes update recovers a likelihood from that error. That is why
                        the sensor parameters are not knobs here: the injection contract fills beams,
                        fov_deg, range_max and sigma_range into the filter FROM the scenario — the filter
                        never sees a grid different from the one that produced its scans.
                    </p>
                </>}
                ko={<>
                    <p>
                        센서는 빔을 쏘고 맞은 끝점을 로봇 프레임으로 돌려준다. 상태가 주어졌을 때 점{" "}
                        <InlineMath math="z = (r, \beta)"/> 하나의 우도는 무엇인가? 방위각을 센서가 쓴
                        것과 같은 규칙으로 양자화하고,{" "}
                        <InlineMath math="j = \mathrm{round}((\beta + \tfrac{fov}{2})/w)"/>, 묻는다:
                        로봇이 정말 상태 <InlineMath math="s"/>에서 빔 <InlineMath math="j"/>를 향해
                        서 있다면 거리가 뭐가 돌아올까? 그 기댓값은 표 뒤지기다 — (자유 셀, 격자 각도)
                       마다 DDA 광선 하나씩, init에 한 번 캐시한다:
                    </p>
                    <BlockMath math="H[j][s] = R\big[f(s)\,\big]\big[(b + j - off) \bmod B\big],\qquad p(z \mid s) = \mathcal{N}(r;\, H[j][s],\, \sigma^2)"/>
                    <Terms items={[
                        ["R[f][k]", <>미계산 표: 셀 중심에서 격자 각도 <InlineMath math="\theta_k"/>로 raycast; miss는 sentinel <InlineMath math="range_{max} + res"/>를 저장</>],
                        ["B,\\ w,\\ off", <>격자 크기 <InlineMath math="B = 2\pi/w"/>와 오프셋 <InlineMath math="off\,w = fov/2"/> — 격자는 빔 눈금을 전체 원으로 확장한 것이다</>],
                        ["(b + j - off) \\bmod B", <>회전이 인덱스 산술이 된다: heading <InlineMath math="b"/>를 보면 빔 <InlineMath math="j"/>는 절대각 <InlineMath math="\theta_b + \text{beam angle}"/>에서 뽑은 표 항목을 읽는다</>],
                    ]}/>
                    <p>
                        <InlineMath math="H"/>를 만드는 것은 기하가 아니라 데이터 이동이다 — 같은{" "}
                        <InlineMath math="R"/>을 회전 항등식으로 모아 다시 담았을 뿐. 그리고 맞지 않은
                        빔은 점을 아예 내지 않는다: miss는 여기서 정직해서 <InlineMath math="L"/>에 아무것도
                        보태지 않지만, ground truth가 놓친 곳에서 맞을 것이라 예측한 상태는 sentinel 값의
                        가우시안 벌점을 전부 뒤집어쓴다. 가설들을 갈라놓는 것이 바로 그 비대칭이다.
                    </p>
                    <p>
                        heading 격자가 왜 센서의 각도 눈금을 그대로 쓰나? 정본 자세가 정확히 표현 가능하게
                        만들려고다. 로봇이 셀 중심에 서서 격자 각도를 향하면 빔은 표의 각도를 따라 쏘고,
                        우도는 순수한 거리 노이즈로 줄어든다. 굵게 균일하게 쪼갠 heading 격자는 이게 안 된다 —{" "}
                        <InlineMath math="\theta"/>를 20°로 양자화하면 스치듯 맞는 빔들이 어떤 거리 정밀도에도
                        뒤집히고, 그 오차에서 우도를 되살리는 Bayes 갱신은 없다. 그래서 센서 파라미터는 여기서
                        노브가 아니다: 주입 계약이 beams/fov_deg/range_max/sigma_range 를 시나리오 자체로 필터에
                        채운다 — 필터는 자기 스캔을 만든 격자와 다른 격자를 결코 보지 않으며, 이 항등을 망칠 수 있는
                        칩은 애초에 존재하지 않는다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Exact within the discretization.</strong> The recursion below is
                            not an approximation of a continuous filter — it <em>is</em> the exact
                            posterior of the discrete-state model, and with a lattice-matched scenario
                            the estimate sits at machine epsilon from ground truth (the demo's{" "}
                            <InlineMath math="ate_{rmse} \approx 10^{-16}"/>). What is lost is entirely in
                            the state space, not in the inference.</li>
                        <li><strong>Deterministic and reproducible.</strong> No random numbers anywhere;
                            fixed operation order makes Python and C++ agree bit for bit.</li>
                        <li><strong>Cost</strong>: with <InlineMath math="|S| = N_{free} \cdot B"/> states,
                        memory is <InlineMath math="O(|S|)"/>, predict is <InlineMath math="O(|S|)"/> per
                            step, update is <InlineMath math="O(K \cdot |S|)"/> for a scan of{" "}
                            <InlineMath math="K"/> points. On the demo map that is tens of thousands of
                            states × 361 table entries — already heavy.</li>
                        <li><strong>The wall</strong>: state-space size grows exponentially with dimension.
                            A 2D pose on a coarse grid fills this much memory; add one landmark and the
                            product explodes. That is exactly why the genealogy moves next to sampling
                            (particle filter) and to factored representations — not because the histogram
                            filter is wrong, but because it is exact at an unaffordable price in higher
                            dimensions.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>이산화 범위 안에서는 정확하다.</strong> 아래의 재귀는 연속 필터의
                            근사가 아니다 — 이산 상태 모델의 정확한 사후분포 그 자체다. 격자 일치 시나리오에서
                            추정은 기계 정밀도까지 ground truth에 붙는다 (demo의{" "}
                            <InlineMath math="ate_{rmse} \approx 10^{-16}"/>). 잃는 것은 전부 상태 공간 쪽에
                            있고, 추론 쪽에 있지 않다.</li>
                        <li><strong>결정적이고 재현 가능하다.</strong> 어디에도 난수가 없고, 고정된 연산
                            순서로 Python과 C++가 비트 단위로 일치한다.</li>
                        <li><strong>비용</strong>: <InlineMath math="|S| = N_{free} \cdot B"/>개 상태에
                            메모리 <InlineMath math="O(|S|)"/>, predict 스텝당 <InlineMath math="O(|S|)"/>,
                            update는 <InlineMath math="K"/>점 스캔에 <InlineMath math="K \cdot |S|"/>. demo
                            맵에서 이미 수만 상태 × 361개 표 항목 — 충분히 무겁다.</li>
                        <li><strong>벽</strong>: 상태 공간 크기는 차원에 대해 지수 폭발한다. 굵은 격자의 2D
                            자세만으로도 이 정도 메모리이고, 랜드마크를 하나만 늘려도 곱이 터진다. 그래서
                            계보가 다음에 샘플링(입자 필터)과 분해된 표현으로 넘어가는 것이다 — histogram
                            filter가 틀려서가 아니라, 높은 차원에서 정확함의 가격이 감당 불가능해서다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The whole filter is four fixed-order passes. Everything about the order below —
                    observation-major accumulation, ascending state sums, first-max argmax — is pinned
                    so that two languages can reproduce it bit for bit.
                </p>}
                ko={<p>
                    필터 전체는 고정 순서를 가진 통과정이다. 아래 순서에 대한 모든 것 — 관측-major 누적,
                    상태 승순 합산, 첫 최대 우선 argmax — 두 언어가 비트 단위로 재현할 수 있도록 못 박혀 있다.
                </p>}
            />
            <Pseudocode code={`init:  for every free cell f, lattice angle θ_k = −π + k·w:                 # 1
           R[f][k] ← raycast(center(f), θ_k)      miss → range_max + res
           bel[s] ← 1/|S| for all s                          uniform prior

predict(u):                                                                # 2
    for every state s = (f, b) with bel[s] ≠ 0:
        q ← quantize( compose((center(f), θ_b), u) )                       # 3
        new[q] += (1 − p_slip) · bel[s]     occupied/off-grid target → no move
        new[s] += p_slip · bel[s]
    bel ← new

update(z₁ … z_K):                                                          # 4
    for every point (r, β) of the scan, in order:
        j ← clamp(round((β + half)/w), 0, beams − 1)
        L[s] += −½ · ((r − H[j][s])/σ)²      for every state s             # 5
    m ← max_s L(s);   bel′(s) ∝ bel(s) · exp(L(s) − m)                     # 6

readout: x̂ = Σ_s bel(s)·center(f(s))                                       # 7
         θ̂ = lattice angle of the largest heading-bin marginal (first max wins)
         σ's = population standard deviations over the belief`}/>
            <T
                en={<ol>
                    <li><strong>Init.</strong> One DDA raycast per free-cell center and lattice angle
                        builds <InlineMath math="R"/>; the prior is uniform over all{" "}
                        <InlineMath math="N_{free} \cdot B"/> states. The config must keep the lattice
                        aligned (<InlineMath math="B\,w = 2\pi"/>, <InlineMath math="off\,w = fov/2"/>);
                        the code asserts it.</li>
                    <li><strong>Predict is child-of-movement</strong>: iterate states in index order and
                        push each mass forward. No inverse motion model needed — every state's child is
                        computed directly.</li>
                    <li>A move whose target cell is occupied or off-grid does not move (slide semantics);
                        the heading still rotates. The slip branch keeps the recursion a mixture instead
                        of a pure permutation.</li>
                    <li><strong>Update</strong>: each observed point in scan order recovers its beam index{" "}
                        <InlineMath math="j"/> from the bearing and adds its Gaussian log-likelihood against
                        every state at once.</li>
                    <li>The accumulation is observation-major, state-minor — the same summation order in
                        every language, so floating-point sums agree bit for bit.</li>
                    <li>Normalize by subtracting the max before exponentiating. If everything underflows to
                        zero (belief collapsed on a state the scan contradicts), the belief stays unchanged —
                        a documented degenerate case, identical in C++.</li>
                    <li><strong>Readout</strong>: position is the belief-weighted mean of cell centers;
                        heading is the marginal argmax with first-max-wins tie-breaking; the reported
                        standard deviations are population std devs over the same table.</li>
                </ol>}
                ko={<ol>
                    <li><strong>Init.</strong> 자유 셀 중심 × 격자 각도마다 DDA raycast로{" "}
                        <InlineMath math="R"/>을 만들고, prior는 <InlineMath math="N_{free} \cdot B"/>개
                        상태에 균일하다. 설정은 격자 정렬(<InlineMath math="B\,w = 2\pi"/>,{" "}
                        <InlineMath math="off\,w = fov/2"/>)을 유지해야 하고 코드는 이를 assert로 검사한다.</li>
                    <li><strong>Predict는 child-of-movement</strong>: 상태를 인덱스 순서로 돌며 각 질량을
                        앞으로 민다. 역운동 모델은 필요 없다 — 모든 상태의 자식은 직접 계산된다.</li>
                    <li>목적지 셀이 점유 셀이거나 격자 밖인 이동은 움직이지 않는다(slide semantics); 단
                        heading은 계속 회전한다. slip 갈래가 재귀를 순수 순열이 아니라 mixture로 유지시킨다.</li>
                    <li><strong>Update</strong>: 스캔의 각 점을 순서대로, 방위각에서 빔 인덱스{" "}
                        <InlineMath math="j"/>를 되찾고 가우시안 로그 우도를 전 상태에 한 번에 더한다.</li>
                    <li>누적은 관측-major/state-minor — 모든 언어에서 같은 합산 순서이므로 부동소수 합도
                        비트 단위로 일치한다.</li>
                    <li>거듭제곱 전 max를 빼서 정규화한다. 전부 0으로 언더플로하면(belief가 스캔과 모순되는
                        상태에 주저앉은 경우) belief는 그대로 유지된다 — 문서화된 퇴화 케이스이고 C++에서도 동일하다.</li>
                    <li><strong>Readout</strong>: 위치는 셀 중심의 belief 가중 평균, heading은 첫 최대 우선
                        argmax의 빈 주변부 최댓값, 보고되는 표준편차는 같은 표 위 모수 표준편차다.</li>
                </ol>}
            />

            <h2>{t("Exact by Construction", "구성으로 정확하다")}</h2>
            <T
                en={<p>
                    Two claims make this filter exact within its own model — which is what lets the first
                    demo below converge to machine epsilon and the second re-converge exactly once its
                    ambiguity resolves. The first claim: the recursion computes the true posterior of the
                    discrete model, not an approximation of one. The second: why the discrete model equals
                    reality here — with lattice-aligned headings and sub-cell odometry noise, quantization
                    self-heals. Both proofs are short; expand them if you want the details.
                </p>}
                ko={<p>
                    이 필터를 자기 모델 안에서 정확하다게 만드는 두 명제다 — 아래 첫 demo가 기계 정밀도까지
                    수렴하는 이유이고, 두 번째 demo가 모호성 해소 후 다시 정확히 수렴하는 이유다. 첫째, 재귀는
                    이산 모델의 참 사후분포를 계산한다 — 근사의 근사가 아니다. 둘째, 왜 여기서 이산 모델이
                    현실과 같은가: 격자 일치 heading에 셀 크기 미만 오도메트리 노이즈라면 양자화는 스스로
                    치유된다. 둘 다 증명은 짧고, 자세히 보고 싶으면 펼쳐 보라.
                </p>}
            />
            <Proof title={t("Theorem (the recursion is the posterior)", "정리 (재귀가 곧 사후분포다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If{" "}
                            <InlineMath math="bel_0"/> is uniform over{" "}
                            <InlineMath math="S = \{(\text{free cell}, \theta_b)\}"/> and every step applies
                            the predict/update pair above, then after step{" "}
                            <InlineMath math="t"/>, <InlineMath math="bel_t(s) = p(x_t = s \mid z_{1:t}, u_{1:t})"/>{" "}
                            exactly, for every state.
                        </p>
                        <p>
                            <strong>Proof by induction.</strong> At <InlineMath math="t = 0"/> both sides are{" "}
                            <InlineMath math="1/|S|"/>. Assume the claim at <InlineMath math="t-1"/>. Predict
                            distributes each state's mass over its children:
                        </p>
                        <BlockMath math="\tilde{bel}_t(q) = \sum_{s} p(x_t = q \mid u_t, x_{t-1} = s)\, bel_{t-1}(s)"/>
                        <p>
                            which is the law of total probability over a partition of the state space — it
                            equals <InlineMath math="p(x_t = q \mid z_{1:t-1}, u_{1:t})"/>. Update then multiplies
                            by the likelihood and renormalizes, which is Bayes' rule on a discrete space:
                        </p>
                        <BlockMath math="bel_t(q) = \frac{p(z_t \mid x_t = q)\;\tilde{bel}_t(q)}{\sum_{q'} p(z_t \mid x_t = q')\,\tilde{bel}_t(q')} = p(x_t = q \mid z_{1:t}, u_{1:t})"/>
                        <p>
                            The log-space normalization computes exactly this ratio (subtracting{" "}
                            <InlineMath math="m"/> cancels between numerator and denominator), so the
                            implementation equals the recursion, which equals the posterior.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> <InlineMath math="bel_0"/>이{" "}
                            <InlineMath math="S = \{(\text{자유 셀}, \theta_b)\}"/> 위 균일하고, 모든 스텝이
                            위의 predict/update 쌍을 적용하면, 스텝 <InlineMath math="t"/> 이후{" "}
                            <InlineMath math="bel_t(s) = p(x_t = s \mid z_{1:t}, u_{1:t})"/>가 모든 상태에
                            대해 정확히 성립한다.
                        </p>
                        <p>
                            <strong>귀납법 증명.</strong> <InlineMath math="t = 0"/>에서 양쪽 다{" "}
                            <InlineMath math="1/|S|"/>. <InlineMath math="t-1"/>에서 성립한다고 가정하자.
                            predict는 각 상태의 질량을 자식들에 분배한다:
                        </p>
                        <BlockMath math="\tilde{bel}_t(q) = \sum_{s} p(x_t = q \mid u_t, x_{t-1} = s)\, bel_{t-1}(s)"/>
                        <p>
                            이것은 상태 공간 분할에 대한 전확률 법칙이라{" "}
                            <InlineMath math="p(x_t = q \mid z_{1:t-1}, u_{1:t})"/>와 같다. 이어 update는 우도를
                            곱해 정규화하는데, 이는 이산 공간에서의 Bayes 공식 그 자체다:
                        </p>
                        <BlockMath math="bel_t(q) = \frac{p(z_t \mid x_t = q)\;\tilde{bel}_t(q)}{\sum_{q'} p(z_t \mid x_t = q')\,\tilde{bel}_t(q')} = p(x_t = q \mid z_{1:t}, u_{1:t})"/>
                        <p>
                            로그 공간 정규화는 정확히 이 비를 계산한다(<InlineMath math="m"/>을 빼는 것은 분자와
                            분모에서 상쇄된다). 따라서 구현 = 재귀 = 사후분포.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Lemma (quantization self-heals)", "보조정리 (양자화는 스스로 치유된다)")}>
                <T
                en={<>
                    <p>
                        The scenario places waypoints on cell centers, moves in exact lattice multiples, and
                        draws heading noise with <InlineMath math="\sigma_\theta = 0"/>; position noise stays
                        below one cell. Then the true state survives quantization exactly: a hypothesis sitting
                        at the true state, pushed through the commanded twist, lands back on the true state —
                        the position round-off cannot cross a boundary (noise {" "}
                        <InlineMath math="< res"/>) and the heading is already a lattice point. So the only
                        transition the filter can even see is slip, and the likelihood at the true state carries
                        nothing but range noise. Quantization error — the usual price of discretizing pose —
                        never enters the estimate at all. <InlineMath math="\blacksquare"/>
                    </p>
                </>}
                ko={<>
                    <p>
                        시나리오는 웨이포인트를 셀 중심에 놓고, 격자 정수배만큼만 움직이고, heading 노이즈는{" "}
                        <InlineMath math="\sigma_\theta = 0"/>으로 그린다; 위치 노이즈는 셀 하나 미만이다. 그러면
                        참 상태는 양자화를 정확히 살아남는다: 참 상태에 앉은 가설이 명령된 twist로 밀려나면 다시
                        참 상태로 돌아온다 — 위치 반올림은 경계를 넘을 수 없고(노이즈{" "}
                        <InlineMath math="< res"/>), heading은 이미 격자점이다. 따라서 필터가 볼 수 있는 전이는 slip
                        하나이고, 참 상태의 우도에는 거리 노이즈 외엔 아무것도 남지 않는다. 이산화의 단골 대가인
                        양자화 오차는 추정에 아예 들어오지 못한다. <InlineMath math="\blacksquare"/>
                    </p>
                </>}
            />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox below runs this exact filter live in your browser — the same operation order
                        as the repository's Python and C++ (the parity checker compares their metrics down to{" "}
                        <InlineMath math="10^{-9}"/>). Drag cells to draw walls: the scan changes and the estimator
                        re-runs from step 0. Two presets are the two halves of this branch:
                    </p>
                    <ul>
                        <li><code>corridor01_back_and_forth</code> — the exactness showcase. Unevenly spaced
                            pillars make every position's signature unique, so one scan collapses the uniform
                            prior onto the true cell and the estimate rides ground truth to machine epsilon
                            (<InlineMath math="ate_{rmse} \approx 10^{-16}"/>).</li>
                        <li><code>corridor02_ambiguous</code> — global localization done honestly. Four identical
                            baffles spaced exactly 4 cells apart, and a 2.5 m range that never reaches the
                            second-next face: three positions share one signature, so step 0 leaves exactly three
                            equally-massed hypothesis columns (times heading twins) and the readout is their{" "}
                            <em>mean</em>, not ground truth. The ambiguity persists while the periodicity holds,
                            dies when asymmetric structure enters range, and blooms again in the open tail where
                            nothing fits inside <InlineMath math="range_{max}"/> at all.</li>
                    </ul>
                    <p>
                        The only chip is <code>p_slip</code> — the whole motion uncertainty, and on the second
                        preset it finally becomes visible: at 0 a collapsed belief stays collapsed, while at 0.1
                        slip keeps smearing mass onto neighbouring cells so the blind zone re-spreads the belief
                        and killing it again takes longer. The sensor parameters are deliberately not chips —
                        injection pins them to the scenario (above), so no chip can break the lattice = sensor
                        identity here; a chip that could only ever desync the two would be a lie. The metrics
                        under the player are computed from the same events you are watching.
                    </p>
                </>}
                ko={<>
                    <p>
                        아래 sandbox는 이 필터를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++과 같은 연산
                        순서이고(parity 체커가 지표를 <InlineMath math="10^{-9}"/>까지 비교한다). 셀을 드래그해
                        벽을 그리면 스캔이 바뀌고 추정이 스텝 0에서 다시 돈다. 두 프리셋은 이 갈래의 두 절이다:
                    </p>
                    <ul>
                        <li><code>corridor01_back_and_forth</code> — 정확성 쇼케이스. 균일하지 않게 배치된 기둥이
                            위치마다 서명을 유일하게 만들므로, 첫 스캔 하나가 균일 prior를 참 셀로 붕괴시키고
                            추정이 기계 정밀도까지 정답을 탄다(<InlineMath math="ate_{rmse} \approx 10^{-16}"/>).</li>
                        <li><code>corridor02_ambiguous</code> — 정직한 전역 국소화. 똑같은 차폐막 4개가 정확히
                            4셀 간격으로 서 있고 2.5 m 사정거리는 다다음 면까지 닿지 않는다 — 세 위치가 하나의
                            서명을 공유하므로 스텝 0에는 질량이 같은 가설 열 셋(× heading 쌍둥이)이 남고 읽기값은
                            정답이 아니라 그 <em>평균</em>이다. 주기성이 유지되는 동안 모호성은 지속되고, 비대칭
                            구조가 사거리에 들어올 때 죽으며, 아무것도 사정거리 안에 없는 열린 테일에서 다시 번진다.</li>
                    </ul>
                    <p>
                        유일한 칩은 <code>p_slip</code> — 운동 불확실성의 전부이고, 두 번째 프리셋에서야 비로소
                        눈에 보인다: 0에서는 무너진 belief가 그대로 유지되고, 0.1에서는 slip이 이웃 셀로 질량을 계속
                        번지게 해서 맹지에서 belief가 다시 퍼지고 죽이는 데도 시간이 걸린다. 센서 파라미터는
                        의도적으로 칩이 아니다 — 주입이 시나리오에 고정하므로(위 참조) 둘을 어긋나게 만들 수 있는
                        칩은 애초에 거짓말일 것이다. 플레이어 아래 지표도 지금 보는 이벤트에서 계산된다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "corridor01_back_and_forth"}, {name: "corridor02_ambiguous"}]}
                run={runHistogramFilter}
                params={{p_slip: 0.1, beams: 361, fov_deg: 360, range_max: 6, sigma_range: 0.05}}
                chips={[{key: "p_slip", values: [0, 0.1, 0.5]}]}
                label={t(
                    "Histogram filter live — heat is the per-cell belief marginal; drag to draw walls",
                    "histogram filter 라이브 — 열기는 셀별 belief 주변부다; 드래그로 벽을 그려라",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other
                    operation for operation: observation-major/state-minor accumulation, ascending sums, scalar{" "}
                    <code>math.exp</code> per state (numpy only for exact elementwise operations), first-max-wins
                    argmax. The estimator emits one event stream (<code>run_started</code>,{" "}
                    <code>step_observed → pose_estimated → belief_updated</code> per step,{" "}
                    <code>run_finished</code>) and the browser engine replays that contract with identical
                    metrics — which is what lets you play with the live sandbox above and trust it.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다:
                    관측-major/상태-minor 누적, 승순 합산, 상태마다 스칼라 <code>math.exp</code>(numpy는 정확한
                    elementwise 연산에만), 첫 최대 우선 argmax. 추정기는 하나의 이벤트 스트림(<code>run_started</code>,
                    스텝마다 <code>step_observed → pose_estimated → belief_updated</code>, <code>run_finished</code>)을
                    내보내고, 브라우저 엔진은 그 계약을 같은 지표로 재생한다 — 그래서 위 라이브 sandbox를 믿고
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
                                name: "python/slam/filtering/histogram_filter.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/filtering/histogram_filter.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/filtering/histogram_filter.hpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/include/slam/filtering/histogram_filter.hpp`,
                            },
                            {
                                name: "cpp/src/filtering/histogram_filter.cpp",
                                code: cppSrc,
                                href: `${REPO}/blob/main/cpp/src/filtering/histogram_filter.cpp`,
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
                    C. L. Cowgill,{" "}
                    <em>Decision-Making Logic in a Mobile Robot</em>, Sc.D. thesis, Massachusetts Institute of
                    Technology, 1970 — the original grid-localization machine: a histogram over quantized poses.
                </li>
                <li>
                    S. Thrun, W. Burgard, D. Fox,{" "}
                    <a href="https://mitpress.mit.edu/9780262201629/probabilistic-robotics" target="_blank"
                       rel="noopener noreferrer">
                        <em>Probabilistic Robotics</em>
                    </a>,
                    MIT Press, 2005 — the recursive Bayes filter (Ch. 2–3) in exactly this grid form.
                </li>
            </ol>
        </>
    )
}

export default HistogramFilter
