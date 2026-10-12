import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import {SandboxScene} from "../../../components/panels/Sandbox";
import {runNdt} from "../../../libs/algorithms/ndt";
import pyImpl from "../../../../../python/slam/registration/ndt.py?raw";
import cppImpl from "../../../../../cpp/src/registration/ndt.cpp?raw";
import cppHeader from "../../../../../cpp/include/slam/registration/ndt.hpp?raw";

const REPO = "https://github.com/robotics-study/slam_basic"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const NdtPage = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the registration branch's second member, and it takes the branch's other half of the idea.
                    ICP matched point to point — a correspondence per source point, truncated when no twin exists. The
                    Normal Distributions Transform (Biber &amp; Strasser, 2003) <strong>never matches anything at
                    all</strong>. The reference scan is compiled into a field: every grid cell that caught enough points
                    stores one probability distribution — a mean and a scatter matrix — and the arriving scan is scored
                    against that field directly. Correspondence is not computed with a nearest-neighbor rule; it dissolves
                    into the field itself: a source point contributes a Gaussian bump centered on whatever cluster its
                    transformed position lands in, and the alignment is wherever the sum of bumps peaks. Like ICP, this
                    estimator reads no odometry — step 0 adopts the declared gauge pose and every later step composes one
                    recovered SE(2) element onto the running estimate.
                </p>}
                ko={<p>
                    이것은 registration 갈래의 두 번째 회원이고, 갈래의 다른 절반의 생각을 가져간다. ICP 는 점-점 대응을
                    만들었다 — 소스 점마다 대응 하나, 쌍둥이가 없으면 잘라 버린다. Normal Distributions Transform(Biber &amp;
                    Strasser, 2003)은 <strong>아무것도 짝짓지 않는다</strong>. 기준 스캔이 필드로 컴파일된다 — 점을 충분히
                    담은 격자 셀마다 확률 분포 하나, 평균과 산포 행렬을 저장하고, 도착한 스캔을 그 필드에 그대로 점수화한다.
                    대응은 최근접 규칙으로 계산되지 않고 필드 자체 속으로 녹아 들어간다: 소스 점은 변환된 위치가 닿는 클러스터의
                    가우스 bump 를 기여하고, 정렬은 bump들의 합이 피크를 이루는 자리다. ICP 처럼 이 추정기도 오도메트리를 읽지
                    않는다 — 스텝 0 은 선언된 게이지 포즈를 채택하고 이후 모든 스텝은 복원된 SE(2) 원소 하나를 추정 위에 합성한다.
                </p>}
            />

            <h2>{t("Correspondence, Dissolved Into a Field", "대응을 필드 속으로 녹이다")}</h2>
            <T
                en={<>
                    <p>
                        The field is built once per step from the target scan: points bucket into cells keyed by{" "}
                        <InlineMath math="(⌊x/c\rfloor, ⌊y/c\rfloor)"/> in scan order, a cell with fewer than three points carries
                        no distribution at all (a two-point cluster cannot span the plane), and one whose scatter determinant is
                        exactly zero — points perfectly collinear on an axis — carries none either. Every remaining cell{" "}
                        <InlineMath math="k"/> stores
                    </p>
                    <BlockMath math="\mu_k = \frac{1}{n}\sum_j x_j, \qquad S_k = \sum_j (x_j - \mu_k)(x_j - \mu_k)^{\mathsf T} \quad\text{(raw — deliberately not } S_k/n\text{)}"/>
                    <BlockMath math="\operatorname{score}(a) = \sum_i \exp\!\left(-\tfrac{1}{2}\, q_i^{\mathsf T} S_{k(i)}^{-1} q_i\right), \qquad q_i = R(a)\,x_i + a - \mu_{k(i)}"/>
                    <p>
                        The paper normalizes the scatter (<InlineMath math="\Sigma = S/n"/>); this repo does not, and that is an
                        honest design choice, not an oversight: <InlineMath math="\Sigma^{-1} = n\,S^{-1}"/> sharpens every bump by
                        the cluster's own point count, so a blob is as wide as the cluster that made it. Running the same scenario
                        with the normalized kernel measures worse — ATE 0.078 against 0.052, and step 2 overshoots to +0.365 against
                        a true +0.25. The scatter-scaled kernel is what captures a 0.25 m step cleanly. And note what the capture
                        range is here: not ICP's <InlineMath math="d_{\max}"/> cutoff — there is no cutoff at all, a point in an
                        empty cell simply contributes 0 — but the blob width itself. That makes the sensor noise constitutive:
                        re-run the scenario with <InlineMath math="\sigma_r = 0"/> and ATE gets <em>worse</em> (0.149 against
                        0.052), because points lying exactly on a face are perfectly collinear, their determinant is exactly zero,
                        and those cells carry nothing at all. The range noise σ is what gives every cluster its normal-direction
                        thickness — here it is part of the algorithm's input, not just a floor under its error.
                    </p>
                </>}
                ko={<>
                    <p>
                        필드는 스텝마다 target 스캔에서 한 번 만들어진다: 점은 스캔 순서로 <InlineMath math="(⌊x/c\rfloor, ⌊y/c\rfloor)"/>
                        키 버킷에 담기고, 점 3개 미만인 셀은 분포를 전혀 갖지 않고(점 두 개로는 평면을 못 잰다), 산포 행렬식이 정확히
                        0 — 축에 완벽히 공선인 점들 — 인 셀도 갖지 않는다. 남은 각 셀 <InlineMath math="k"/>는 이것을 저장한다:
                    </p>
                    <BlockMath math="\mu_k = \frac{1}{n}\sum_j x_j, \qquad S_k = \sum_j (x_j - \mu_k)(x_j - \mu_k)^{\mathsf T} \quad\text{(raw — deliberately not } S_k/n\text{)}"/>
                    <BlockMath math="\operatorname{score}(a) = \sum_i \exp\!\left(-\tfrac{1}{2}\, q_i^{\mathsf T} S_{k(i)}^{-1} q_i\right), \qquad q_i = R(a)\,x_i + a - \mu_{k(i)}"/>
                    <p>
                        논문은 산포를 정규화한다(<InlineMath math="\Sigma = S/n"/>); 이 저장소는 하지 않고, 그리고 그것은 정직한 설계
                        선택이지 실수가 아니다: <InlineMath math="\Sigma^{-1} = n\,S^{-1}"/>는 bump 전부를 클러스터 자신의 점 개수만큼
                        날카롭게 해서, blob 은 그것을 만든 클러스터만큼 넓어진다. 정규화된 커널로 같은 시나리오를 돌리면 실측은 더
                        나빠진다 — ATE 0.078 대 0.052, 스텝 2 는 참값 +0.25 에 대해 +0.365 로 오버슈트한다. 산포 스케일 커널이
                        0.25 m 스텝을 깔끔하게 잡는 것이다. 그리고 여기서 포획 범위가 무엇인지 주목하라: ICP 의
                        <InlineMath math="d_{\max}"/> 절단이 아니다 — 절단 자체는 전혀 없고, 빈 셀에 닿은 점은 그냥 0 을 기여할 뿐 —
                        blob 폭 그 자체다. 그래서 센서 노이즈가 구성 요소가 된다: 시나리오를 <InlineMath math="\sigma_r = 0"/>으로
                        다시 돌리면 ATE 는 오히려 <em>나빠진다</em>(0.149 대 0.052), 정확히 면 위에 놓인 점은 완벽히 공선이고 행렬식은
                        정확히 0 이고 그 셀들은 아무것도 담지 않으니까. 거리 노이즈 σ 가 모든 클러스터에 법선 방향 두께를 준다 —
                        여기서 그것은 알고리즘 입력의 일부이지, 단순한 오차 바닥이 아니다.
                    </p>
                </>}
            />
            <T
                en={<p>
                    A source point whose transformed position lands in an empty cell contributes exactly 0 — that is the whole
                    correspondence story now: no nearest neighbor, no truncation radius, just whether a bump exists where the point
                    landed. The score is smooth everywhere except where cells switch, and its peak is what the solver climbs to.
                </p>}
                ko={<p>
                    소스 점이 분포 없는 셀에 닿으면 기여는 정확히 0 — 대응의 전부가 이제 이것이다: 최근접도, 절단 반경도 없고, 오직
                    점이 착지한 자리에 bump 가 있는지뿐이다. 점수는 셀이 갈리는 자리를 빼면 어디든 매끄럽고, 그 피크로 솔버가 기어오른다.
                </p>}
            />

            <h2>{t("Newton on a Score Surface", "점수 표면 위의 뉴턴")}</h2>
            <T
                en={<>
                    <p>
                        Maximizing the score is minimizing <InlineMath math="f = -\operatorname{score}"/>, and this repo solves it
                        with damped Newton — no line search borrowed from a library, every operation in fixed order so all three
                        languages reproduce the same bits. With <InlineMath math="E = \exp(-\tfrac12 q^{\mathsf T}S^{-1}q)"/> and{" "}
                        <InlineMath math="u_j = (S^{-1}q)\cdot J_j"/> for <InlineMath math="J_1 = e_1"/>, <InlineMath math="J_2 = e_2"/>,{" "}
                        <InlineMath math="J_3 = \partial R(\varphi)x/\partial\varphi"/>, the gradient and Hessian accumulate in scan order:
                    </p>
                    <BlockMath math="g_j = \sum_i E_i\,u_j^{(i)}, \qquad H_{jk} = \sum_i E_i\left(-u_j u_k + W_{jk}\right)"/>
                    <p>
                        Far from a peak the surface is flat along the aliased axis and <InlineMath math="H"/> goes strongly indefinite —
                        measured at step 1's second iteration on the demo: eigenvalues <InlineMath math="-859"/> and{" "}
                        <InlineMath math="+70675"/>. The damping <InlineMath math="\lambda"/> starts at{" "}
                        <InlineMath math="\lambda_0 = 10^{-6}"/> and doubles while any Sylvester leading principal minor of{" "}
                        <InlineMath math="H + \lambda I"/> is ≤ 0 — measured: ×30 doublings to ≈ 1.07×10³ before the solve proceeds. That
                        is what this score surface actually looks like, and it is why the solver is damped Newton rather than plain
                        Newton. The step then passes a backtracking line search: Δ is accepted only on <em>strict</em> improvement,{" "}
                        <InlineMath math="f(a+\Delta) < f(a)"/>; otherwise it halves (exact in binary, so halving costs no rounding) until
                        it does — and if <InlineMath math="\max(\lvert\Delta dx\rvert,\lvert\Delta dy\rvert,\lvert\Delta d\theta\rvert) \le \varepsilon"/>
                        without any strict improvement, the solver returns the current estimate unchanged. Convergence here means a fixed
                        point of &quot;Newton then backtrack&quot;, not a guarantee — measured at 8/13/6/11 iterations per step in the demo,
                        never near the cap of 64.
                    </p>
                </>}
                ko={<>
                    <p>
                        점수 최대화는 <InlineMath math="f = -\operatorname{score}"/> 최소화이고, 이 저장소는 이를 감쇠 뉴턴으로 푼다 — 라이브러리
                       에서 빌려온 선 탐색 없이, 모든 연산이 고정 순서로, 그래서 세 언어가 같은 비트를 재현한다.{" "}
                        <InlineMath math="E = \exp(-\tfrac12 q^{\mathsf T}S^{-1}q)"/>와 <InlineMath math="u_j = (S^{-1}q)\cdot J_j"/>, 여기서{" "}
                        <InlineMath math="J_1 = e_1"/>, <InlineMath math="J_2 = e_2"/>, <InlineMath math="J_3 = \partial R(\varphi)x/\partial\varphi"/> —
                        기울기와 헤시안은 스캔 순서로 축적된다:
                    </p>
                    <BlockMath math="g_j = \sum_i E_i\,u_j^{(i)}, \qquad H_{jk} = \sum_i E_i\left(-u_j u_k + W_{jk}\right)"/>
                    <p>
                        피크에서 멀리 떨어진 곳에서 표면은 알리아싱된 축를 따라 평평해지고 <InlineMath math="H"/>는 강하게 부정정이 된다 —
                        실측: 데모의 스텝 1 두 번째 반복에서 고윳값 <InlineMath math="-859"/> 와 <InlineMath math="+70675"/>. 감쇠
                        <InlineMath math="\lambda"/>는 <InlineMath math="\lambda_0 = 10^{-6}"/>에서 시작해 <InlineMath math="H + \lambda I"/>의
                        실베스터 선행 주 소행렬식 하나가 ≤ 0 인 동안 배가된다 — 실측: ×30 배가로 ≈ 1.07×10³ 까지 간 뒤에야 솔버가 진행한다.
                        이것이 이 점수 표면의 실제 모습이고, 그래서 솔버이 평범한 뉴턴이 아니라 감쇠 뉴턴이다. 단계는 그다음 백트래킹 선
                        탐색을 지난다: Δ는 <em>엄격한</em> 개선 <InlineMath math="f(a+\Delta) < f(a)"/> 일 때만 수용되고, 아니면 그럴 때까지
                        절반 난다(이진에서 정확해서 절반 나누기에 반올림 비용 없음); 그리고 어떤 엄격한 개선도 없이
                        <InlineMath math="\max(\lvert\Delta dx\rvert,\lvert\Delta dy\rvert,\lvert\Delta d\theta\rvert) \le \varepsilon"/> 이면
                        솔버는 현재 추정을 그대로 돌려준다. 여기서 수렴은 &quot;뉴턴 다음 백트래킹&quot;의 고정점이고 보장이 아니다 — 실측:
                        데모에서 스텝마다 8/13/6/11 반복, 상한 64 에는 전혀 근접하지 않는다.
                    </p>
                </>}
            />
            <Proof title={t("Proposition (why the gradient is Σ E·u)", "명제 (왜 기울기가 Σ E·u 인가)")}>
                <T
                    en={<p>
                        One bump is <InlineMath math="E(q) = \exp(-\tfrac12 q^{\mathsf T}S^{-1}q)"/> with{" "}
                        <InlineMath math="q(a) = R(a)x + a - \mu"/>. Since <InlineMath math="S^{-1}"/> is symmetric,{" "}
                        <InlineMath math="\partial_q(\tfrac12 q^{\mathsf T}S^{-1}q) = S^{-1}q"/> and the chain rule gives{" "}
                        <InlineMath math="\nabla_a f_i = -E_i\,u_j \cdot(-1)"/> — collecting the sign into the definition,{" "}
                        <InlineMath math="g_j = \sum E_i u_j"/> is exactly <InlineMath math="\nabla f"/>. The Hessian is one more product
                        rule: <InlineMath math="\partial_k(E\,u_j) = (\partial_k E)u_j + E\,\partial_k u_j = -E\,u_k u_j + E\,W_{jk}"/> with{" "}
                        <InlineMath math="W_{jk} = J_k^{\mathsf T} S^{-1} J_j + (S^{-1}q)\cdot\partial J_j/\partial a_k"/> evaluated at the same
                        fixed point. Every sum runs in scan order, so the accumulation order is pinned like everything else.
                    </p>}
                    ko={<p>
                        bump 하나는 <InlineMath math="E(q) = \exp(-\tfrac12 q^{\mathsf T}S^{-1}q)"/>이고 <InlineMath math="q(a) = R(a)x + a - \mu"/>.{" "}
                        <InlineMath math="S^{-1}"/>는 대칭이므로 <InlineMath math="\partial_q(\tfrac12 q^{\mathsf T}S^{-1}q) = S^{-1}q"/>이고 연쇄법칙이{" "}
                        <InlineMath math="\nabla_a f_i"/>를 준다 — 부호를 정의에 흡수하면 <InlineMath math="g_j = \sum E_i u_j"/>가 정확히{" "}
                        <InlineMath math="\nabla f"/>. 헤시안은 곱의 미분을 한 번 더: <InlineMath math="\partial_k(E\,u_j) = -E\,u_k u_j + E\,W_{jk}"/>,
                        여기서 <InlineMath math="W_{jk}"/>는 위와 같은 고정점에서 평가된다. 모든 합은 스캔 순서로 돌고, 그래서 축적 순서도 다른 전부처럼 못 박혀 있다.
                    </p>}
                />
            </Proof>
            <T
                en={<>
                    <p>
                        What is a capture basin without correspondence? The unit tests pin the honest answer. One cluster of four points —
                        a cross with <InlineMath math="\sigma_{\text{eff}} = \sqrt{0.125} \approx 0.354"/> — and the source is that cross shifted
                        by −0.25 in x. A point-to-point rule would return +0.25 exactly. NDT cannot: its score is a sum of four bumps whose
                        centers sit closer together than the blob is wide, so the surface has several stationary points and Newton-with-backtracking
                        slides to the nearest one — pinned at dx = 0.4000…, not +0.25; even a single iteration already lands at 0.3933…. Soft capture,
                        honestly wrong when the geometry is degenerate, exactly like ICP's aliased lattice pairs.
                    </p>
                </>}
                ko={<p>
                    대응이 없는데 포획 영역은 무엇인가? 단위 테스트가 정직한 답을 못 박는다. 점 네 개로 된 클러스터 하나 — σ_eff = √0.125 ≈ 0.354 인
                    십자 — 그리고 소스는 그 십자를 x 로 −0.25 밀어 만든 것. 점-점 규칙이면 +0.25 를 정확히 돌려준다. NDT 는 그럴 수 없다: 점수는 중심들이
                    blob 폭보다 가깝게 몰린 bump 네 개의 합이고, 그래서 표면엔 정류점이 여럿이며 뉴턴+백트래킹은 가장 가까운 하나로 미끄러진다 — dx = 0.4000… 에
                    못 박혀 있고 +0.25 가 아니다; 반복 한 번만 해도 이미 0.3933…. 기하가 퇴화할 때 조용히 틀리는 부드러운 포획 — ICP 의 알리아싱된 격자 쌍과
                    정확히 같은 정직함이다.
                </p>}
            />

            <h2>{t("Why the Scenario Is a Staircase", "왜 시나리오가 계단인가")}</h2>
            <T
                en={<>
                    <p>
                        Run this estimator on room01_straight — ICP's home map, walls parallel to the axes — and it honestly fails: 6 of
                        the 8 steps recover but step 2 collapses to identity (dx = −0.0011 against a true +0.25). The measured gradient at
                        that step says why: <InlineMath math="\lvert\partial\text{score}/\partial x\rvert = 0.037"/> against{" "}
                        <InlineMath math="\lvert\partial\text{score}/\partial y\rvert = 169"/> and 220 on the heading — the x signal literally
                        vanishes. A wall's points spread along its whole visible length, so their clusters are translation-invariant{" "}
                        <em>along</em> the wall: score carries no signal in that direction, and whether a step recovers at all hangs on coverage-edge
                        accidents. ATE lands at 0.334 where ICP gets 0.021 on the same map.
                    </p>
                    <p>
                        The staircase is therefore not decoration — it is the minimum structure that makes this algorithm honest: every riser spans
                        exactly one row height, so each cluster is bounded on <em>both</em> axes and both axes carry signal. Inside, the estimator tracks
                        each 0.25 m step to centimeters (the demo pins dx = +0.2439/+0.2211/+0.2377/+0.2593 per step). The path stops at x = 2.75 because
                        free space on the path row ends at the face x = 3.5; extending it to x = 3.25 is measured too — no collapse, but the last step's
                        y/θ error grows (twist +0.2498/+0.0498/+0.0103, ATE 0.054 against 0.052) as the head-on riser crowds the treads out of the fan.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 추정기를 room01_straight — ICP 의 본진 지도, 축에 평행한 벽들 — 에서 돌리면 정직하게 실패한다: 8 스텝 중 6 개는 복원되지만
                        스텝 2 는 정체로 붕괴하고(dx = −0.0011, 참값은 +0.25) 그 스텝의 실측 기울기가 이유를 말한다: |∂score/∂x| = 0.037 대 |∂score/∂y| = 169,
                        heading 에게는 220 — x 신호는 문자 그대로 소멸한다. 벽의 점은 보이는 길이 전체로 퍼져 있으니 클러스터가 벽 방향을 따라 병진 불변이고,
                        score 는 그 축 신호를 잃으며 스텝이 복원되느냐는 커버리지 가장자리 우연에 매달리게 된다. ATE 는 0.334 에 닿고 같은 지도에서 ICP 는 0.021 이다.
                    </p>
                    <p>
                        그래서 계단은 장식이 아니다 — 이 알고리즘을 정직하게 만드는 최소 구조다: 모든 riser 는 행 높이 하나를 정확히 가로지르고, 그래서
                        각 클러스터는 두 축 모두로 경계 지어지고 두 축 모두 신호를 싣는다. 안쪽에서 추정기는 스텝마다 0.25 m 를 센티미터로 따라간다
                        (데모는 스텝마다 dx = +0.2439/+0.2211/+0.2377/+0.2593 을 못 박는다). 경로는 x = 2.75 에서 멈춘다 — 경로 행의 자유 공간이
                        면 x = 3.5 에서 끝나니까; x = 3.25 로 늘린 것도 실측했다 — 붕괴는 없지만 마지막 스텝의 y/θ 오차가 커진다
                        (twist +0.2498/+0.0498/+0.0103, ATE 0.054 대 0.052), 정면으로 마주하는 riser 가 tread 들을 팬에서 밀어내기 때문이다.
                    </p>
                </>}
            />
            <T
                en={<p>
                    (This is the mirror image of ICP's honesty: what aliases one algorithm is signal for the other. The straight room that lets
                    point-to-point match perfectly is where NDT goes blind; the staircase that gives every cluster a second dimension is where this
                    page lives.)
                </p>}
                ko={<p>
                    (이것은 ICP 의 정직함의 거울상이다: 한 알고리즘을 알리아싱하는 것이 다른 알고리즘엔 신호다. 점-점이 완벽히 맞는 직선 방이 NDT 를
                    실명하게 하고, 모든 클러스터에 두 번째 차원을 주는 계단이 이 페이지가 사는 땅이다.)
                </p>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop below is the whole estimator, with the same fixed operation order as every page in this repo — scan-order bucketing,
                    ascending accumulation (explicit loops, not a compensated sum), the quadratic form evaluated as{" "}
                    <InlineMath math="(d_{xq}u_1 + d_{yq}u_2)/2"/>, λ doubling on Sylvester's criterion, cofactor expansion along row 0, halving exact in
                    binary — pinned so Python, C++ and TypeScript reproduce identical values (the web engine within parity tolerance). Note what is
                    absent again: no noise model, no covariance on the output. NDT carries no uncertainty; that is what the filter_based branch adds
                    on top of this.
                </p>}
                ko={<p>
                    아래 루프가 추정기 전체이고, 이 저장소의 모든 페이지와 같은 고정 연산 순서를 지킨다 — 스캔 순서 버킷팅, 좌결합 축적(보정 합이 아닌
                    명시적 루프), 이차형식을 <InlineMath math="(d_{xq}u_1 + d_{yq}u_2)/2"/>로 평가, 실베스터 판정에 의한 λ 배가, row-0 여인수 전개,
                    이진에서 정확한 절반 줄이기 — Python·C++·TypeScript 가 같은 값을 재현하도록 못 박혀 있다(웹 엔진은 패리티 허용 오차 안에서). 없는 것에
                    다시 주목하라: 노이즈 모델 없고 출력에 공분산 없다. NDT 는 불확실성을 싣지 않고, 그 위에 얹는 것이 filter_based 갈래의 일이다.
                </p>}
            />
            <Pseudocode code={`build_field(points, cell):                                  # one distribution per cell — the field
    bucket points into (⌊x/cell⌋, ⌊y/cell⌋) in scan order      # fixed order — mirrors bit-for-bit
    for each bucket with n ≥ 3:                                     # fewer than three → no distribution
        μ ← ascending sum / n;  S ← Σ (x−μ)(x−μ)ᵀ                   # RAW scatter, not S/n
        skip the cell if det(S) is exactly 0.0                        # collinear-on-an-axis is not a measurement

score(a) = Σ_i exp(−½ q_iᵀ S⁻¹ q_i),  q_i = R(a)x_i + a − μ_k(i)    # empty cell → contributes exactly 0

solve(source, target, cell, λ₀, eps, max_iters):                    # one pairwise registration — the whole algorithm
    if source or target empty: return identity                       # no information, no motion
    field ← build_field(target); (ax, ay, ang) ← 0                   # the estimate starts at identity (relative!)
    repeat ≤ max_iters:
        g_j ← Σ E·u_j and H_jk ← Σ E(−u_j u_k + W_jk) in scan order  # gradient and Hessian of f = −score
        λ ← λ₀; while any Sylvester leading minor of (H+λI) ≤ 0: λ ← 2λ   # terminates: every entry of H is finite
        Δ ← Cramer on (H + λI)Δ = −g, cofactors along row 0          # fixed expansion order
        accept only if score(a+Δ) > score(a) strictly; else halve Δ (exact in binary)
        stop when max(|Δdx|, |Δdy|, |Δdθ|) ≤ eps without strict improvement → return current estimate
    return (ax, ay, ang)

run(step):
    t = 0: pose ← declared gauge (x₀, y₀, θ₀); prev ← scan          # the frame is a declaration, not a measurement
    t ≥ 1: twist ← solve(scan_t, prev_scan);  pose ← pose ⊕ twist   # Step.odom is NEVER read`}
            />
            <T
                en={<ol>
                    <li><strong>The gauge is declared.</strong> Same convention as ICP: the absolute frame comes from{" "}
                        <InlineMath math="x_0/y_0/\theta_0"/> parameters, step 0 emits it verbatim, and every later pose is composed from scans alone.
                        The scenario's <InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/> is logic, not a fudge — the estimator never reads{" "}
                        <InlineMath math="u_t"/>.</li>
                    <li><strong>Absorbing, not matching:</strong> there is no correspondence list anywhere in the implementation. A point landing in an
                        empty cell contributes exactly 0; everything else is one smooth score over a field of Gaussians.</li>
                    <li><strong>Damped Newton, fixed point convergence:</strong> λ doubles while H+λI fails Sylvester (measured: ×30 doublings at step 1's
                        second iteration), and the line search accepts only strict improvement — max_iters exists because convergence is a fixed point, not
                        a guarantee; the demo reaches 8/13/6/11 iterations.</li>
                    <li><strong>Emission</strong>: pose_estimated per step, no cov field at all — an honest trace of a method that carries no uncertainty model.</li>
                </ol>}
                ko={<ol>
                    <li><strong>게이지는 선언이다.</strong> ICP 와 같은 관례: 절대 좌표는 <InlineMath math="x_0/y_0/\theta_0"/> 파라미터에서 오고, 스텝 0 은
                        그것을 그대로 방출하며 이후 모든 자세는 스캔만으로 합성된다. 시나리오의 <InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/>은 정직함의
                        기만이 아니라 논리다 — 추정기는 <InlineMath math="u_t"/>를 전혀 읽지 않는다.</li>
                    <li><strong>대응이 아니라 흡수:</strong> 구현 어디에도 대응 목록은 없다. 빈 셀에 닿는 점은 정확히 0 을 기여하고, 나머지는 전부 가우스들의
                        필드 위에 매끄러운 점수 하나다.</li>
                    <li><strong>감쇠 뉴턴, 고정점 수렴:</strong> H+λI 가 실베스터에 실패하는 동안 λ 는 배가되고(실측: 스텝 1 반복 2 에서 ×30), 선 탐색은 엄격한
                        개선만 수용한다 — max_iters 는 수렴이 보증이 아니라 고정점이라 존재하고, 데모는 스텝마다 8/13/6/11 회에 도달한다.</li>
                    <li><strong>방출</strong>: 스텝마다 pose_estimated, cov 필드는 아예 없음 — 불확실성 모델을 싣지 않는 방법의 정직한 trace 다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox runs this exact estimator live — same operation order as the repository's Python and C++ (parity checked to{" "}
                        <InlineMath math="10^{-9}"/>). On the staircase, the estimate tracks each 0.25 m step to centimeters: pinned dx =
                        +0.2439/+0.2211/+0.2377/+0.2593 against a true +0.25, ATE ≈ 0.052 (RPE ≈ 0.062). Everything above the σ floor is what
                        the blob width lets through.
                    </p>
                    <p>
                        Cycle the chips to see what actually does the work: <code>cell_size</code> is a real knob here (unlike ICP's d_max) — widening
                        it to 1.0 smooths every cluster into a fatter blob and ATE drops to ≈ 0.030, tightening it to 0.25 shrinks the cells below the
                        staircase step and lands at ≈ 0.040; the cell size <em>is</em> the capture range. <code>max_iters</code> 64 → 1 pins a single
                        Newton pass per step — ATE jumps to ≈ 0.216, which is what the iteration was doing all along. <code>lam0</code> 1e-6 → 1e-3 starts
                        the damping two orders higher and drags ATE to ≈ 0.087: over-damped steps halve more before they are allowed to move.
                    </p>
                </>}
                ko={<>
                    <p>
                        아래 sandbox 는 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++ 과 같은 연산 순서(패리티{" "}
                        <InlineMath math="10^{-9}"/>까지 검사). 계단에서 추정은 스텝마다 0.25 m 를 센티미터로 따라간다: dx = +0.2439/+0.2211/+0.2377/+0.2593
                        (참값 +0.25), ATE ≈ 0.052(RPE ≈ 0.062). σ 바닥 위는 전부 blob 폭이 통과시킨 것이다.
                    </p>
                    <p>
                        칩을 돌려 실제로 일이 일어나는 곳을 봐라: <code>cell_size</code> 는 여기서 진짜 노브다(ICP 의 d_max 와 다르다) — 1.0 으로 넓히면 모든
                    클러스터가 더 뚱뚱한 blob 으로 매끄러워져 ATE ≈ 0.030 으로 떨어지고, 0.25 로 조이면 셀이 계단 스텝보다 작아져 ≈ 0.040 에 앉는다; 셀 크기가
                    곧 포획 범위다. <code>max_iters</code> 64 → 1 은 스텝당 뉴턴 패스 하나로 못 박고 — ATE 는 ≈ 0.216 으로 뛰고, 반복이 그동안 하던 일이 그대로
                        드러난다. <code>lam0</code> 1e-6 → 1e-3 은 감쇠를 두 자릿수 위에서 시작해 ATE 를 ≈ 0.087 로 끄어당긴다: 과감쇠 단계는 움직이기 전에 더 많이 절반 난다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "room02_diagonal"}]}
                run={runNdt}
                params={{x0: 1.75, y0: 1.75, theta_deg: 0, cell_size: 0.5, lam0: 1e-6, eps: 1e-9, max_iters: 64}}
                chips={[{key: "cell_size", values: [0.5, 1.0]}, {key: "max_iters", values: [64, 1]}, {key: "lam0", values: [1e-6, 1e-3]}]}
                label={t(
                    "ndt live — scan-to-field registration on the staircase: widen cell_size and every cluster becomes a fatter blob (ATE 0.052 → 0.030), drop max_iters to 1 and watch what the iteration was doing (→ 0.216)",
                    "ndt 라이브 — 계단에서 스캔-투-필드 등록: cell_size 를 넓히면 모든 클러스터가 뚱뚱한 blob 이 되고(ATE 0.052 → 0.030), max_iters 를 1 로 낮추면 반복이 하던 일이 그대로 보인다(→ 0.216)",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other operation for operation: scan-order bucketing,
                    ascending accumulation into μ and S, the quadratic form evaluated as (dxq·u1 + dyq·u2)/2, λ doubling on Sylvester's criterion, cofactor
                    expansion along row 0 — and the browser engine replays that contract with identical metrics. The C++ side routes its cos/sin pairs through
                    dlsym-resolved scalar libm calls (the same-argument pair is exactly what Apple clang folds into a SIMD sincos whose results differ from
                    CPython's by 1 ulp on rare inputs); exp binds directly — probe-verified bit-identical across the score's exponent domain. The Python side
                    accumulates with explicit loops, deliberately not the builtin sum(): from CPython 3.12 (gh-100425) that is Neumaier compensated summation,
                    a different arithmetic expression — measured once on this very algorithm, where one mean landing 1 ulp off moved the demo's ATE at digit ten.
                    The scenario's seed enters the simulator only — the algorithm itself reads no parameters but the gauge and the three solver knobs.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: 스캔 순서 버킷팅, μ 와 S 에 대한 좌결합 축적, 이차형식을
                    (dxq·u1 + dyq·u2)/2 로 평가, 실베스터 판정에 의한 λ 배가, row-0 여인수 전개 — 그리고 브라우저 엔진은 그 계약을 같은 지표로 재생한다. C++ 쪽은
                    cos/sin 쌍을 dlsym 으로 해석된 스칼라 libm 호출로 라우팅하고(같은 인자 쌍이 바로 Apple clang 이 SIMD sincos 로 접는 대상이고 드문 입력에서
                    CPython 과 1 ulp 다르다), exp 는 직접 바인딩 — 점수의 지수 도메인에서 비트 동일을 프로브로 검증했다. python 쪽은 명시적 루프로 축적하고 내장
                    sum() 은 일부러 쓰지 않는다: CPython 3.12(gh-100425) 부터 그건 Neumaier 보정 합이고 — 다른 산술 표현이다 — 바로 이 알고리즘에서 실측한 적 있다,
                    평균 하나가 1 ulp 어긋나는 것이 데모 ATE 를 열 번째 자리까지 옮긴다. 시나리오의 seed 는 시뮬레이터에만 들어가고 — 알고리즘 자체는 게이지와 솔버
                    세 개 외의 파라미터를 읽지 않는다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/registration/ndt.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/registration/ndt.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/registration/ndt.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/slam/registration/ndt.hpp`,
                            },
                            {
                                name: "cpp/src/registration/ndt.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/registration/ndt.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "저장소에서 그대로 embed 한 추정기 소스 — demo 드라이버와 trace 계약은 demos/ 와 core/ 에 있다",
                    "The estimator sources, embedded from the repository — the demo driver and trace contract live in demos/ and core/",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    P. Biber, W. Strasser,{" "}
                    <a href="https://ieeexplore.ieee.org/document/1249285" target="_blank" rel="noopener noreferrer">
                        <em>The normal distributions transform: a new approach to laser scan matching</em>
                    </a>,
                    Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2743–2748, 2003 — the
                    algorithm itself: scan matching by transforming point clouds into continuous probability density functions,
                    a grid of Gaussian cell distributions, and Newton-style score maximization instead of correspondence.
                </li>
                <li>
                    M. Magnusson, A. Lilienthal, T. Duckett,{" "}
                    <a href="https://doi.org/10.1002/rob.20204" target="_blank" rel="noopener noreferrer">
                        <em>Scan registration for autonomous mining vehicles using 3D-NDT</em>
                    </a>,
                    Journal of Field Robotics 24(10):803–825, 2007 — the family's maturation: multi-resolution NDT hierarchies
                    and the scan-matching-as-density-registration view this page implements in its plainest 2-D form.
                </li>
                <li>
                    M. Magnusson, A. Nuchter, C. Lorken, A. J. Lilienthal, J. Hertzberg,{" "}
                    <a href="https://ieeexplore.ieee.org/document/5152538" target="_blank" rel="noopener noreferrer">
                        <em>Evaluation of 3D registration reliability and speed — a comparison of ICP and NDT</em>
                    </a>,
                    Proc. IEEE International Conference on Robotics and Automation (ICRA), 3907–3912, 2009 — the branch's
                    comparative literature: what density-registration buys over point-to-point matching, measured side by side —
                    exactly the question this page answers with pinned numbers instead of adjectives.
                </li>
            </ol>
        </>
    )
}

export default NdtPage
