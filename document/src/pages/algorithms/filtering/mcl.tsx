import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import {SandboxScene} from "../../../components/panels/Sandbox";
import {runMcl} from "../../../libs/algorithms/mcl";
import pyImpl from "../../../../../python/slam/filtering/mcl.py?raw";
import pyStats from "../../../../../python/slam/core/stats.py?raw";
import cppImpl from "../../../../../cpp/include/slam/filtering/mcl.hpp?raw";
import cppSrc from "../../../../../cpp/src/filtering/mcl.cpp?raw";

const REPO = "https://github.com/robotics-study/slam_basic"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const MclPage = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the filtering branch's fourth step, and it answers the question the previous page ended
                    on. A fixed particle budget is a lottery: too few samples and the true mode dies before it is
                    ever sampled; too many and every raycast of every step is wasted on a belief that collapsed to
                    one ancestor three steps ago. The previous demo showed both failure faces — at{" "}
                    <InlineMath math="t=0"/> the cloud straddles two headings, and once the first eastward move kills
                    the west blob, hundreds of samples keep guarding a hypothesis nobody believes anymore.{" "}
                    <strong>Monte Carlo Localization with KLD-sampling</strong> keeps the same filter — local prior
                    over the declared cell × full heading, bootstrap motion model, beam-likelihood weighting,
                    weighted-mean readout — and adds exactly one mechanism: at every resample-move step the sample
                    count is not fixed. Samples are drawn <em>one at a time</em>, and each new sample's bin either
                    widens the covered support (raising the required count) or doesn't; when the coverage{" "}
                    <em>statistically certifies</em> that the cloud is dense enough, the step stops drawing. What
                    adapts is the count, not the prior — this is still local localization.
                </p>}
                ko={<p>
                    이것은 필터링 갈래의 네 번째 단계이고, 이전 페이지가 질문으로 끝낸 것에 답한다. 고정 입자 예산은
                    경품 추첨이다: 표본이 너무 적으면 참된 모드가 그려지기도 전에 죽고, 너무 많으면 세 스텝 전 이미
                    조상 하나로 붕괴한 믿음을 더 이상 아무도 믿지 않는 가설에 매 스텝 모든 레이캐스트를 낭비한다. 이전
                    데모는 실패의 두 얼굴을 모두 보여줬다 — <InlineMath math="t=0"/>에 구름은 두 heading에 걸쳐 있고,
                    첫 동쪽 이동이 서쪽 덩어리를 죽인 뒤에도 수백 표본은 아무도 믿지 않는 가설을 계속 지킨다.{" "}
                    <strong>KLD-sampling MCL</strong>은 같은 필터를 유지한다 — 선언된 셀 × heading 전체의 지역 사전분포,
                    부트스트랩 운동 모델, 빔 우도 가중, 가중 평균 판독 — 그리고 정확히 하나의 메커니즘을 더한다: 매
                    resample-move 스텝에서 표본 수는 고정되지 않는다. 표본은 <em>하나씩</em> 그려지고, 새 표본의 빈이
                    커버된 지지를 넓히면(필요 수를 올리고) 그렇지 않으면 그대로; 커버가 구름이 충분히 빽빹하다는 것을{" "}
                    <em>통계적으로 보증</em>하면 그 스텝은 그리기를 멈춘다. 적응하는 것은 수가 아니라 — 여전히 지역
                    국소화다.
                </p>}
            />

            <h2>{t("How Many Samples Are Enough?", "표본은 얼마나 충분한가")}</h2>
            <T
                en={<>
                    <p>
                        A fixed <InlineMath math="N"/> forces you to guess the worst case at design time. But the
                        belief's <em>complexity</em> changes every step: after the scan resolves the heading ambiguity
                        it lives in a corner of the map, and 33 samples describe it as well as 800 would — the extra
                        767 raycasts buy nothing. The trick is to turn "dense enough" into a statistic. Discretize
                        pose space into boxes — here{" "}
                        <InlineMath math="\text{bin}_{xy} \times \text{bin}_{xy} \times \text{bin}_\theta"/> cells of
                        the world grid (0.5 m × 0.5 m × 10° in this repo's config, exactly the paper's experiment
                        discretization) — and let <InlineMath math="p"/> be the true belief discretized over them. If{" "}
                        <InlineMath math="n"/> samples are drawn i.i.d. from <InlineMath math="p"/>, they land in bins
                        multinomially, and the empirical distribution{" "}
                        <InlineMath math="\hat p"/> built from the counts is a consistent estimator of{" "}
                        <InlineMath math="p"/>. The question becomes: how large must <InlineMath math="n"/> be so that{" "}
                        <InlineMath math="\mathrm{KL}(\hat p \,\|\, p) \le \varepsilon"/> with probability{" "}
                        <InlineMath math="1 - \delta"/>?
                    </p>
                    <p>
                        That question has a classical answer (next section). Its shape is worth seeing before the
                        algebra: the required count grows like{" "}
                        <InlineMath math="\propto 1/(2\varepsilon)"/> and with the number of occupied bins{" "}
                        <InlineMath math="k"/> — so a belief concentrated in two bins needs tens of samples, a belief
                        smeared over forty needs hundreds. The filter simply counts its own occupied bins and stops
                        drawing when the bound is satisfied:
                    </p>
                    <BlockMath math="\text{stop when } n \ge n_\chi(k) \;\text{and}\; n \ge n_{\min},\quad\text{or at the hard cap } N_{\max}"/>
                    <p>
                        Two honest caveats, both from the paper itself. The filter never knows{" "}
                        <InlineMath math="p"/> — it treats the bins its <em>own samples</em> occupy as the support,{" "}
                        <InlineMath math="k"/> being the count of those; when the predictive belief stands in for an
                        unknown posterior, a too-loose bound can certify coverage of the wrong support. And the bins
                        are a world-coordinate grid: a cluster crossing the{" "}
                        <InlineMath math="\pm\pi"/> heading seam splits into two bins and the count spikes — an
                        artifact of the discretization, not of the theory. The demo below shows both honestly.
                    </p>
                </>}
                ko={<>
                    <p>
                    고정 <InlineMath math="N"/>은 설계 시점에 최악의 경우를 추측하게 강요한다. 그런데 belief의{" "}
                    <em>복잡도</em>는 스텝마다 바뀐다: 스캔이 heading 모호성을 풀면 믿음은 지도 구석에 살고, 33표본이
                    800개와 똑같이 잘 묘사한다 — 나머지 767 레이캐스트는 아무것도 사지 못한다. 재주는 "충분히 빽빹"을
                    통계량으로 바꾸는 것: 자세 공간을 박스로 이산화한다 — 여기선 세계 격자의{" "}
                    <InlineMath math="\text{bin}_{xy} \times \text{bin}_{xy} \times \text{bin}_\theta"/> 셀(이 저장소
                    설정에서 0.5 m × 0.5 m × 10°, 논문 실험의 이산화와 정확히 같음) — 그리고{" "}
                    <InlineMath math="p"/>를 그 위에 이산화된 참된 belief라 하자. <InlineMath math="n"/>개 표본이{" "}
                    <InlineMath math="p"/>에서 i.i.d.로 그려지면 빈에 다항분포로 착지하고, 세기로 만든 경험 분포{" "}
                    <InlineMath math="\hat p"/>는 <InlineMath math="p"/>의 일관 추정량이 된다. 질문은 이렇다:{" "}
                    <InlineMath math="\mathrm{KL}(\hat p \,\|\, p) \le \varepsilon"/>가 확률 <InlineMath math="1 - \delta"/>로
                    성립하려면 <InlineMath math="n"/>은 얼마나 커야 하는가?
                    </p>
                    <p>
                        이 질문은 고전적 답이 있다(다음 절). 대수 전에 모양을 보면 값지다: 필요 수는{" "}
                        <InlineMath math="\propto 1/(2\varepsilon)"/>와 점유 빈 수 <InlineMath math="k"/>에 비례해
                        커진다 — 두 빈에 집중된 믿음은 수십 표본, 사십 개 빈에 퍼진 믿음은 수백 표본이 필요하다. 필터는
                        자기가 점유한 빈을 그냥 세고, 상한이 충족되면 그리기를 멈춘다:
                    </p>
                    <BlockMath math="\text{stop when } n \ge n_\chi(k) \;\text{and}\; n \ge n_{\min},\quad\text{or at the hard cap } N_{\max}"/>
                    <p>
                        정직한 주의 둘, 모두 논문 자신의 것. 필터는 <InlineMath math="p"/>를 결코 모른다 — 자기가{" "}
                        <em>점유한</em> 빈들을 지지로 취급하고, <InlineMath math="k"/>는 그 개수다; 예측 belief가 알려지지
                        않은 사후를 대신할 때 너무 느슨한 상한은 잘못된 지지의 커버를 보증할 수 있다. 그리고 빈은
                        세계좌표 격자다: 클러스터가 <InlineMath math="\pm\pi"/> seam을 넘으면 두 빈으로 갈라지고 수가
                        솟구친다 — 이론이 아니라 이산화의 아티팩트다. 아래 데모는 둘 다 정직하게 보여준다.
                    </p>
                </>}
            />

            <h2>{t("From Coverage to a Bound", "커버에서 상한으로")}</h2>
            <T
                en={<>
                    <p>
                        Here is the whole statistical argument, which is a likelihood-ratio test in disguise. With{" "}
                        <InlineMath math="k"/> occupied bins, counts <InlineMath math="x_1, \dots, x_k"/> and true
                        discretized belief <InlineMath math="p = (p_1, \dots, p_k)"/>, the likelihood of observing
                        those counts under <InlineMath math="p"/> is multinomial. The MLE over all distributions{" "}
                        <InlineMath math="q"/> on these bins is the empirical one,{" "}
                        <InlineMath math="\hat p_i = x_i/n"/>, so the generalized likelihood ratio is:
                    </p>
                    <BlockMath math="\lambda_n = \prod_{i=1}^{k} \left(\frac{\hat p_i}{p_i}\right)^{x_i}
                        \qquad\Longrightarrow\qquad
                        \log \lambda_n = \sum_i x_i \log \frac{\hat p_i}{p_i} = n\,\mathrm{KL}(\hat p \,\|\, p)"/>
                    <p>
                        The last step is the definition of KL read backwards:{" "}
                        <InlineMath math="\sum_i x_i \log(\hat p_i / p_i) = n \sum_i \hat p_i \log(\hat p_i / p_i)"/>.
                        So "the samples came from <InlineMath math="p"/>" versus "they came from their own empirical
                        distribution" is a test of{" "}
                        <InlineMath math="\mathrm{KL}(\hat p \,\|\, p) = 0"/> — and Wilks' theorem says the log-likelihood-ratio
                        statistic converges in distribution to chi-square on the model's free parameters. The simplex on{" "}
                        <InlineMath math="k"/> bins has <InlineMath math="k - 1"/> of them:
                    </p>
                    <BlockMath math="2 \log \lambda_n = 2 n\,\mathrm{KL}(\hat p \,\|\, p) \;\xrightarrow{\;d\;}\; \chi^2_{k-1}"/>
                    <p>
                        Read the quantile backwards: if{" "}
                        <InlineMath math="n \ge \chi^2_{k-1,\,1-\delta} / (2\varepsilon)"/> then{" "}
                        <InlineMath math="P\big(\mathrm{KL}(\hat p \,\|\, p) > \varepsilon\big) \le \delta"/>. That is
                        the guarantee the filter enforces at every step — with{" "}
                        <InlineMath math="\varepsilon = 0.1"/> and{" "}
                        <InlineMath math="1 - \delta = 0.99"/> (the paper's own experiment settings), a belief spread
                        over two bins needs about 33 samples, and the same belief smeared over forty bins needs about
                        319. Sample count literally <em>is</em> uncertainty.
                    </p>
                </>}
                ko={<>
                    <p>
                    여기 전체 통계 논증이 있다 — 사실은 변장한 우도비 검정이다. <InlineMath math="k"/>개의 점유된 빈,
                    세기 <InlineMath math="x_1, \dots, x_k"/>, 참된 이산 belief <InlineMath math="p = (p_1, \dots, p_k)"/>에서{" "}
                    <InlineMath math="p"/> 아래 그 세기가 관측될 우도는 다항분포다. 이 빈 위 모든 분포{" "}
                    <InlineMath math="q"/>에 대한 MLE는 경험 분포 <InlineMath math="\hat p_i = x_i/n"/>이므로 일반화된
                    우도비는 이렇게 된다:
                    </p>
                    <BlockMath math="\lambda_n = \prod_{i=1}^{k} \left(\frac{\hat p_i}{p_i}\right)^{x_i}
                        \qquad\Longrightarrow\qquad
                        \log \lambda_n = \sum_i x_i \log \frac{\hat p_i}{p_i} = n\,\mathrm{KL}(\hat p \,\|\, p)"/>
                    <p>
                        마지막 단계는 KL의 정의를 거꾸로 읽은 것이다:{" "}
                        <InlineMath math="\sum_i x_i \log(\hat p_i / p_i) = n \sum_i \hat p_i \log(\hat p_i / p_i)"/>.
                        즉 "표본이 <InlineMath math="p"/>에서 왔다" 대 "자기 경험 분포에서 왔다"는{" "}
                        <InlineMath math="\mathrm{KL}(\hat p \,\|\, p) = 0"/>에 대한 검정이고 — Wilks 정리는 우도비
                        통계량의 로그가 자유 매개변수 수의 카이제곱으로 분포 수렴한다고 말한다: <InlineMath math="k"/>개
                        빈의 심플렉스는 자유도가 <InlineMath math="k - 1"/>이다.
                    </p>
                    <BlockMath math="2 \log \lambda_n = 2 n\,\mathrm{KL}(\hat p \,\|\, p) \;\xrightarrow{\;d\;}\; \chi^2_{k-1}"/>
                    <p>
                        양자화를 거꾸로 읽는다: <InlineMath math="n \ge \chi^2_{k-1,\,1-\delta} / (2\varepsilon)"/>이면{" "}
                        <InlineMath math="P\big(\mathrm{KL}(\hat p \,\|\, p) > \varepsilon\big) \le \delta"/>. 그리고
                        필터는 매 스텝 바로 그 보증을 집행한다 — <InlineMath math="\varepsilon = 0.1"/>,{" "}
                        <InlineMath math="1 - \delta = 0.99"/>(논문 실험 설정 그대로)로 두 빈에 퍼진 믿음은 표본 약 33개,
                        같은 믿음이 마흔 개 빈에 퍼지면 약 319개가 필요하다. 표본 수가 문자 그대로 <em>그것이</em>{" "}
                        불확실성이다.
                    </p>
                </>}
            />

            <Proof title={t("Proposition (why the log-likelihood ratio is n·KL)", "명제 (왜 우도비의 로그가 n·KL인가)")}>
                <T
                    en={<p>
                        The multinomial likelihood of counts <InlineMath math="x"/> under{" "}
                        <InlineMath math="q"/> is proportional to <InlineMath math="\prod_i q_i^{x_i}"/> (the factorial
                        constant cancels in the ratio). Maximizing over all <InlineMath math="q"/> on the simplex gives{" "}
                        <InlineMath math="\hat p"/> — plug <InlineMath math="q = \hat p"/> and{" "}
                        <InlineMath math="q = p"/> into numerator and denominator:
                        <InlineMath math="\lambda_n = \prod_i (\hat p_i / p_i)^{x_i}"/>, so{" "}
                        <InlineMath math="\log \lambda_n = \sum_i x_i \log(\hat p_i/p_i) = n \sum_i (x_i/n) \log(\hat p_i / p_i) = n\,\mathrm{KL}(\hat p \,\|\, p)"/>.
                        Wilks' theorem then applies with degrees of freedom equal to the simplex's free dimension,{" "}
                        <InlineMath math="k - 1"/> — and because KL is non-negative (Gibbs), the one-sided event{" "}
                        <InlineMath math="\{\mathrm{KL} > \varepsilon\}"/> has probability at most{" "}
                        <InlineMath math="\delta"/> once{" "}
                        <InlineMath math="2n\varepsilon \ge \chi^2_{k-1,\,1-\delta}"/>. <InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        <InlineMath math="q"/> 아래 세기 <InlineMath math="x"/>의 다항 우도는{" "}
                        <InlineMath math="\prod_i q_i^{x_i}"/>에 비례하고(계수상수는 비율에서 소거) 심플렉스 위 모든{" "}
                        <InlineMath math="q"/>에 대한 최대화는 <InlineMath math="\hat p"/>를 준다 — 분모·분자에{" "}
                        <InlineMath math="q = \hat p"/>, <InlineMath math="q = p"/>를 대입하면{" "}
                        <InlineMath math="\lambda_n = \prod_i (\hat p_i / p_i)^{x_i}"/>이고 따라서{" "}
                        <InlineMath math="\log \lambda_n = \sum_i x_i \log(\hat p_i/p_i) = n\,\mathrm{KL}(\hat p \,\|\, p)"/>.
                        이후 Wilks 정리가 자유도 = 심플렉스의 자유 차수 <InlineMath math="k - 1"/>로 적용되고 — KL은
                        비음(Gibbs)이므로 단측 사건 <InlineMath math="\{\mathrm{KL} > \varepsilon\}"/>는{" "}
                        <InlineMath math="2n\varepsilon \ge \chi^2_{k-1,\,1-\delta}"/>부터 확률 <InlineMath math="\delta"/> 이하.{" "}
                        <InlineMath math="\blacksquare"/>
                    </p>}
                />
            </Proof>

            <h2>{t("The Quantile in Closed Form", "폐형이 된 양자화")}</h2>
            <T
                en={<>
                    <p>
                        A chi-square quantile has no closed form — but Wilson &amp; Hilferty's 1931 cube-root
                        approximation is excellent even at tiny degrees of freedom, and that is what the implementation
                        actually computes:
                    </p>
                    <BlockMath math="n_\chi(k) = \frac{\nu}{2\varepsilon} \left[ \left(1 - \frac{2}{9\nu}\right) + z_q \sqrt{\frac{2}{9\nu}} \right]^{3},\qquad \nu = k - 1,\quad q = 1-\delta"/>
                    <p>
                        with <InlineMath math="z_q = \Phi^{-1}(1-\delta)"/> the standard-normal quantile — computed once
                        per configuration by Acklam's rational approximation (relative error below{" "}
                        <InlineMath math="1.15 \times 10^{-9}"/>: a rational function of{" "}
                        <InlineMath math="\sqrt{-2\log p}"/> in the tails, of <InlineMath math="q = p - 0.5"/> in the
                        center). The degenerate case is part of the contract: for <InlineMath math="k \le 1"/> the
                        bound is defined to be 0 — a belief living in one bin needs no coverage guarantee beyond{" "}
                        <InlineMath math="n_{\min}"/> — and since the bound scales as{" "}
                        <InlineMath math="\nu / 2\varepsilon"/>, halving <InlineMath math="\varepsilon"/> doubles every
                        count. The bound is evaluated with the <em>current</em>{" "}
                        <InlineMath math="k"/> after every single sample: a first-seen bin raises{" "}
                        <InlineMath math="k"/>, which recomputes{" "}
                        <InlineMath math="n_\chi"/> — that feedback loop is the whole algorithm.
                    </p>
                </>}
                ko={<>
                    <p>
                    카이제곱 양자화에 폐형은 없다 — 그러나 Wilson &amp; Hilferty의 1931년 세제곱근 근사는 자유도가 아주
                    작아도 훌륭하고, 구현이 실제로 계산하는 것이 그것이다:
                    </p>
                    <BlockMath math="n_\chi(k) = \frac{\nu}{2\varepsilon} \left[ \left(1 - \frac{2}{9\nu}\right) + z_q \sqrt{\frac{2}{9\nu}} \right]^{3},\qquad \nu = k - 1,\quad q = 1-\delta"/>
                    <p>
                        여기서 <InlineMath math="z_q = \Phi^{-1}(1-\delta)"/>는 표준정규 양자화 — 설정당 한 번, Acklam의
                        유리식 근사로 계산한다(상대 오차 <InlineMath math="1.15 \times 10^{-9}"/> 미만: 꼬리에{" "}
                        <InlineMath math="\sqrt{-2\log p}"/>의 유리식, 중앙에서 <InlineMath math="q = p - 0.5"/>의
                        유리식). 퇴화 케이스가 계약의 일부다: <InlineMath math="k \le 1"/>에서 상한은 0으로 정의된다 —
                        한 빈에 사는 믿음은 <InlineMath math="n_{\min}"/> 밖의 커버 보장이 필요 없다 — 그리고 상한은{" "}
                        <InlineMath math="\nu / 2\varepsilon"/>로 커지므로 <InlineMath math="\varepsilon"/>을 반으로
                        줄이면 모든 수가 두 배가 된다. 상한은 표본 하나마다 <em>현재</em>{" "}
                        <InlineMath math="k"/>로 평가된다: 처음 밟은 빈이 <InlineMath math="k"/>를 올리고, 그게{" "}
                        <InlineMath math="n_\chi"/>를 재계산한다 — 그 피드백 루프가 알고리즘 전체다.
                    </p>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop below is the whole estimator, with the same fixed operation order as every page in this
                    branch — ascending draws, scan-order likelihood, max-shifted exps — pinned so three languages
                    reproduce it bit for bit. Note what did <em>not</em> change: motion model, likelihood, readout are
                    byte-identical to the previous page; only the resample step grew a stopping rule.
                </p>}
                ko={<p>
                    아래 루프가 추정기 전체이고, 이 갈래의 모든 페이지와 같은 고정 연산 순서를 지킨다 — 오름차순 드로,
                    스캔 순서 우도, max-shifted exp — 세 언어가 비트 단위로 재현하도록 못 박혀 있다. 바뀌지{" "}
                    <em>않은</em> 것에 주목하라: 운동 모델, 우도, 판독은 이전 페이지와 문자 그대로 동일하고, 리샘플
                    단계에 정지 규칙이 자라난 것뿐이다.
                </p>}
            />
            <Pseudocode code={`init(t = 0): for i = 1..N_max (ascending draws):
      x^i ← ox + (col₀ + u₁)·res          # the declared start cell, uniform inside it
      y^i ← oy + (h − 1 − row₀ + u₂)·res
      θ^i ← u₃·2π − π                      # heading: everything from −π to π
      w^i ← 1/N_max                        # the PRIOR is fixed-size — adaptivity starts at t = 1

kld_update(u, z₁ … z_K):                   # every step after the first
    points ← (r, β) per scan point, in order; k ← 0; n ← 0; seen ← ∅; n_chi ← +∞
    loop one sample at a time:
        u₁ ~ U(0,1); walk old weights with strict > → ancestor x_old   # plain multinomial resampling
        ε ~ N(0, diag(σ_xy², σ_θ²));  x_new ← x_old ⊕ (u + ε)          # same motion model as before
        ll ← Σ_beams −½·((r − raycast(x_new, θ+β)) / σ_r)²             # miss → sentinel; max-shifted exps
        key ← (⌊x/bin_xy⌋, ⌊y/bin_xy⌋, ⌊θ/bin_θ⌋);  key ∉ seen → k ← k + 1
        n ← n + 1
        if n ≥ n_min: n_chi ← (k−1)/(2ε) · ((1 − 2/(9(k−1))) + z_q·√(2/(9(k−1))))³   # z_q = Φ⁻¹(1−δ), once
        stop when (n ≥ n_chi and n ≥ n_min) or n ≥ N_max
    w^i ← exp(ll^i − m)/Σ;  the old weights already acted — they drove the walk

readout: unchanged from the particle_filter page (weighted mean, circular heading, population std)`}/>
            <T
                en={<ol>
                    <li><strong>The prior is still fixed-size.</strong> At{" "}
                        <InlineMath math="t=0"/> all <InlineMath math="N_{\max}"/> particles are drawn uniformly over
                        the declared cell × full heading — adaptivity starts at the first KLD step, and the fixed
                        budget only ever applies to the prior.</li>
                    <li><strong>Sampling one at a time is not cosmetic:</strong> each sample's bin key{" "}
                        <InlineMath math="(\lfloor x/\text{bin}_{xy}\rfloor, \lfloor y/\text{bin}_{xy}\rfloor, \lfloor \theta/\text{bin}_\theta\rfloor)"/>
                        {" "}can raise <InlineMath math="k"/>, which raises the bar the loop is trying to clear. The
                        ancestor walk itself stays plain multinomial resampling (one uniform draw, strict{" "}
                        <InlineMath math="\gt;"/> over cumulative weights) — KLD changes <em>when</em> it stops, not{" "}
                        <em>how</em> it picks.</li>
                    <li><strong>The stop rule has two floors and a ceiling:</strong> the bound only applies once{" "}
                        <InlineMath math="n \ge n_{\min}"/> (default 10, the paper's own floor), and{" "}
                        <InlineMath math="N_{\max}"/> caps the loop — when the bound exceeds the cap, the guarantee is
                        silently violated. That is honest: the cap is a real-time budget, and the demo shows what
                        violating it looks like.</li>
                    <li><strong>Emission</strong>: pose_estimated (pose + diagonal stds) first, particles_updated
                        ([x, y, θ, w] per particle, ascending) second — at the same t. The cloud payload is where you
                        literally watch <InlineMath math="n(t)"/> breathe.</li>
                </ol>}
                ko={<ol>
                    <li><strong>사전분포는 여전히 고정 크기다.</strong> <InlineMath math="t=0"/>에서{" "}
                        <InlineMath math="N_{\max}"/>개를 선언된 셀 × heading 전체 위에서 균일하게 그린다 — 적응성은 첫
                        KLD 스텝에서 시작하고, 고정 예산은 사전분포에만 실제로 적용된다.</li>
                    <li><strong>하나씩 그리는 것은 장식이다:</strong> 각 표본의 빈 키{" "}
                        <InlineMath math="(\lfloor x/\text{bin}_{xy}\rfloor, \lfloor y/\text{bin}_{xy}\rfloor, \lfloor \theta/\text{bin}_\theta\rfloor)"/>가{" "}
                        <InlineMath math="k"/>를 올릴 수 있고, 그게 루프가 넘으려는 문턱을 올린다. 조상 워크 자체는
                        평범한 multinomial 리샘플 그대로다(균일 드로 하나, 누적 가중치 위 strict{" "}
                        <InlineMath math="\gt;"/>) — KLD는 <em>언제</em> 멈추는가만 바꾸고 <em>어떻게</em> 고르는지가
                        아니다.</li>
                    <li><strong>정지 규칙에 바닥 둘과 천장 하나:</strong> 상한은 <InlineMath math="n \ge n_{\min}"/>부터
                        적용되고(기본 10, 논문 자신의 바닥), <InlineMath math="N_{\max}"/>이 루프를 깎는다 — 상한이 예산을
                        넘으면 보증은 조용히 위반된다. 정직하다: 천장은 실시간 예산이고, 데모는 위반의 모습을 보여준다.</li>
                    <li><strong>방출</strong>: pose_estimated(자세 + 대각 표준편차)이 먼저, particles_updated(입자마다
                        [x, y, θ, w], 오름차순)가 나중 — 같은 t에서. 구름 페이로드가 바로 <InlineMath math="n(t)"/>가
                        숨쉬는 것을 문자로 지켜보는 곳이다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox runs this exact estimator live — same operation order as the repository's Python
                        and C++ (parity checked to <InlineMath math="10^{-9}"/>). Watch{" "}
                        <InlineMath math="n(t)"/>, the cloud size, which is now a signal in its own right: at{" "}
                        <InlineMath math="t=0"/> it is the full budget 800 (all distinct — nothing has resampled yet;
                        both twins survive, 427 of them facing east). At <InlineMath math="t=1"/> the first KLD step
                        samples until <InlineMath math="k=41"/> bins justify 319. Then the west blob dies,{" "}
                        <InlineMath math="k"/> collapses to 2 and every later step draws exactly{" "}
                        <InlineMath math="\lceil n_\chi(2)\rceil = 33"/> samples — 4% of the budget for the same job.
                        When the U-turn drags the cluster across position bins and the <InlineMath math="\pm\pi"/>
                        seam, <InlineMath math="k"/> spikes to 4/5/3 and the count jumps to 57, 67, 47 before settling
                        back. That breathing is the algorithm thinking.
                    </p>
                    <p>
                        The honesty check: <code>ate_rmse</code> ≈ 0.052 here versus ≈ 0.057 for the fixed-500 particle
                        filter on the same scenario — KLD-sampling bought a ~15× sample saving at{" "}
                        <em>equal</em> accuracy, because accuracy was never what the samples were buying (the blunt{" "}
                        <InlineMath math="\sigma_r"/> likelihood floors it near <InlineMath math="\sigma_r/\sqrt{k}"/>;
                        the large <code>rpe_rmse</code> ≈ 1.05 is still the heading jump when the surviving hypothesis
                        gets decided). KLD saves samples, not accuracy. Cycle the <code>epsilon</code> chip: at{" "}
                        <InlineMath math="\varepsilon = 0.01"/> every bound grows tenfold (33 → 330 for{" "}
                        <InlineMath math="k=2"/>) — and with <code>max_particles</code> at 200 the cap silently
                        overrides the guarantee. That is what violating a budget looks like.
                    </p>
                </>}
                ko={<>
                    <p>
                    아래 sandbox는 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++과 같은 연산 순서이고
                    (패리티를 <InlineMath math="10^{-9}"/>까지 검사). 이제 그 자체가 신호가 된 것,{" "}
                    <InlineMath math="n(t)"/>, 구름 크기를 지켜봐라: <InlineMath math="t=0"/>에 고정 예산 800이고(전부
                    distinct — 아직 아무것도 리샘플되지 않았고, 쌍둥이가 둘 다 살아 동쪽이 427), <InlineMath math="t=1"/>의
                    첫 KLD 스텝은 <InlineMath math="k=41"/> 빈이 319를 정당화할 때까지 그린다. 그러자 서쪽 덩어리가 죽고{" "}
                    <InlineMath math="k"/>가 2로 붕괴해 이후 모든 스텝은 정확히{" "}
                    <InlineMath math="\lceil n_\chi(2)\rceil = 33"/>개를 그린다 — 같은 일을 예산의 4%로. U-turn이
                    클러스터를 위치 빈과 <InlineMath math="\pm\pi"/> seam 너머로 끌면 <InlineMath math="k"/>가 4/5/3으로
                    솟구쳐 수가 57, 67, 47 뛰었다가 돌아온다. 그 호흡이 알고리즘이 생각하는 모습이다.
                    </p>
                    <p>
                        정직성 확인: 같은 시나리오에서 고정-500 입자 필터의 ≈ 0.057 대비 여기{" "}
                        <code>ate_rmse</code> ≈ 0.052 — KLD-sampling은 <em>같은</em> 정확도에서 표본 약 15배를 절약했다.
                        정확도는 애초에 표본이 사던 것이 아니었으니까(뭉툭한 <InlineMath math="\sigma_r"/> 우도가{" "}
                        <InlineMath math="\sigma_r/\sqrt{k}"/> 근처에 바닥을 깔고; 큰 <code>rpe_rmse</code> ≈ 1.05는 여전히
                        생존 가설이 결정되는 순간의 헤딩 점프다). KLD는 표본을 절약하지 정확도를 만들지 않는다.{" "}
                        <code>epsilon</code> 칩을 돌려라: <InlineMath math="\varepsilon = 0.01"/>이면 모든 상한이 열 배로
                        자란다(<InlineMath math="k=2"/>에 33 → 330) — 그리고 <code>max_particles</code>를 200으로 놓으면
                        천장이 보증을 조용히 덮어쓴다. 예산 위반의 모습이 저렇다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "corridor03_drift"}]}
                run={runMcl}
                params={{x0: 4.25, y0: 1.75, epsilon: 0.1, delta: 0.01, bin_xy: 0.5, bin_theta: 0.17453292519943295, n_min: 10, max_particles: 800, range_max: 2.5, sigma_range: 0.3, sigma_xy: 0.05, sigma_theta: 0.05, seed: 42}}
                chips={[{key: "epsilon", values: [0.1, 0.01]}, {key: "max_particles", values: [800, 200]}]}
                label={t(
                    "mcl live — the cloud size n(t) IS uncertainty: watch it collapse to 33 and spike at the U-turn seam",
                    "mcl 라이브 — 구름 크기 n(t) 그 자체가 불확실성이다: 33으로 붕괴하고 U-turn seam에서 솟구치는 것을 보라",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other operation
                    for operation: ascending draws from the splitmix64 stream, the strict-<code>&gt;</code> ancestor
                    walk, scan-order likelihood with max-shifted exps, floor-binned keys, scalar libm sin/cos/atan2/exp
                    on both sides — and the browser engine replays that contract with identical metrics. The bound's
                    quantile is Acklam's rational approximation with its branch structure pinned as part of the
                    contract (central branch symmetric bit-identically; tails via <code>sqrt(−2 log p)</code>). The
                    scenario's seed enters the algorithm as a declared param; sim noise and algorithm streams share the
                    value but stay independent streams.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: splitmix64 스트림의
                    오름차순 드로, strict <code>&gt;</code> 조상 워크, max-shifted exp와 스캔 순서 우도, floor 빈 키,
                    양쪽 모두 스칼라 libm sin/cos/atan2/exp — 그리고 브라우저 엔진은 그 계약을 같은 지표로 재생한다.
                    상한의 양자화는 Acklam 유리식이고 분기 구조 자체가 계약의 일부로 못 박혀 있다(중앙 분기는 비트 단위로
                    대칭, 꼬리는 <code>sqrt(−2 log p)</code> 경유). 시나리오의 seed는 선언된 파라미터로 알고리즘에
                    들어가고; 시뮬레이터 노이즈와 알고리즘 스트림은 값만 공유하고 독립 스트림으로 남는다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/filtering/mcl.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/filtering/mcl.py`,
                            },
                            {
                                name: "python/slam/core/stats.py",
                                code: pyStats,
                                href: `${REPO}/blob/main/python/slam/core/stats.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/filtering/mcl.hpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/include/slam/filtering/mcl.hpp`,
                            },
                            {
                                name: "cpp/src/filtering/mcl.cpp",
                                code: cppSrc,
                                href: `${REPO}/blob/main/cpp/src/filtering/mcl.cpp`,
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
                    D. Fox, W. Burgard, F. Dellaert, S. Thrun,{" "}
                    <a href="https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/"
                       target="_blank" rel="noopener noreferrer">
                        <em>Monte Carlo Localization: Efficient Position Estimation for Mobile Robots</em>
                    </a>,
                    Proc. AAAI, 1999 — the particle filter as robot localization; the sampling/importance-resampling
                    skeleton this page's filter still is.
                </li>
                <li>
                    D. Fox,{" "}
                    <a href="https://journals.sagepub.com/doi/10.1177/0278364903022012001" target="_blank"
                       rel="noopener noreferrer">
                        <em>Adapting the Sample Size in Particle Filters Through KLD-Sampling</em>
                    </a>,
                    Int. J. Robotics Research 22(12):985–1003, 2003 — the likelihood-ratio-to-KL argument, the{" "}
                    <InlineMath math="\chi^2_{k-1}"/> bound, and the experiment discretization (50 cm × 50 cm × 10°)
                    this repo's config copies.
                </li>
                <li>
                    E. B. Wilson, M. M. Hilferty,{" "}
                    <a href="https://www.pnas.org/doi/10.1073/pnas.17.12.684" target="_blank" rel="noopener noreferrer">
                        <em>The distribution of chi-square</em>
                    </a>,
                    Proc. National Academy of Sciences 17(12):684–688, 1931 — the cube-root approximation whose closed
                    form every implementation here evaluates literally.
                </li>
                <li>
                    G. W. Acklam,{" "}
                    <a href="https://web.archive.org/web/20151030215612/http://home.online.no/~pjacklam/notes/invnorm/"
                       target="_blank" rel="noopener noreferrer">
                        <em>Approximations of the inverse normal distribution function</em>
                    </a>
                    (archived) — the rational approximation behind{" "}
                    <InlineMath math="z_q = \Phi^{-1}(1-\delta)"/>.
                </li>
                <li>
                    S. Thrun, W. Burgard, D. Fox,{" "}
                    <a href="https://mitpress.mit.edu/9780262201629/probabilistic-robotics" target="_blank"
                       rel="noopener noreferrer">
                        <em>Probabilistic Robotics</em>
                    </a>,
                    MIT Press, 2005 — Ch. 4: the KLD-sampling pseudocode this page mirrors, including{" "}
                    <InlineMath math="n_{\min}"/> and the Wilson–Hilferty form of the bound.
                </li>
            </ol>
        </>
    )
}

export default MclPage
