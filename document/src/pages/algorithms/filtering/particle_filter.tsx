import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import {SandboxScene} from "../../../components/panels/Sandbox";
import {runParticleFilter} from "../../../libs/algorithms/particle-filter";
import pyImpl from "../../../../../python/slam/filtering/particle_filter.py?raw";
import cppImpl from "../../../../../cpp/include/slam/filtering/particle_filter.hpp?raw";
import cppSrc from "../../../../../cpp/src/filtering/particle_filter.cpp?raw";

const REPO = "https://github.com/robotics-study/slam_basic"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const ParticleFilterPage = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the filtering branch's third step. The histogram filter spent one belief vector per{" "}
                    <em>lattice</em> state — it could enumerate free cells × heading bins, so exact Bayes was
                    cheap. A continuous pose <InlineMath math="(x, y, \theta)"/> cannot be enumerated. But the
                    recursion never required discreteness — it requires a <em>representation</em>, and here that
                    representation is samples: <InlineMath math="N"/> weighted particles, each one hypothesis,
                    and resampling replaces the marginalization the lattice could afford. Nothing else about the
                    Bayes filter changes: predict still pushes every hypothesis through the motion model, update
                    still multiplies in the sensor likelihood — the same beam model as the simulator's forward
                    model, evaluated at each particle's own pose. This is <strong>local</strong> localization:
                    the prior is uniform over one declared cell with heading fully unknown — and the budget is a
                    fixed lottery: too few samples and the true mode dies before it is ever drawn. How many samples
                    are enough, decided per step instead of at design time, is the next page's question
                    (MCL / KLD-sampling).
                </p>}
                ko={<p>
                    이것은 필터링 갈래의 세 번째 단계다. histogram filter는 belief 벡터를 <em>격자</em> 상태마다
                    하나씩 썼다 — 자유 셀 × heading 빈을 열거할 수 있으니 정확한 Bayes가 쌌다. 연속 자세{" "}
                    <InlineMath math="(x, y, \theta)"/>는 열거할 수 없다. 그러나 재귀는 이산성을 요구한 적이 없다
                    — 필요한 것은 <em>표현</em>이고, 여기서 그 표현은 표본이다: N개의 가중 입자, 하나하나가 가설이고,
                    격자가 감당했던 주변화를 리샘플링이 대체한다. Bayes 필터의 나머지는 아무것도 안 바뀐다: predict는
                    여전히 모든 가설을 운동 모델로 밀고, update는 여전히 센서 우도를 곱한다 — 시뮬레이터 순방향 모델과
                    같은 빔 모델을 입자 각자의 자세에서 평가한다. 이것은 <strong>지역</strong> 국소화다: 사전분포는
                    선언된 셀 하나 위에서 균일하고 heading은 전혀 모른다 — 그리고 예산은 고정된 경품 추첨이다: 표본이
                    너무 적으면 참된 모드는 그려지기도 전에 죽는다. 설계 시점이 아니라 스텝마다 표본이 몇 개면
                    충분한지 판정하는 것은 다음 페이지(MCL / KLD-sampling)의 질문이다.
                </p>}
            />

            <h2>{t("From a Table to Samples", "표에서 표본으로")}</h2>
            <T
                en={<>
                    <p>
                        The histogram filter's belief was a table: one probability per free cell × heading bin, and
                        the Bayes recursion ran on the table exactly. That works because a lattice is{" "}
                        <em>enumerable</em>. A pose has three continuous coordinates — no table covers them. What
                        the recursion actually needs is not a table but a representation you can (a) push through
                        the motion model and (b) multiply by the likelihood. Samples do both directly:
                    </p>
                    <BlockMath math="\mathrm{bel}_t(x) \;\approx\; \sum_{i=1}^{N} w^i_t\, \delta(x - x^i_t)"/>
                    <p>
                        Prediction becomes trivially exact — no discretization to apologize for: every sample just{" "}
                        <em>rides</em> the arriving command with its own drawn noise. This is what makes the filter{" "}
                        <strong>bootstrap</strong>: the proposal distribution is the motion model itself, no cleverness.
                        And expectations become weighted sums — the readout comes in the next section. What sampling pays for it:
                        a hypothesis only survives if its predicted scan actually matches the observed one (section 4),
                        and coverage of the state space by finitely many samples is a <em>lottery</em> whose odds you
                        will compute below. The cost per step is{" "}
                        <InlineMath math="O(N \cdot K)"/> raycasts for <InlineMath math="K"/> beams — independent of
                        the map's size, which is exactly what the lattice could not say.
                    </p>
                </>}
                ko={<>
                    <p>histogram filter의 belief는 표였다: 자유 셀 × heading 빈마다 확률 하나이고 Bayes 재귀를 표 위에서
                    그대로 돌렸다. 격자는 <em>열거 가능</em>하니 그게 통했다. 자세는 연속 좌표 셋 — 표가 덮을 수 없다.
                    그런데 재귀가 실제로 필요로 하는 것은 표가 아니라 (a) 운동 모델로 밀 수 있고 (b) 우도를 곱할 수 있는{" "}
                    <em>표현</em>이다. 표본은 둘을 직접 한다:</p>
                    <BlockMath math="\mathrm{bel}_t(x) \;\approx\; \sum_{i=1}^{N} w^i_t\, \delta(x - x^i_t)"/>
                    <p>예측은 원리적으로 정확해진다 — 사과할 이산화 따위 없으니까: 각 표본은 도착한 명령을 자기 노이즈와
                    함께 <em>태울</em> 뿐이다. 이것이 필터를 <strong>부트스트랩</strong>이라 부르는 이유다: 제안 분포가
                    운동 모델 그 자체다, 재주 없음. 그리고 기댓값은 가중합이 된다 — 판독은 다음 절에서. 표본이 치른 대가:
                    가설은 예측 스캔이 관측과 실제로 맞을 때만 살아남고(4절), 유한 표본의 상태공간 커버는 경품 추첨이고
                    그 확률은 아래에서 계산한다. 스텝당 비용은 <InlineMath math="K"/>빔에{" "}
                    <InlineMath math="O(N \cdot K)"/> 레이캐스트 — 지도 크기와 무관하고, 바로 그것이 격자가 할 수
                    없었던 말이다.</p>
                </>}
            />

            <h2>{t("The Bootstrap Recursion", "부트스트랩 재귀")}</h2>
            <T
                en={<>
                    <p>
                        <strong>Predict.</strong> Every particle rides the arriving command with its own drawn noise —
                        three gaussians per particle, in ascending particle order:
                    </p>
                    <BlockMath math="x^i \leftarrow x^i \oplus (u + \varepsilon),\qquad \varepsilon \sim \mathcal{N}(0,\ \operatorname{diag}(\sigma_{xy}^2,\ \sigma_\theta^2))"/>
                    <p>
                        The scenario commands exact twists (<InlineMath math="\sigma_{xy}=\sigma_\theta=0"/> on the
                        odometry side): the drift this branch teaches lives entirely{" "}
                        <em>inside</em> the filter's model — honest, because a filter that cannot see a noise source
                        must model it anyway.
                    </p>
                    <p>
                        <strong>Update.</strong> The scan points are robot-frame endpoints; each particle reconstructs
                        them in polar form and re-runs the simulator's own raycast at its own pose:
                    </p>
                    <BlockMath math="r = \sqrt{z_x^2 + z_y^2},\qquad \beta = \operatorname{atan2}(z_y, z_x),\qquad ll^i = \sum_{\text{beams}} -\tfrac{1}{2}\left(\frac{r - e(x^i,\ \theta^i + \beta)}{\sigma_r}\right)^2"/>
                    <p>
                        A beam that missed the map contributes the sentinel{" "}
                        <InlineMath math="e = \text{range}_{\max} + \mathrm{res}"/> — a point exists where this
                        particle sees nothing, so it pays the full penalty. Then the weights multiply by the shifted
                        likelihood and renormalize:
                    </p>
                    <BlockMath math="w^i \leftarrow \frac{w^i \exp(ll^i - m)}{\sum_j w^j \exp(ll^j - m)},\qquad m = \max_j ll^j"/>
                    <p>
                        The shift by <InlineMath math="m"/> never overflows (every factor is{" "}
                        <InlineMath math="\le 1"/>) and the argmax particle keeps its weight exactly. A total of zero
                        would make normalization impossible — that degenerate case leaves the belief unchanged, in all
                        three languages identically.
                    </p>
                </>}
                ko={<>
                    <p><strong>예측.</strong> 모든 입자는 도착한 명령을 자기 노이즈와 함께 태운다 — 입자마다 가우시안 셋,
                    입자 오름차순 순서로:</p>
                    <BlockMath math="x^i \leftarrow x^i \oplus (u + \varepsilon),\qquad \varepsilon \sim \mathcal{N}(0,\ \operatorname{diag}(\sigma_{xy}^2,\ \sigma_\theta^2))"/>
                    <p>시나리오는 정확한 twist를 명령한다(오도메트리 쪽 <InlineMath math="\sigma_{xy}=\sigma_\theta=0"/>):
                    이 갈래가 가르치는 드리프트는 전적으로 필터 모델 <em>안</em>에 산다 — 정직하다, 자기 노이즈원을
                    볼 수 없는 필터는 어쨌든 그것을 모델해야 하니까.</p>
                    <p><strong>갱신.</strong> 스캔 점은 로봇 프레임 끝점이고, 각 입자는 극좌표로 재구성한 뒤 자기 자세에서
                        시뮬레이터의 그 raycast를 다시 돌린다:</p>
                    <BlockMath math="r = \sqrt{z_x^2 + z_y^2},\qquad \beta = \operatorname{atan2}(z_y, z_x),\qquad ll^i = \sum_{\text{beams}} -\tfrac{1}{2}\left(\frac{r - e(x^i,\ \theta^i + \beta)}{\sigma_r}\right)^2"/>
                    <p>맵을 빗나간 빔은 sentinel <InlineMath math="e = \text{range}_{\max} + \mathrm{res}"/>를 기댓값으로
                    쓴다 — 이 입자는 아무것도 못 보는 곳에 점이 존재하므로 전체 페널티를 낸다. 이어 가중치가 이동된
                    우도로 곱해지고 정규화된다:</p>
                    <BlockMath math="w^i \leftarrow \frac{w^i \exp(ll^i - m)}{\sum_j w^j \exp(ll^j - m)},\qquad m = \max_j ll^j"/>
                    <p><InlineMath math="m"/> 이동은 절대 오버플로하지 않고(모든 인자는{" "}
                    <InlineMath math="\le 1"/>) argmax 입자는 가중치를 그대로 보존한다. 합이 0이면 정규화가 불가능하고 —
                    그 퇴화 케이스는 믿음을 그대로 두고, 세 언어에서 동일하게 그렇게 동작한다.</p>
                </>}
            />
            <T
                en={<>
                    <p>
                        <strong>Readout.</strong> Position is the weighted mean; heading is the circular mean — the
                        arithmetic mean of angles lies across the{" "}
                        <InlineMath math="\pm\pi"/> seam (the mean of <InlineMath math="3.1"/> and{" "}
                        <InlineMath math="-3.1"/> is <InlineMath math="0"/>, which is the worst possible guess for two
                        hypotheses that both point west), so heading goes through the sin/cos sums instead:
                    </p>
                    <BlockMath math="\hat x = \sum_i w^i x^i,\qquad \hat\theta = \operatorname{atan2}\Big(\sum_i w^i \sin\theta^i,\ \sum_i w^i \cos\theta^i\Big)"/>
                    <p>
                        The spread is the population standard deviation — <InlineMath math="x"/>,<InlineMath math="y"/>
                        {" "}plain, <InlineMath math="\theta"/> through <InlineMath math="\operatorname{wrap}"/> so the
                        seam never explodes it. The emitted covariance is the triple{" "}
                        <InlineMath math="[\sigma_x,\ \sigma_y,\ \sigma_\theta]"/> — and when the cloud is bimodal,
                        that heading spread is the honest picture of what the filter does not know.
                    </p>
                </>}
                ko={<>
                    <p><strong>판독.</strong> 위치는 가중 평균이고, 헤딩은 circular mean이다 — 각도의 산술 평균은{" "}
                    <InlineMath math="\pm\pi"/> seam을 넘어서 거짓말한다(<InlineMath math="3.1"/>과{" "}
                    <InlineMath math="-3.1"/>의 평균은 <InlineMath math="0"/>이고, 서쪽을 가리키는 두 가설에 대한 그것만큼
                    나쁜 추측은 없다) — 그래서 헤딩은 sin/cos 합으로 들어간다:</p>
                    <BlockMath math="\hat x = \sum_i w^i x^i,\qquad \hat\theta = \operatorname{atan2}\Big(\sum_i w^i \sin\theta^i,\ \sum_i w^i \cos\theta^i\Big)"/>
                    <p>퍼짐은 population 표준편차 — <InlineMath math="x"/>,<InlineMath math="y"/>는 그대로,{" "}
                    <InlineMath math="\theta"/>는 <InlineMath math="\operatorname{wrap}"/>을 통과해 seam에 폭발하지 않는다.
                    방출되는 공분산은 삼항 <InlineMath math="[\sigma_x,\ \sigma_y,\ \sigma_\theta]"/>이고, 구름이 이중봉일
                    때 그 heading 퍼짐이야말로 필터가 무엇을 모르는지에 대한 정직한 그림이다.</p>
                </>}
            />

            <h2>{t("Systematic Resampling", "체계적 리샘플링")}</h2>
            <T
                en={<>
                    <p>
                        Weighted importance sampling without resampling collapses: after a few steps one ancestor owns
                        everything and the cloud is dead. The trigger is the effective sample size — the inverse of the
                        weight concentration — and it fires strictly below half the budget:
                    </p>
                    <BlockMath math="\mathrm{ESS} = \frac{1}{\sum_i (w^i)^2},\qquad \text{resample when } \mathrm{ESS} < \frac{N}{2}"/>
                    <p>
                        The resampler is <em>systematic</em>, not multinomial: one uniform draw{" "}
                        <InlineMath math="u \in [0, 1/N)"/> anchors a ladder of thresholds{" "}
                        <InlineMath math="u + j/N"/>, and the ancestors are picked by walking the cumulative weights
                        with a strict <InlineMath math="\gt;"/>. One draw covers every stratum of{" "}
                        <InlineMath math="[0,1)"/> — that stratification is exactly why systematic resampling has less
                        variance than drawing multinomially. Afterward every weight is exactly <InlineMath math="1/N"/>:
                        the survivors start equal again, and only the next likelihood decides who matters. The loop's
                        final (unused) <InlineMath math="u \mathrel{+}= 1/N"/> is kept in all three languages so the
                        random stream advances identically everywhere.
                    </p>
                </>}
                ko={<>
                    <p>리샘플링 없이 가중 중요표본추출은 붕괴한다: 몇 스텝이면 조상 하나가 전부를 소유하고 구름은 죽는다.
                    트리거는 유효 표본 수 — 가중치 집중도의 역 — 그리고 예산의 절반 아래로 떨어질 때만 발동한다:</p>
                    <BlockMath math="\mathrm{ESS} = \frac{1}{\sum_i (w^i)^2},\qquad \text{resample when } \mathrm{ESS} < \frac{N}{2}"/>
                    <p>리샘플러는 multinomial이 아니라 <em>체계적</em>이다: 균일 드로 하나 <InlineMath math="u \in [0, 1/N)"/>가
                    문지방 사다리 <InlineMath math="u + j/N"/>을 고정하고, 조상은 누적 가중치를 strict{" "}
                    <InlineMath math="\gt;"/>로 걸어서 고른다. 드로 하나가 <InlineMath math="[0,1)"/>의 모든 층을 덮는다 —
                    multinomial로 그릴보다 분산이 작은 이유가 정확히 그 층화다. 이후 가중치는 전부 정확히{" "}
                    <InlineMath math="1/N"/>: 생존자는 다시 동등하게 시작하고, 다음 우도만이 누가 중요한지 결정한다. 루프의
                    마지막 (무사용) <InlineMath math="u \mathrel{+}= 1/N"/>은 난수 스트림이 모든 언어에서 동일하게
                    진행되도록 그대로 유지된다.</p>
                </>}
            />
            <Proof title={t("Proposition (ESS bounds)", "명제 (ESS 경계)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> For weights summing to 1,{" "}
                            <InlineMath math="1 \le \mathrm{ESS} \le N"/>; equality on the left iff one particle owns
                            everything, on the right iff all weights are equal.
                        </p>
                        <p>
                            <strong>Proof.</strong> Since <InlineMath math="w^i \le 1"/> we have{" "}
                            <InlineMath math="(w^i)^2 \le w^i"/> so <InlineMath math="\sum (w^i)^2 \le 1"/>, giving{" "}
                            <InlineMath math="\mathrm{ESS} \ge 1"/>; Cauchy–Schwarz on{" "}
                            <InlineMath math="1 = \sum w^i \cdot 1"/> gives{" "}
                            <InlineMath math="1 \le N \sum (w^i)^2"/>, giving <InlineMath math="\mathrm{ESS} \le N"/>.
                            Both equalities are exactly the equality cases of those two inequalities. So the trigger
                            <InlineMath math="\ \mathrm{ESS} < N/2"/> fires precisely when the cloud is more than
                            half-collapsed — and a uniform cloud never triggers it. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 합이 1인 가중치에 대해{" "}
                            <InlineMath math="1 \le \mathrm{ESS} \le N"/>; 좌변 등식은 하나가 전부를 소유할 때, 우변
                            등식은 모든 가중치가 같을 때만 성립한다.
                        </p>
                        <p>
                            <strong>증명.</strong> <InlineMath math="w^i \le 1"/>이므로{" "}
                            <InlineMath math="(w^i)^2 \le w^i"/>이고 따라서 <InlineMath math="\sum (w^i)^2 \le 1"/> →{" "}
                            <InlineMath math="\mathrm{ESS} \ge 1"/>; <InlineMath math="1 = \sum w^i \cdot 1"/>에
                            Cauchy–Schwarz를 쓰면 <InlineMath math="1 \le N \sum (w^i)^2"/> →{" "}
                            <InlineMath math="\mathrm{ESS} \le N"/>. 두 등식은 정확히 그 부등식들의 등호 성립 케이스다.
                            따라서 트리거 <InlineMath math="\ \mathrm{ESS} < N/2"/>는 구름이 절반 이상 붕괴했을 때만
                            발동하고 — 균일한 구름은 절대 발동하지 않는다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("Why σ Is Blunt Here", "왜 여기서 σ는 뭉툭한가")}</h2>
            <T
                en={<>
                    <p>
                        The scenario declares <InlineMath math="\sigma_r = 0.3"/> — five times the other pages' beam
                        noise, and that is a design decision, not sloppiness. A likelihood with small{" "}
                        <InlineMath math="\sigma_r"/> has support centimeters wide: for a particle to survive one scan,
                        its predicted ranges must land within a few centimeters of the observed ones at every beam.
                        Uniform samples over (cell × full heading) land inside that sliver with probability far below{" "}
                        <InlineMath math="1/N"/> — with sharp likelihoods and thousands of particles the filter locks
                        onto whichever single particle got lucky, seed by seed. That lottery <em>is</em> the sampling
                        lesson: coarse sampling needs a blunt likelihood (or more particles, or smarter proposals —
                        which is what the next page buys).
                    </p>
                    <p>
                        The price is honest too. A blunt likelihood scores every survivor nearly alike, so position
                        accuracy floors near{" "}
                        <InlineMath math="\sigma_r/\sqrt{k}"/> (the demo's ATE sits at ≈0.057 m), and whatever the
                        forward fan cannot see — the lateral offset inside the corridor slit — stays spread in the cloud
                        where the map gives no information. Watch the demo: at <InlineMath math="t=0"/> both east-facing
                        and west-facing particles survive (42 distinct ancestors, heading std ≈ 1.36 rad); it is the
                        first <em>eastward move</em> that kills the west blob — then the covariance collapses to float
                        noise and tracking continues at σ<InlineMath math="_{xy}"/> level. Cycling the{" "}
                        <code>n_particles</code> chip barely moves the metrics: here the likelihood, not N, dominates.
                    </p>
                </>}
                ko={<>
                    <p>시나리오는 <InlineMath math="\sigma_r = 0.3"/>을 선언한다 — 다른 페이지들 빔 노이즈의 다섯 배이고,
                    이것은 게으름이 아니라 설계 결정이다. 작은 <InlineMath math="\sigma_r"/>의 우도는 지지가 센티미터
                    폭이다: 입자가 한 스캔에서 살아남으려면 모든 빔에서 예측 거리가 관측과 수 cm 안에 들어가야 한다.
                    (셀 × 전체 heading) 위의 균일 표본은 그 가느다란 띠에 확률 <InlineMath math="1/N"/>보다 훨씬 작게
                    들어간다 — 날카로운 우도에 수천 입자면 필터는 운 좋은 입자 하나에 잠기고, 시드마다 다른 결론이
                    나온다. 그 경품 추첨이 바로 표본추출의 교훈이다: 거친 표본추출은 뭉툭한 우도(또는 더 많은 입자, 또는
                    더 영리한 제안 — 다음 페이지가 사는 것)를 필요로 한다.</p>
                    <p>대가도 정직하다. 뭉툭한 우도는 생존자를 거의 똑같이 점수 매기므로 위치 정확도는{" "}
                        <InlineMath math="\sigma_r/\sqrt{k}"/> 근처에서 바닥나고(데모의 ATE는 ≈0.057 m), 앞 부채꼴이 못
                        보는 것 — 복도 가늘고 긴 구역의 가로 방향 오프셋 — 은 지도가 정보를 주지 않으니 구름에 퍼진 채로
                        남는다. 데모를 봐라: <InlineMath math="t=0"/>에 동쪽 입자와 서쪽 입자가 같이 살고(42개 조상,
                        heading std ≈ 1.36 rad), 첫 <em>동쪽 이동</em>이 서쪽 덩어리를 죽인다 — 그때 공분산은 부동소수
                        노이즈로 붕괴하고 σ<InlineMath math="_{xy}"/> 수준의 추적이 계속된다. <code>n_particles</code>{" "}
                        칩을 돌려도 지표는 거의 안 움직인다: 여기서 지배적인 건 N이 아니라 우도다.
                    </p>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The whole estimator is one loop with a fixed operation order — ascending particle draws, scan-order
                    likelihood per particle, max-shifted exps accumulated and divided ascending, the resample walk with
                    its strict <InlineMath math="\gt;"/> and final unused increment — pinned so three languages reproduce
                    it bit for bit.
                </p>}
                ko={<p>
                    추정기 전체는 고정 연산 순서를 가진 루프 하나다 — 입자 오름차순 드로, 입자별 스캔 순서 우도,
                    max-shifted exp를 오름차순으로 쌓고 오름차순으로 나누고, strict <InlineMath math="\gt;"/>와 마지막
                    무사용 증분까지 있는 리샘플 워크 — 세 언어가 비트 단위로 재현하도록 못 박혀 있다.
                </p>}
            />
            <Pseudocode code={`init: for i = 1..N (ascending draws):
      x^i ← ox + (col₀ + u₁)·res          # the declared start cell, uniform inside it
      y^i ← oy + (h − 1 − row₀ + u₂)·res
      θ^i ← u₃·2π − π                     # heading: everything from −π to π
      w^i ← 1/N

predict(u):                                            # every particle rides the command
    for i = 1..N: ε ~ N(0, diag(σ_xy², σ_θ²)) drawn in order; x^i ← x^i ⊕ (u + ε)

update(z₁ … z_K):                                      # t = 0 has no u — the scan alone weights
    for each point z in scan order: r ← √(zx² + zy²); β ← atan2(zy, zx)
    for i = 1..N: ll^i ← Σ_beams −½·((r − raycast(x^i, y^i, θ^i + β)) / σ_r)²   # miss → sentinel
    m ← max over i of ll^i                             # exact — order-independent by construction
    total ← Σ (ascending) w^i · exp(ll^i − m); if total ≠ 0: w^i ← weighted / total

resample_if_effective():                               # systematic, Thrun's pseudocode 2.8
    if 1/Σ(w^i)² < N/2:
        u ← uniform(0,1) · (1/N); c ← w¹; i ← 0
        for j = 1..N: while i < N and u > c: i ← i+1; c ← c + w^i   # strict >
                      copy particle i into slot j; u ← u + 1/N       # last increment unused
        every w^j ← 1/N

readout: x̂ ← Σ w·x; ŷ ← Σ w·y; θ̂ ← atan2(Σ w·sin θ, Σ w·cos θ)
         spread = population std (θ deviations wrapped); emit pose+cov, then the cloud`}/>
            <T
                en={<ol>
                    <li><strong>Prior.</strong> Uniform over the declared start cell × full heading — position is known
                        to a cell, heading not at all. Three draws per particle in ascending order; weights exactly{" "}
                        <InlineMath math="1/N"/>.</li>
                    <li><strong>The likelihood is the simulator's own model</strong>: the same DDA raycast run at the
                        particle's pose along <InlineMath math="\theta^i + \beta"/>; a missed beam keeps the sentinel{" "}
                        <InlineMath math="\text{range}_{\max}+\mathrm{res}"/>.</li>
                    <li><strong>Resampling is not per-step</strong>: it fires only when ESS drops below{" "}
                        <InlineMath math="N/2"/>, and one uniform draw anchors the whole ladder — the walk's strict{" "}
                        <InlineMath math="\gt;"/> and even the final unused increment are part of the contract.</li>
                    <li><strong>Emission</strong>: pose_estimated (pose + diagonal stds) first, particles_updated
                        ([x, y, θ, w] per particle, ascending) second — at the same t.</li>
                </ol>}
                ko={<ol>
                    <li><strong>Prior.</strong> 선언된 출발 셀 × heading 전체 위에서 균일 — 위치는 셀 하나로 알려져 있고
                        방향은 전혀 모른다. 입자마다 드로 셋, 오름차순; 가중치는 정확히 <InlineMath math="1/N"/>.</li>
                    <li><strong>우도는 시뮬레이터의 그 모델이다</strong>: 입자 자세에서{" "}
                        <InlineMath math="\theta^i + \beta"/> 방향으로 돌리는 같은 DDA raycast; 빗나간 빔은 sentinel{" "}
                        <InlineMath math="\text{range}_{\max}+\mathrm{res}"/>을 기댓값으로 쓴다.</li>
                    <li><strong>리샘플링은 스텝마다가 아니다</strong>: ESS가 <InlineMath math="N/2"/> 아래로 떨어질 때만
                        발동하고, 균일 드로 하나가 사다리 전체를 고정한다 — 워크의 strict <InlineMath math="\gt;"/>와
                        마지막 무사용 증분까지가 계약의 일부다.</li>
                    <li><strong>방출</strong>: pose_estimated(자세 + 대각 표준편차)이 먼저, particles_updated(입자마다
                        [x, y, θ, w], 오름차순)가 나중 — 같은 t에서.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox below runs this exact estimator live in your browser — the same operation order as
                        the repository's Python and C++ (the parity checker compares their metrics down to{" "}
                        <InlineMath math="10^{-9}"/>). Drag cells to draw walls: the scan changes and the estimator
                        re-runs from step 0. The blue dots are the cloud, colored by{" "}
                        <InlineMath math="w \cdot N"/> — a dominant particle is dark blue, dead ones fade to nothing;
                        the teal trail is the readout. Watch the story: at <InlineMath math="t=0"/> the point-symmetric
                        start cell keeps both directions alive (42 ancestors, heading std ≈ 1.36 rad), the first eastward
                        move kills the west blob and the covariance collapses to float noise, then tracking rides on at{" "}
                        <InlineMath math="\sigma_{xy}"/> level — <code>ate_rmse</code> ≈ 0.057. The large{" "}
                        <code>rpe_rmse</code> (≈1.05) is honest too: it is dominated by the heading jump at the moment
                        the surviving hypothesis got decided, not by tracking error.
                    </p>
                    <p>
                        The only chip is <code>n_particles</code>. Cycle 150 ↔ 500 and watch the metrics barely move —
                        here the likelihood's bluntness dominates N (that is the lesson of section 4, not a bug). The
                        seed comes from the scenario; every draw is deterministic in all three languages.
                    </p>
                </>}
                ko={<p>
                    아래 sandbox는 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++과 같은 연산 순서이고
                    (parity 체커가 지표를 <InlineMath math="10^{-9}"/>까지 비교한다). 셀을 드래그해 벽을 그리면 스캔이
                    바뀌고 추정이 스텝 0에서 다시 돈다. 파란 점이 구름이고 <InlineMath math="w \cdot N"/>으로 색칠된다 —
                    지배적 입자는 진한 파랑, 죽은 입자는 사라지고; 청록색 트레일이 판독이다. 이야기를 봐라:{" "}
                    <InlineMath math="t=0"/>에 점대칭 출발 셀이 두 방향을 다 살리고(42 조상, heading std ≈ 1.36 rad),
                    첫 동쪽 이동이 서쪽 덩어리를 죽이면 공분산이 부동소수 노이즈로 붕괴하고, 이후 σ<InlineMath math="_{xy}"/>
                    {" "}수준의 추적이 이어진다(<code>ate_rmse</code> ≈ 0.057). 큰 <code>rpe_rmse</code>(≈1.05)도 정직하다 —
                    지배 가설이 결정되는 순간의 헤딩 점프가 지배하고 추적 오차가 아니다.
                </p>}
            />
            <SandboxScene
                presets={[{name: "corridor03_drift"}]}
                run={runParticleFilter}
                params={{n_particles: 500, x0: 4.25, y0: 1.75, range_max: 2.5, sigma_range: 0.3, sigma_xy: 0.05, sigma_theta: 0.05, seed: 42}}
                chips={[{key: "n_particles", values: [150, 500]}]}
                label={t(
                    "particle_filter live — dots are particles colored by w·N; watch the blob collapse when the first move breaks symmetry",
                    "particle_filter 라이브 — 점은 w·N으로 색칠한 입자; 첫 이동이 대칭을 깰 때 덩어리가 붕괴하는 것을 보라",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other operation
                    for operation: ascending draws from the splitmix64 stream, scan-order likelihood with max-shifted
                    exps, the strict-<code>&gt;</code> resample walk, scalar libm sin/cos/atan2/exp on both sides — and
                    the browser engine replays that contract with identical metrics, which is what lets you play with
                    the live sandbox above and trust it. The scenario's seed enters the algorithm as a declared param;
                    sim noise and algorithm streams share the value but stay independent streams.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: splitmix64 스트림의
                    오름차순 드로, max-shifted exp와 스캔 순서 우도, strict <code>&gt;</code> 리샘플 워크, 양쪽 모두
                    스칼라 libm sin/cos/atan2/exp — 그리고 브라우저 엔진은 그 계약을 같은 지표로 재생하고, 그래서 위
                    라이브 sandbox를 믿고 가지고 놀 수 있다. 시나리오의 seed는 선언된 파라미터로 알고리즘에 들어가고;
                    시뮬레이터 노이즈와 알고리즘 스트림은 값만 공유하고 독립 스트림으로 남는다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/filtering/particle_filter.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/filtering/particle_filter.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/filtering/particle_filter.hpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/include/slam/filtering/particle_filter.hpp`,
                            },
                            {
                                name: "cpp/src/filtering/particle_filter.cpp",
                                code: cppSrc,
                                href: `${REPO}/blob/main/cpp/src/filtering/particle_filter.cpp`,
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
                    N. J. Gordon, D. J. Salmond, A. F. M. Smith,{" "}
                    <em>Novel approach to nonlinear/non-Gaussian Bayesian state estimation</em>,
                    IEE Proceedings F 140(2):107–113, 1993 — the bootstrap filter: samples as the belief,
                    the motion model as the proposal.
                </li>
                <li>
                    F. Dellaert, D. Fox, W. Burgard, S. Thrun,{" "}
                    <a href="https://publications.ri.cmu.edu/storage/publications/pub_files/pub1/dellaert_frank_1999_2/dellaert_frank_1999_2.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Monte Carlo Localization for Mobile Robots</em>
                    </a>,
                    Proc. IEEE Int. Conf. on Robotics and Automation (ICRA), 1999 — the particle filter as robot
                    localization: sampling/importance resampling over the pose space.
                </li>
                <li>
                    D. Fox, W. Burgard, F. Dellaert, S. Thrun,{" "}
                    <a href="https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/"
                       target="_blank" rel="noopener noreferrer">
                        <em>Monte Carlo Localization: Efficient Position Estimation for Mobile Robots</em>
                    </a>,
                    Proc. AAAI, 1999 — the same algorithm's efficiency analysis; the resampling this page implements.
                </li>
                <li>
                    S. Thrun, W. Burgard, D. Fox,{" "}
                    <a href="https://mitpress.mit.edu/9780262201629/probabilistic-robotics" target="_blank"
                       rel="noopener noreferrer">
                        <em>Probabilistic Robotics</em>
                    </a>,
                    MIT Press, 2005 — Ch. 4: the Monte Carlo (particle) filter pseudocode this page mirrors, including
                    the systematic resampler and the sample-mean readout.
                </li>
            </ol>
        </>
    )
}

export default ParticleFilterPage
