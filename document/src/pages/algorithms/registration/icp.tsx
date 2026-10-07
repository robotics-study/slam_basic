import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import {SandboxScene} from "../../../components/panels/Sandbox";
import {runIcp} from "../../../libs/algorithms/icp";
import pyImpl from "../../../../../python/slam/registration/icp.py?raw";
import cppImpl from "../../../../../cpp/src/registration/icp.cpp?raw";
import cppHeader from "../../../../../cpp/include/slam/registration/icp.hpp?raw";

const REPO = "https://github.com/robotics-study/slam_basic"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const IcpPage = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the registration branch's first member, and it answers a question the filtering branch
                    never could. Every filter so far <em>rode on odometry</em>: the motion model was given, and the
                    sensor only corrected what the wheels (simulated here) had already integrated. Here the wheels are
                    removed entirely. There is no motion model to integrate and no map to match against either — that
                    is the graph_based branch's move later. What remains is the whole of registration:{" "}
                    <strong>two scans of the same walls, aligned</strong>. At every step the arriving scan is the{" "}
                    <em>source</em>, the previous scan is the <em>target</em>, and one solve returns the SE(2) element
                    that carries one onto the other; composing it onto the running estimate integrates motion into a
                    pose. This is Iterative Closest Point (Besl &amp; McKay, 1992) in its point-to-point form — run not
                    scan-to-map but <em>scan-to-scan</em>, pairwise, so nothing but measurements feeds the trajectory.
                </p>}
                ko={<p>
                    이것은 registration 갈래의 첫 회원이고, filtering 갈래가 끝내 답하지 못한 질문에 답한다. 여기까지
                    모든 필터는 <em>오도메트리 위에 올라탔다</em>: 운동 모델은 주어진 것이고 센서는 바퀴(여기선 모의)가
                    이미 적분한 것을 바로잡기만 했다. 여기서 바퀴는 완전히 제거된다. 적분할 운동 모델도 없고, 맞출 지도
                    없다 — 그건 나중에 graph_based 갈래의 수다. 남는 것은 등록의 전부다: <strong>같은 벽의 두 스캔을
                    정렬한다</strong>. 매 스텝 도착한 스캔은 <em>source</em>, 직전 스캔은 <em>target</em>이고, 해 하나가
                    하나를 다른 위에 얹는 SE(2) 원소를 돌려주고 추정 위에 합성하면 운동이 자세로 적분된다. 이것은
                    Iterative Closest Point(Besl &amp; McKay, 1992)의 점-점 형태 — scan-to-map 이 아니라 스캔-투-스캔으로,
                    스텝마다 쌍으로 굴려 측정 외에는 아무것도 궤적에 넣지 않는다.
                </p>}
            />

            <h2>{t("Odometry Without Wheels", "바퀴 없는 오도메트리")}</h2>
            <T
                en={<>
                    <p>
                        The absolute frame is not measured — it is <strong>declared</strong>. The parameters{" "}
                        <InlineMath math="x_0, y_0, \theta_0"/> are a gauge: the same convention grid_mapping anchored
                        its map on. Step 0 emits that declared pose unchanged (there is no earlier scan to align
                        against); every later step composes one recovered twist. The scenario declares{" "}
                        <InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/> for its odometry — not a fudge but logic:
                        the estimator never reads <InlineMath math="u_t"/>, so the command equals ground truth by
                        construction, and whatever the estimate drifts from ground truth is pure registration error.
                    </p>
                    <p>
                        The correspondence rule is the measurement model here. Each source point{" "}
                        <InlineMath math="q_j"/> (in the robot frame of its own scan) is transformed by the current
                        estimate and matched to the target point nearest in squared distance — ascending index scan,
                        strict{" "}
                        <InlineMath math="<"/>, a tie keeping the lower index — and the pair survives only if that
                        distance is at most <InlineMath math="d_{\max}"/>. Everything else follows mechanically: solve
                        for the rigid transform over the kept pairs, re-pair under the improved estimate, repeat until
                        the update stops moving. &quot;Iterative&quot; is not decoration — the fixed point of{" "}
                        <em>pair-then-solve</em> is what the algorithm means by &quot;the&quot; alignment.
                    </p>
                </>}
                ko={<>
                    <p>
                        절대 좌표계는 측정되지 않는다 — <strong>선언된다</strong>. 파라미터{" "}
                        <InlineMath math="x_0, y_0, \theta_0"/>는 게이지이고, grid_mapping 이 지도를 걸었던 그와 같은
                        관례다. 스텝 0 은 선언된 그 포즈를 그대로 방출하고(맞출 이전 스캔이 없다), 이후 모든 스텝은
                        복원된 twist 하나를 합성한다. 시나리오는 오도메트리에 <InlineMath math="\sigma_{xy} =
                        \sigma_\theta = 0"/>을 선언한다 — 정직함의 기만이 아니라 논리다: 추정기는 <InlineMath math="u_t"/>를
                        전혀 읽지 않으므로 명령은 구성상 참값과 같고, 추정이 참값에서 벗어난 정도가 곧 순수한 등록 오차다.
                    </p>
                    <p>
                        대응 규칙이 여기서는 측정 모델이다. 각 소스 점 <InlineMath math="q_j"/>(자기 스캔의 로봇 프레임)를
                        현재 추정 변환하고 제곱 거리 최근접 대상점에 짝짓는다 — 오름차순 인덱스 탐색, strict{" "}
                        <InlineMath math="<"/>, 타이는 낮은 인덱스 — 그리고 그 거리가 <InlineMath math="d_{\max}"/> 이하여야
                        쌍은 살아남는다. 나머지는 기계적으로 따라온다: 남은 쌍 위에서 강체 변환을 풀고, 개선된 추정 아래
                        다시 짝짓고, 갱신이 움직임을 멈출 때까지 반복한다. &quot;Iterative&quot;는 장식이 아니다 —{" "}
                        <em>대응한 뒤 푼다</em>의 고정점이 곧 이 알고리즘이 &quot;그&quot; 정렬이라 부르는 것이다.
                    </p>
                </>}
            />

            <h2>{t("The Closed Form in 2D", "2D 에서 폐형으로")}</h2>
            <T
                en={<>
                    <p>
                        Besl &amp; McKay minimize the point-to-point error over the whole rigid group at once — not
                        per-coordinate, and with no linearization anywhere:
                    </p>
                    <BlockMath math="(\hat R, \hat t) = \arg\min_{R \in SO(2),\, t} \sum_j \lVert R q_j + t - p_j \rVert^2"/>
                    <p>
                        Their general 3-D solution recovers the rotation as a unit quaternion (an eigenvalue
                        problem on a 4×4 matrix); Arun et al.'s equivalent closed form is an SVD of the cross-covariance — orthogonal
                        Procrustes. In 2D both machineries collapse to two lines, and this repo implements the collapse —
                        it is the same algorithm with fewer digits. The translation decouples: the energy
                        is minimized at <InlineMath math="t = \bar p - R\,\bar q"/> (centroids over the KEPT pairs),
                        leaving a single scalar to maximize over the rotation. With centered points{" "}
                        <InlineMath math="z_j = q_j - \bar q"/>, <InlineMath math="w_j = p_j - \bar p"/> and{" "}
                        <InlineMath math="R(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}"/>:
                    </p>
                    <BlockMath math="\max_\theta \sum_j \langle R z_j, w_j\rangle = \max_\theta \left[\cos\theta \underbrace{\textstyle\sum_j (z_x w_x + z_y w_y)}_{\text{den}} + \sin\theta \underbrace{\textstyle\sum_j (z_x w_y - z_y w_x)}_{\text{num}}\right]"/>
                    <BlockMath math="\hat\theta = \operatorname{atan2}(\text{num}, \text{den})"/>
                </>}
                ko={<>
                    <p>
                        Besl &amp; McKay 는 점-점 오차를 강체 군 전체에서 한 번에 최소화한다 — 좌표별로가 아니고, 어디에도
                        선형화 없이:
                    </p>
                    <BlockMath math="(\hat R, \hat t) = \arg\min_{R \in SO(2),\, t} \sum_j \lVert R q_j + t - p_j \rVert^2"/>
                    <p>
                        그들의 3D 일반 해는 회전을 유닛 사원수로 되찾고(4×4 행렬의 고윳값 문제), Arun 등의 동치 폐형은
                        교차공분산의 SVD — 직교 Procrustes 다. 2D 에서 두 기계 모두 두 줄로 접히고, 이 저장소는 그 접힌 형태를 구현한다 —
                        같은 알고리즘을 자릿수 적게 푼 것뿐이다. 병진은 분리된다: 에너지는 <InlineMath math="t = \bar p - R\,\bar q"/>에서 최소이고(중심은{" "}
                        <em>보존된</em> 쌍 위), 회전 위에 스칼라 하나만 남는다. 중심 잡힌 점 <InlineMath math="z_j = q_j - \bar q"/>,{" "}
                        <InlineMath math="w_j = p_j - \bar p"/>와 <InlineMath math="R(\theta)"/>에 대해:
                    </p>
                    <BlockMath math="\max_\theta \sum_j \langle R z_j, w_j\rangle = \max_\theta \left[\cos\theta \underbrace{\textstyle\sum_j (z_x w_x + z_y w_y)}_{\text{den}} + \sin\theta \underbrace{\textstyle\sum_j (z_x w_y - z_y w_x)}_{\text{num}}\right]"/>
                    <BlockMath math="\hat\theta = \operatorname{atan2}(\text{num}, \text{den})"/>
                </>}
            />

            <Proof title={t("Proposition (why the 2D solve is one atan2)", "명제 (왜 2D 해가 atan2 하나인가)")}>
                <T
                    en={<p>
                        Expand the inner product with <InlineMath math="Rz = (\cos\theta\, z_x - \sin\theta\, z_y,\; \sin\theta\, z_x + \cos\theta\, z_y)"/>:{" "}
                        <InlineMath math="\langle Rz_j, w_j\rangle = \cos\theta(z_x w_x + z_y w_y) + \sin\theta(z_x w_y - z_y w_x)"/> —
                        a sum of the form <InlineMath math="D\cos\theta + N\sin\theta"/>, whose maximum over{" "}
                        <InlineMath math="\theta"/> is at <InlineMath math="\hat\theta = \operatorname{atan2}(N, D)"/> (the
                        amplitude <InlineMath math="\sqrt{N^2+D^2}"/> is reached exactly there). Translation: substituting{" "}
                        <InlineMath math="t"/>'s optimum back in leaves exactly that sum — the centroid term contributes zero by
                        construction since <InlineMath math="\sum z_j = \sum w_j = 0"/>. That is the closed form collapsed to 2D: no
                        matrix decomposes, one atan2 carries the rotation, and both languages evaluate the same fixed expression
                        bit for bit.
                    </p>}
                    ko={<p>
                        <InlineMath math="Rz = (\cos\theta\, z_x - \sin\theta\, z_y,\; \sin\theta\, z_x + \cos\theta\, z_y)"/>를
                        내적에 전개하면 <InlineMath math="\langle Rz_j, w_j\rangle = \cos\theta(z_x w_x + z_y w_y) + \sin\theta(z_x w_y - z_y w_x)"/> —{" "}
                        <InlineMath math="D\cos\theta + N\sin\theta"/> 꼴의 합이고, <InlineMath math="\theta"/>에 대한 최대는 정확히{" "}
                        <InlineMath math="\hat\theta = \operatorname{atan2}(N, D)"/>에서 진폭 <InlineMath math="\sqrt{N^2+D^2}"/>에 닿는다.
                        병진: <InlineMath math="t"/>의 최적점을 되대면 정확히 그 합만 남고 — 중심항은 <InlineMath math="\sum z_j = \sum w_j = 0"/>이라
                        기여가 0. 이것이 SVD-for-2D 의 전부다: 행렬은 분해되지 않고, atan2 하나가 회전을 나르며, 두 언어가 같은
                        고정식을 비트 단위로 계산한다.
                    </p>}
                />
            </Proof>

            <h2>{t("Truncation, Aliasing, and the Capture Basin", "절단, 알리아싱, 그리고 포획 영역")}</h2>
            <T
                en={<>
                    <p>
                        Correspondence is where ICP lives or dies, and the honest version of this page says so twice.{" "}
                        <strong>Truncation first.</strong> A wall entering the fan has points whose physical twins are not in
                        the other scan at all; an untruncated sum would let those wrong pairs drag the least-squares mean
                        toward identity. So a pair survives only under <InlineMath math="d_{\max}"/> — Besl &amp; McKay's own
                        &quot;truncated least squares&quot;. The unit tests make the mechanism visible with two points: source{" "}
                        <InlineMath math="(1, 0.5), (9, 0.5)"/>, target <InlineMath math="(1.25, 0.5), (2, 0.5)"/> — at{" "}
                        <InlineMath math="d_{\max}=1"/> the far pair is truncated and the solve returns the kept pair's
                        displacement exactly (+0.25); widen to 20 and both contradictory pairs are kept and their least-squares
                        mean lands at −3.375: silently, confidently wrong. In the demo <InlineMath math="d_{\max}"/> is a
                        safety net, not a knob — any value from 0.4 to 2.0 leaves the metrics identical.
                    </p>
                    <p>
                        Second honesty: <strong>a lattice point closer than the displacement aliases</strong>. The scenario's
                        beams sample angle at 1°, so along a wall the points sit{" "}
                        <InlineMath math="\approx r\,\Delta\beta \approx 0.05\,\text{m}"/> apart — five times finer than the
                        0.25 m step. A source point on a wall parallel to the motion therefore has no honest pair under the{" "}
                        <em>identity</em> estimate: it matches some other lattice point of the same wall at nearly the same robot-frame
                        position, and that wrong pair contributes (current estimate + quantization noise), not truth. Only the end
                        wall — perpendicular to the motion — carries honest pairs from the first pass. Iteration is what rescues
                        this: every pass re-pairs under the improved estimate, and once the estimate comes within half a lattice
                        spacing of the truth, every point's nearest target point <em>is</em> its physical twin and the fixed point
                        sits at the true displacement. Measured with one iteration pinned: the recovered step is +0.11 instead of
                        +0.25 (the aliased pairs carry the estimate they are given) and ATE jumps 0.021 → 0.477. And when{" "}
                        <em>every</em> pair aliases — points spaced 1.0 apart, displacement 0.6 — the fixed point is honestly wrong:
                        +0.1, pinned in a test. ICP is a local method; the capture basin is its honest limit, not an implementation bug.
                    </p>
                    <p>
                        Which is also why this branch's scenario is an empty box. A fin in the corridor violates the pairing
                        condition at the passing step (the far face was occluded while it mattered — the fan points forward), and
                        measured there, the recovered displacement collapses to ≈ 0.0004 against a commanded +0.5: error equal to
                        the whole step. And an instantaneous 90° turn has no shared physical points at all —{" "}
                        <em>no</em> correspondence rule can recover it. A straight line is not a limitation nobody noticed; it is
                        this algorithm's honest minimum terrain. The residual floor is honest too: re-running the scenario with{" "}
                        <InlineMath math="\sigma_r = 0"/> lands at ATE ≈ 0.0032, not zero — beams sample angle discretely (1°), so even a noise-free scan quantizes where along a wall its points sit.
                    </p>
                </>}
                ko={<>
                    <p>
                        대응은 ICP 가 살고 죽는 자리이고, 이 페이지의 정직한 버전은 그걸 두 번 말한다. <strong>먼저 절단.</strong>{" "}
                        부채꼴에 들어오는 벽은 물리 쌍둥이가 다른 스캔에 아예 없는 점을 갖고, 절단하지 않은 합은 그 잘못된 쌍이
                        최소자승 평균을 정체(identity)로 끌어당기게 한다. 그래서 쌍은 <InlineMath math="d_{\max}"/> 아래만 살아남는다 —
                        Besl &amp; McKay 자신의 &quot;truncated least squares&quot;. 단위 테스트는 메커니즘을 점 두 개로 보여준다: 소스{" "}
                        <InlineMath math="(1, 0.5), (9, 0.5)"/>, 대상 <InlineMath math="(1.25, 0.5), (2, 0.5)"/> — <InlineMath math="d_{\max}=1"/>에서는
                        먼 쌍이 잘려 보존된 쌍의 변위를 정확히(+0.25) 돌려주고, 20 으로 넓히면 모순되는 둘 다 살아 최소자승 평균은 −3.375:
                        조용하고 자신 있게 틀린다. 데모에서 <InlineMath math="d_{\max}"/>는 안전장치이지 노브가 아니다 — 0.4 부터 2.0 까지
                        어떤 값도 지표를 동일하게 남긴다.
                    </p>
                    <p>
                        두 번째 정직함: <strong>변위보다 가까운 격자 점은 알리아싱된다</strong>. 시나리오 빔은 각도를 1°로 샘플하니 벽을
                        따라 점 간격은 <InlineMath math="\approx r\,\Delta\beta \approx 0.05\,\text{m}"/> — 0.25 m 스텝의 5분의 1 이다. 따라서
                        운동에 평행한 벽의 소스 점은 <em>정체</em> 추정 아래 정직한 쌍이 없다: 같은 벽의 다른 격자점 — 거의 같은 로봇 좌표
                        위치 — 과 짝지어지고, 그 잘못된 쌍은 참된 값이 아니라 (현재 추정 + 양자화 노이즈)를 나른다. 운동에 수직인 끝벽만
                        첫 패스부터 정직한 쌍을 나른다. 반복이 이것을 구한다: 매 패스가 개선된 추정 아래 다시 짝짓고, 추정이 참값의 격자
                        반간격 안으로 들어오는 순간 모든 점의 최근접 대상점이 <em>정말로</em> 자기 물리 쌍둥이가 되어 고정점이 참 변위에 앉는다.
                        반복을 1 로 못 박아 실측하면: 복원 스텝은 +0.25 대신 +0.11 (알리아싱된 쌍은 주어진 추정을 그대로 나른다) 그리고 ATE 는
                        0.021 에서 0.477 로 뛴다. 그리고 <em>모든</em> 쌍이 알리아싱되면 — 간격 1.0, 변위 0.6 — 고정점은 정직하게 틀린다: +0.1,
                        테스트에 못 박혀 있다. ICP 는 지역 방법이고, 포획 영역은 구현 버그가 아니라 그 정직한 한계다.
                    </p>
                    <p>
                        그래서 이 갈래의 시나리오가 빈 상자이기도 하다. 복도에 핀을 세우면 통과 스텝에서 대응 조건이 깨지고(정면 부채꼴에
                        가려졌던 먼 면), 실측하면 복원 변위는 명령 +0.5 에 ≈ 0.0004 — 오차가 스텝 전체와 같다. 그리고 순간 90° 회전은 공유
                        물리 점이 아예 없어서 <em>어떤</em> 대응 규칙으로도 복원 불가능하다. 직선은 아무도 못 알아낸 한계가 아니라, 이
                        알고리즘의 정직한 최소 지형이다. 잔차 바닥도 정직하다: 시나리오를 <InlineMath math="\sigma_r = 0"/>으로 재실행하면
                        ATE ≈ 0.0032, 0 은 아니다 — 빔은 각도를 이산 샘플(1°)하니 노이즈 없는 스캔도 벽 위 점의 위치를 양자화한다.
                    </p>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop below is the whole estimator, with the same fixed operation order as every page in this repo —
                    ascending scans, squared distances (no sqrt anywhere), strict{" "}
                    <InlineMath math="<"/> ties to the lower index, ascending centroid sums, one atan2, convergence checked on{" "}
                    <InlineMath math="\max(\lvert\Delta dx\rvert, \lvert\Delta dy\rvert, \lvert\Delta d\theta\rvert)"/> after each update —
                    pinned so three languages reproduce it bit for bit. Note what is absent: no noise model, no covariance on the
                    output. ICP carries no uncertainty; that is what the filter_based branch adds on top of this.
                </p>}
                ko={<p>
                    아래 루프가 추정기 전체이고, 이 저장소의 모든 페이지와 같은 고정 연산 순서를 지킨다 — 오름차순 스캔, 제곱 거리(어디에도
                    sqrt 없음), strict <InlineMath math="<"/> 타이 낮은 인덱스, 오름차순 중심 합, atan2 하나, 수렴은 매 갱신{" "}
                    <InlineMath math="\max(\lvert\Delta dx\rvert, \lvert\Delta dy\rvert, \lvert\Delta d\theta\rvert)"/>로 판정 — 세 언어가 비트
                    단위로 재현하도록 못 박혀 있다. 없는 것에 주목하라: 노이즈 모델 없고 출력에 공분산 없다. ICP 는 불확실성을 싣지 않고, 그 위에
                    얹는 것이 filter_based 갈래의 일이다.
                </p>}
            />
            <Pseudocode code={`solve(source, target, d_max, eps, max_iters):        # one pairwise registration — the whole algorithm
    if source empty or target empty: return identity      # no information, no motion
    (ax, ay, ang) ← 0                                    # the estimate starts at identity (relative!)
    repeat ≤ max_iters:
        for q in source (scan order):                     # fixed order — mirrors bit-for-bit
            q' ← R(ang)·q + (ax, ay)                      # cos first
            j* ← argmin_j |q' − target[j]|²  (ascending scan, strict < → tie keeps lower index)
            keep (q, target[j*]) iff |q' − target[j*]|² ≤ d_max²      # truncated least squares
        if nothing kept: return current estimate           # degenerate: carry it forward
        z_j ← q_j − q̄;  w_j ← p_j − p̄                    # centroids over KEPT pairs, ascending sums
        θ ← atan2( Σ z_x w_y − z_y w_x , Σ z_x w_x + z_y w_y )         # the closed form
        (ax, ay) ← p̄ − R(θ)·q̄ ;  ang ← θ
        stop when max(|Δdx|, |Δdy|, |Δθ|) ≤ eps           # fixed point of pair-then-solve
    return (ax, ay, ang)

run(step):
    t = 0: pose ← declared gauge (x₀, y₀, θ₀); prev ← scan   # the frame is a declaration, not a measurement
    t ≥ 1: twist ← solve(scan_t, prev_scan);  pose ← pose ⊕ twist   # Step.odom is NEVER read`}
            />
            <T
                en={<ol>
                    <li><strong>The gauge is declared.</strong> The absolute frame comes from{" "}
                        <InlineMath math="x_0/y_0/\theta_0"/> parameters — the same convention grid_mapping anchored its map on.
                        Registration measures relative motion only; step 0 emits the declared pose verbatim and every later pose
                        is composed from scans alone.</li>
                    <li><strong>Squared distances, no square roots:</strong> the comparison against{" "}
                        <InlineMath math="d_{\max}"/> happens on squared distance (computed once), and every sum in the closed form
                        runs in ascending point order — fixed so Python, C++ and TypeScript reproduce identical bits (the web engine
                        within parity tolerance).</li>
                    <li><strong>Convergence is a fixed point, not a guarantee:</strong> the loop stops when the update moves by at most{" "}
                        <InlineMath math="\varepsilon"/> in any component — which is exactly why max_iters exists and why the demo's{" "}
                        <code>max_iters</code> chip (64 → 1) shows iteration doing real work.</li>
                    <li><strong>Emission</strong>: pose_estimated per step, no cov field at all — an honest trace of a method that
                        carries no uncertainty model.</li>
                </ol>}
                ko={<ol>
                    <li><strong>게이지는 선언이다.</strong> 절대 좌표계는 <InlineMath math="x_0/y_0/\theta_0"/> 파라미터에서 오고 — grid_mapping 이
                        지도를 건 것과 같은 관례다. 등록은 상대 운동만 측정하고, 스텝 0 은 선언 포즈를 그대로 방출하며 이후 모든 자세는 스캔만으로 합성된다.</li>
                    <li><strong>제곱 거리, 제곱근 없음:</strong> <InlineMath math="d_{\max}"/> 비교가 제곱 거리에서(한 번 계산) 이루어지고, 폐형의 모든 합은
                        오름차순 점 순서로 돈다 — Python·C++·TypeScript 가 같은 비트를 재현하도록 고정되어 있다(웹 엔진은 패리티 허용 오차 안에서).</li>
                    <li><strong>수렴은 보장이 아니라 고정점이다:</strong> 갱신이 어떤 성분에서도 <InlineMath math="\varepsilon"/> 이하로 움직이면 멈춘다 —
                        max_iters 가 존재하는 정확히 그 이유이고, 데모의 <code>max_iters</code> 칩(64 → 1)이 반복의 실제 일을 보여주는 이유다.</li>
                    <li><strong>방출</strong>: 스텝마다 pose_estimated, cov 필드는 아예 없음 — 불확실성 모델을 싣지 않는 방법의 정직한 trace 다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox runs this exact estimator live — same operation order as the repository's Python and C++ (parity
                        checked to <InlineMath math="10^{-9}"/>). The straight empty room is not decoration: it is the terrain where every
                        source point has a physical twin in the target scan. Watch the estimated trajectory track ground truth with{" "}
                        <code>ate_rmse</code> ≈ 0.021 (RPE ≈ 0.019) — sub-cell by construction, and everything above the σ=0 floor of
                        ≈ 0.0032 is range noise over beam-angle quantization.
                    </p>
                    <p>
                        Cycle the chips to see what actually does the work:{" "}
                        <code>max_iters</code> 64 → 1 pins a single pass under identity — aliased lattice pairs carry the estimate they are
                        given (+0.11 at t=1 instead of +0.25) and ATE jumps to 0.477; iteration is a contraction toward the honest fixed
                        point, not polish. <code>d_max</code> 1.0 → 0.25 tightens truncation around the true pair distance and moves the
                        metrics only slightly (ATE ≈ 0.020) — confirming it is a safety net here, not a knob. The pinned unit tests show what
                        this scenario hides by design: one outlier pair with <InlineMath math="d_{\max}=20"/> drags the mean to −3.375, and a
                        lattice finer than the displacement pins the wrong fixed point (+0.1) exactly.
                    </p>
                </>}
                ko={<>
                    <p>
                        아래 sandbox 는 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++ 과 같은 연산 순서(패리티{" "}
                        <InlineMath math="10^{-9}"/>까지 검사). 직선 빈 방은 연출이 아니다: 모든 소스 점이 대상 스캔에 물리 쌍둥이를 가진 채로 남는 지형이다.
                        추정 궤적이 <code>ate_rmse</code> ≈ 0.021(RPE ≈ 0.019)로 참값을 따라가는 것을 봐라 — 구성상 셀 이하이고, σ=0 바닥 ≈ 0.0032 위는 전부
                        빔 각도 양자화 위의 거리 노이즈다.
                    </p>
                    <p>
                        칩을 돌려 실제로 일이 일어나는 곳을 봐라: <code>max_iters</code> 64 → 1 은 정체 아래 단일 패스를 못 박고 — 알리아싱된 격자 쌍은
                        주어진 추정을 그대로 나르고(t=1 에서 +0.25 대신 +0.11) ATE 는 0.477 로 뛴다; 반복은 정직한 고정점으로의 수축이지 마감이 아니다.{" "}
                        <code>d_max</code> 1.0 → 0.25 는 절단을 참 쌍 거리 근처로 조이고 지표를 살짝 움직일 뿐(ATE ≈ 0.020) — 여기선 노브가 아니라 안전장치임을
                        확인시킨다. 못 박힌 단위 테스트는 이 시나리오가 설계로 감춘 것을 보여준다: 이상치 쌍 하나에 <InlineMath math="d_{\max}=20"/>이면 평균이
                        −3.375 로 끌려가고, 변위보다 가는 격자는 잘못된 고정점(+0.1)을 정확히 못 박는다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "room01_straight"}]}
                run={runIcp}
                params={{x0: 1.25, y0: 1.75, theta_deg: 0, d_max: 1, eps: 1e-9, max_iters: 64}}
                chips={[{key: "d_max", values: [1.0, 0.25]}, {key: "max_iters", values: [64, 1]}]}
                label={t(
                    "icp live — scan-to-scan registration on an empty straight room: drop max_iters to 1 and watch what the iteration was doing (ATE 0.021 → 0.477)",
                    "icp 라이브 — 빈 직선 방에서 스캔-투-스캔 등록: max_iters 를 1 로 낮추면 반복이 하던 일이 그대로 보인다(ATE 0.021 → 0.477)",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other operation for
                    operation: ascending scan-order pairing with strict{" "}
                    <code>&lt;</code>, squared-distance comparisons against a precomputed <InlineMath math="d_{\max}^2"/> (no sqrt anywhere),
                    ascending centroid sums, one atan2 — and the browser engine replays that contract with identical metrics. The C++ side routes its cos/sin pairs through dlsym-resolved scalar libm calls (the same-argument pair is exactly what Apple clang folds into a SIMD sincos whose results differ from CPython's by 1 ulp on rare inputs); atan2 binds directly. The scenario's seed enters the simulator only — the algorithm itself reads no parameters but the gauge and the three solver knobs.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: 오름차순 스캔 순서 대응에 strict{" "}
                    <code>&lt;</code>, 미리 계산한 <InlineMath math="d_{\max}^2"/> 와의 제곱 거리 비교(어디에도 sqrt 없음), 오름차순 중심 합, atan2 하나 —
                    그리고 브라우저 엔진은 그 계약을 같은 지표로 재생한다. C++ 쪽은 cos/sin 쌍을 dlsym 으로 해석된 스칼라 libm 호출로 라우팅하고(같은 인자 쌍이
                    바로 Apple clang 이 SIMD sincos 로 접는 대상이고 드문 입력에서 CPython 과 1 ulp 다르다), atan2 는 직접 바인딩한다. 시나리오의 seed 는
                    시뮬레이터에만 들어가고 — 알고리즘 자체는 게이지와 풀어 세 개 외의 파라미터를 읽지 않는다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/registration/icp.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/registration/icp.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/registration/icp.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/slam/registration/icp.hpp`,
                            },
                            {
                                name: "cpp/src/registration/icp.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/registration/icp.cpp`,
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
                    P. J. Besl, N. D. McKay,{" "}
                    <a href="https://ieeexplore.ieee.org/document/121791" target="_blank" rel="noopener noreferrer">
                        <em>A method for registration of 3-D shapes</em>
                    </a>,
                    IEEE Trans. Pattern Analysis and Machine Intelligence 14(2):239–256, 1992 — the iterative
                    closest point algorithm itself: nearest-neighbour correspondence, the truncated least-squares
                    variant this page implements, and the monotone-convergence-to-a-local-minimum guarantee.
                </li>
                <li>
                    K. S. Arun, T. S. Huang, S. D. Blostein,{" "}
                    <a href="https://doi.org/10.1109/TPAMI.1987.4767965" target="_blank" rel="noopener noreferrer">
                        <em>Least-Squares Fitting of Two 3-D Point Sets</em>
                    </a>,
                    IEEE Trans. Pattern Analysis and Machine Intelligence PAMI-9(5):698–700, 1987 — the closed-form
                    least-squares solve (centroid alignment + optimal rotation) that in 2D collapses to the single
                    atan2 this repo evaluates literally.
                </li>
                <li>
                    S. Rusinkiewicz, M. Levoy,{" "}
                    <a href="https://ieeexplore.ieee.org/document/924423" target="_blank" rel="noopener noreferrer">
                        <em>Efficient variants of the ICP algorithm</em>
                    </a>,
                    Proc. International Conference on 3-D Digital Imaging and Modeling (3DIM), 145–152, 2001 — the
                    family's later lineage (point-to-plane, downsampling, k-d trees): what this page deliberately
                    does NOT do, keeping point-to-point exact enough to pin bit-for-bit.
                </li>
            </ol>
        </>
    )
}

export default IcpPage
