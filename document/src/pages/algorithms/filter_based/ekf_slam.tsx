import {T, useTr} from "../../../libs/i18n"
import {BlockMath, InlineMath} from "../../../components/math/Tex"
import Pseudocode from "../../../components/Pseudocode"
import CodeTabs from "../../../components/CodeTabs"
import {SandboxScene} from "../../../components/panels/Sandbox"
import {runEkfSlam} from "../../../libs/algorithms/ekf-slam"
import pyImpl from "../../../../../python/slam/filter_based/ekf_slam.py?raw"
import cppImpl from "../../../../../cpp/src/filter_based/ekf_slam.cpp?raw"
import cppHeader from "../../../../../cpp/include/slam/filter_based/ekf_slam.hpp?raw"

const REPO = "https://github.com/robotics-study/slam_basic"

const EkfSlamPage = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the filter_based branch's first member, and it answers what every earlier page on this site
                    assumed away. The features branch produced <InlineMath math="(id, bearing, range)"/> observations with association{" "}
                    <em>given</em> — but what does an observation actually <strong>do</strong> once it is given? Here is the answer:
                    it augments the state. The state vector becomes{" "}
                    <InlineMath math="x = [p_x, p_y, \theta \mid l_{0x}, l_{0y}, l_{1x}, l_{1y}, \dots]"/> — the pose plus one (x, y) pair per
                    landmark discovered so far — and ONE dense covariance <InlineMath math="\Sigma"/> over all of it grows as landmarks are born.
                    The filtering branch taught belief over the POSE on a given map; grid mapping taught belief over the MAP given a pose;
                    this page welds both halves into a single Gaussian, exactly as Smith &amp; Cheeseman formulated it (1986). Association stays{" "}
                    <em>given</em>, honestly — as in the original paper: the id list arrives ready-made and its bookkeeping is part of the lesson.
                </p>}
                ko={<p>
                    이것은 filter_based 갈래의 첫 회원이고, 이 사이트의 모든 이전 페이지가 가정으로 넘긴 질문에 답한다. features 갈래는{" "}
                    <InlineMath math="(id, bearing, range)"/> 관측을 association이 <em>주어진 것</em>으로 소비했지만 — 주어진 관측은 실제로 무엇을{" "}
                    <strong>하는가</strong>? 답은 이것이다: 상태를 증강한다. 상태 벡터는{" "}
                    <InlineMath math="x = [p_x, p_y, \theta \mid l_{0x}, l_{0y}, l_{1x}, l_{1y}, \dots]"/> — 포즈에 더해 발견된 랜드마크마다 (x, y) 쌍 하나 —
                    그리고 그 전부를 싣는 조밀 공분산 <InlineMath math="\Sigma"/> 하나가 랜드마크가 태어날 때마다 자란다. filtering 갈래는 주어진 지도 위에서
                    <em>포즈</em>의 belief를 가르쳤고 grid mapping은 주어진 자세에서 지도의 belief를 가르쳤고, 이 페이지는 두 절반을 하나의 가우시안으로 용접한다 —
                    Smith &amp; Cheeseman(1986)이 정식화한 그대로. association은 정직하게 계속 <em>주어진다</em> — 원논문처럼: id 목록은 완성된 채로 도착하고 그
                    부기(id↔slot)는 수업의 일부다.
                </p>}
            />

            <h2>{t("The Map Lives Inside the Filter", "지도가 필터 안에 산다")}</h2>
            <T
                en={<>
                    <p>
                        The gauge keeps the convention this repo has kept since histogram_filter: the declared start pose IS known exactly, so{" "}
                        <InlineMath math="\Sigma"/> starts as the 3×3 ZERO matrix — pose uncertainty is honestly zero at t = 0 and grows ONLY from{" "}
                        <InlineMath math="Q"/>, the odometry noise the filter <em>declares</em> (the same values the scenario injects: declared, not injected).
                        The scenario is office01_landmarks: a straight eastward path along <InlineMath math="y = 4.625"/> from x = 0.875 to x = 10.125 in
                        0.4 m steps (25 steps, seed 42), with ten point landmarks at free-cell centers; a landmark is observable when the segment to it
                        crosses no occupied cell and its range is under 6 m.
                    </p>
                    <p>
                        Landmarks enter in OBSERVATION order, not id order — that bookkeeping is part of the lesson, not hidden by it. At t = 0 five landmarks
                        are already visible (ids {`{0, 2, 3, 4, 5}`} → slots 0…4; id 1 hides behind block A until the robot passes it at t = 3, arriving late as{" "}
                        <strong>slot 5</strong>); ids 6 and 8 are born after the door at t = 10 and t = 13, ids 7 and 9 only at t = 20 and t = 23. The final state
                        reads <InlineMath math="[0, 2, 3, 4, 5, 1, 6, 8, 7, 9]"/> in slot order. At t = 0 the emitted pose covariance is exactly{" "}
                        <InlineMath math="[\,0.0, 0.0, 0.0\,]"/> — and it stays exactly zero for any pose component whose Q entry is zero, because every product that
                        could move it multiplies those zeros (zero times finite IS exactly ±0 in IEEE arithmetic). That is dead reckoning itself, pinned by a unit test:
                        with <InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/> and noise-free odometry the estimated pose equals ground truth bit-for-bit.
                    </p>
                </>}
                ko={<>
                    <p>
                        게이지는 histogram_filter 이후 이 저장소의 관례 그대로다: 선언된 출발 자세는 정확히 알려졌으므로 <InlineMath math="\Sigma"/>는 3×3 영 행렬로
                        시작하고 — 포즈 불확실성은 t = 0에 정직하게 영이며 필터가 <em>선언한</em> 오도메트리 노이즈 <InlineMath math="Q"/>에서만 자란다(시나리오가 주입하는
                        값과 같은 값을 선언한다: 주입이 아니라 선언). 시나리오 office01_landmarks는 <InlineMath math="y = 4.625"/>를 따라 x = 0.875에서 10.125까지 동쪽으로
                        0.4 m씩 직진하는 25 스텝(시드 42)이고, free 셀 중심에 점 랜드마크 열 개가 있고, 랜드마크는 연결 선분이 occupied 셀을 지나지 않고 거리가 6 m 이내일 때
                        관측 가능하다.
                    </p>
                    <p>
                        랜드마크는 id 순서가 아니라 <em>관측 순서</em>로 들어오고 — 그 부기(id↔slot)는 숨기지 않는 수업의 일부다. t = 0에 다섯이 이미 보인다(ids {`{0, 2, 3, 4, 5}`}{" "}
                        → slot 0…4; id 1은 블록 A 뒤에 숨어 로봇이 지나치는 t = 3에 늦게 <strong>slot 5</strong>로 도착한다); id 6과 8은 문을 지난 뒤 t = 10과 t = 13에,
                        id 7과 9는 t = 20과 t = 23에 태어나고 최종 상태는 slot 순서로 <InlineMath math="[0, 2, 3, 4, 5, 1, 6, 8, 7, 9]"/>를 읽는다. t = 0에서 방출되는 포즈
                        공분산은 정확히 <InlineMath math="[\,0.0, 0.0, 0.0\,]"/>이고 — Q 항목이 영인 포즈 성분에서는 영원히 정확히 영으로 남는다: 그것을 움직일 수 있는 모든 곱이
                        그 영들을 곱하므로(IEEE 산술에서 영 × 유한은 정확히 ±0). 이것이 데드레커닝 자체이고 단위 테스트로 못 박혀 있다:{" "}
                        <InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/>에 노이즈 없는 오도메트리면 추정 포즈는 정본과 비트 단위로 같다.
                    </p>
                </>}
            />

            <h2>{t("Birth Is Not an Update", "출생은 갱신이 아니다")}</h2>
            <T
                en={<>
                    <p>
                        An unknown landmark is a state element with infinite variance, and the limit of the Kalman update in that limit is not an update. Let{" "}
                        <InlineMath math="\bar\Sigma"/> have block <InlineMath math="\lambda I"/> on the unknown landmark with cross blocks exactly ±0, and let{" "}
                        <InlineMath math="\lambda \to \infty"/>: the pose rows of K vanish (so the pose block does not move at all), and what survives is exactly two
                        objects — the mean becomes the observation back-projected through the current pose,{" "}
                        <InlineMath math="l = \mathrm{robot\_to\_world}((z_r \cos z_b,\; z_r \sin z_b), \hat x)"/>, the exact inverse of the measurement model, so this
                        observation's innovation is exactly 0 by construction and no mean correction could exist anyway — and the covariance grows by{" "}
                        <InlineMath math="B^{-1} W B^{-\top}"/> with cross columns <InlineMath math="-G B^{-\top}"/>, where B is the landmark block of the Jacobian, its
                        determinant <InlineMath math="1/\hat r \neq 0"/> — always invertible. (And this is why σ_range and σ_bearing keep a strictly positive floor in configs:
                        at t = 0, Σ is still the zero matrix so W collapses to R, and a zero σ would make it singular.) At t = 0 that new block is pure measurement noise
                        rotated into world frame and the cross columns are exactly ±0 there — but later births differ: by then Σ_pp has grown, so W carries the pose's own
                        uncertainty into the newcomer's block and −GB⁻ᵀ is nonzero — the newcomer inherits the pose uncertainty it will one day pay back.
                    </p>
                    <BlockMath math="B = \begin{bmatrix} \cos\beta & \sin\beta \\ -\sin\beta/\hat r & \cos\beta/\hat r \end{bmatrix}, \qquad W = H_p \Sigma_{pp} H_p^\top + R, \qquad \Sigma_{ll}^{\text{new}} = B^{-1} W B^{-\top}"/>
                    <p>
                        The birth ellipse is the measurement noise seen through geometry: id 0 is born at range ≈ 2.3 m and bearing ≈ 77.5°, so its pinned birth stds are{" "}
                        <InlineMath math="s_x = 0.0695 \approx r\sigma_\beta"/> (tangential) against <InlineMath math="s_y = 0.0513 \approx \sigma_r"/> (radial); id 4,
                        born at bearing ≈ 7°, is the mirror image — <InlineMath math="s_x = 0.0552 \approx \sigma_r"/> and <InlineMath math="s_y = 0.1706 \approx r\sigma_\beta"/>.
                        R is diagonal in MEASUREMENT space (metres, radians — different units, never isotropic in world frame);{" "}
                        <InlineMath math="B^{-1}RB^{-\top}"/> rotates that anisotropy into the world axes and re-observation squeezes it back: id 0's x std shrinks from its
                        birth 0.0695 to a final 0.0468 as later beams re-observe it.
                    </p>
                    <p>
                        A KNOWN id is the opposite event — a standard EKF update in Joseph form, and this is where the branch's thesis pays out: the observation's Jacobian now
                        has nonzero columns for the POSE slots too, so K carries information into the pose through the correlation blocks. Landmarks do not just get estimated,{" "}
                        <em>they estimate the pose back</em> — what graph_based will later restate as factor graphs. Measured on the default scenario: raw odometry drifts 0.7664 m
                        (ATE) over the run, but with Q declared at σ_xy = 0.05, σ_θ = 0.035 the re-observed landmarks pull the pose back to ATE 0.0408 / RPE 0.0530, and landmark
                        estimates land at rmse 0.0496. And the honest limit, pinned by test: declare σ_xy = σ_θ = 0 AND feed noise-free odometry — then every pose row of K is exactly
                        ±0 forever, no observation can ever correct the pose, every estimated pose equals ground truth bit-for-bit (ATE and RPE are EXACTLY zero), and landmarks alone
                        chase the pure observation floor at rmse 0.0310.
                    </p>
                </>}
                ko={<>
                    <p>
                        알려지지 않은 랜드마크는 분산이 무한인 상태 성분이고, 그 극한에서 칼만 갱신의 극한은 갱신이 아니다. <InlineMath math="\bar\Sigma"/>의 모르는 랜드마크
                        블록을 <InlineMath math="\lambda I"/>(크로스 블록은 정확히 ±0)로 두고 <InlineMath math="\lambda \to \infty"/>를 취하면: K의 포즈 행들은 사라지고(그래서
                        포즈 블록은 전혀 움직이지 않는다) 남는 것은 정확히 두 물건이다 — 평균은 관측을 현재 추정 포즈로 역투영한 것{" "}
                        <InlineMath math="l = \mathrm{robot\_to\_world}((z_r \cos z_b,\; z_r \sin z_b), \hat x)"/>, 측정 모델의 정확한 역이므로 이 관측의 혁신은 구성상 정확히 0이고
                        평균 보정은 애초에 존재할 수 없으며 — 공분산은 크로스 열 <InlineMath math="-G B^{-\top}"/>와 함께 <InlineMath math="B^{-1} W B^{-\top}"/>만큼 자란다.
                        B는 야코비안의 랜드마크 블록이고 그 행렬식은 <InlineMath math="1/\hat r \neq 0"/> — 언제나 가역이다. (그리고 config에서 σ_range과 σ_bearing의 최소가 0보다 큰
                        이유가 이것이다: t = 0에 Σ은 아직 영 행렬이라 W는 R로 쪼그라들고 σ가 0이면 특이가 된다.) t = 0에서 그 새 블록은 world 좌표로 회전된 순수 관측 노이즈이고 크로스 열은 거기서 정확히 ±0이다 — 하지만 이후의 출생은 다르다: 그때쯤 Σ_pp는 자라 있어 W가 포즈 자신의 불확실성을 신규 랜드마크 블록에 실어 나르고 −GB⁻ᵀ이 0이 아니다 — 새로 온 것은 언젠가 갚을 포즈 불확실성을 출생부터 상속받는다.
                    </p>
                    <BlockMath math="B = \begin{bmatrix} \cos\beta & \sin\beta \\ -\sin\beta/\hat r & \cos\beta/\hat r \end{bmatrix}, \qquad W = H_p \Sigma_{pp} H_p^\top + R, \qquad \Sigma_{ll}^{\text{new}} = B^{-1} W B^{-\top}"/>
                    <p>
                        출생 타원은 기하를 통해 본 관측 노이즈다: id 0은 거리 ≈ 2.3 m, 방위각 ≈ 77.5°에서 태어나 못 박힌 출생 표준편차가{" "}
                        <InlineMath math="s_x = 0.0695 \approx r\sigma_\beta"/>(접선)에 <InlineMath math="s_y = 0.0513 \approx \sigma_r"/>(반경), 문틀의 id 4는 그 거울상이다 —{" "}
                        <InlineMath math="s_x = 0.0552 \approx \sigma_r"/>에 <InlineMath math="s_y = 0.1706 \approx r\sigma_\beta"/>. R은 관측 공간에서 대각이다(미터, 라디안 —
                        단위가 다른 절대 등방이 아니다)이고 <InlineMath math="B^{-1}RB^{-\top}"/>가 그 비등방을 world 축으로 회전시켜 넣고, 재관측이 다시 조인다: id 0의 x 표준편차는
                        이후 빔이 재관측하며 출생 0.0695에서 최종 0.0468로 줄어든다.
                    </p>
                    <p>
                        알려진 id는 정반대 사건이다 — Joseph 형식의 표준 EKF 갱신, 그리고 여기서 갈래의 논지가 지불된다: 관측 야코비안의 포즈 열이 이제 0이 아니므로 K가 상관 블록을
                        통해 포즈로 정보를 나른다. 랜드마크는 단순히 추정당하는 것이 아니라 <em>포즈를 역으로 추정한다</em> — graph_based가 나중에 팩터 그래프로 다시 정식화할 그것.
                        기본 시나리오 실측: 날 오도메트리는 ATE 0.7664 m로 표류하지만 σ_xy = 0.05, σ_θ = 0.035를 선언하면 재관측된 랜드마크들이 포즈를 ATE 0.0408 / RPE 0.0530으로
                        끌어당기고 랜드마크 추정은 rmse 0.0496에 앉는다. 그리고 테스트로 못 박은 정직한 극한: σ_xy = σ_θ = 0을 선언하고 노이즈 없는 오도메트리까지 주면 K의 포즈 행은
                        영원히 정확히 ±0이고 어떤 관측도 포즈를 고칠 수 없으며 모든 추정 포즈는 정본과 비트 단위로 같아지고(ATE와 RPE는 정확히 영) 랜드마크만 순수 관측 바닥 rmse 0.0310을
                        쫓는다.
                    </p>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop below is the whole estimator, with the same fixed operation order as every page in this repo — ascending-index matmul products accumulating into a{" "}
                    <InlineMath math="+0.0"/> accumulator, every covariance product computing its upper triangle in ascending order and COPYING it below the diagonal (a copy is exact;
                    two independent ulp-different ways of computing the same entry would not be), an explicit 2×2 inverse by determinant and adjugate — no solver anywhere — and the{" "}
                    <em>Joseph form</em> for the update. The shortcut <InlineMath math="(I - KH)\bar\Sigma"/> is only ALGEBRAICALLY equal to{" "}
                    <InlineMath math="A\bar\Sigma A^\top + KRK^\top"/> when <InlineMath math="\bar\Sigma"/> was exact — and pinning bit-for-bit arithmetic is exactly a case where
                    “algebraically equal” is not equality. Bearings wrap like the simulator wraps them; range never wraps.
                </p>}
                ko={<p>
                    아래 루프가 추정기 전체이고, 이 저장소의 모든 페이지와 같은 고정 연산 순서를 지킨다 — 오름차순 인덱스 행렬곱이 <InlineMath math="+0.0"/> 누적기에 쌓이고, 모든 공분산
                    곱은 상삼각을 오름차순으로 계산해 대각 아래에 <em>복사</em>하고(복사는 정확하다 — 같은 성분을 울프 두 번 다른 방식으로 계산하면 같지 않다), 2×2 역행렬은 행렬식과 수반행렬로
                    명시적으로 — 어디에도 솔버 없다 — 갱신은 <em>Joseph 형식</em>. 지름길 <InlineMath math="(I - KH)\bar\Sigma"/>는 <InlineMath math="\bar\Sigma"/>가 정확했을 때만
                    대수적으로 <InlineMath math="A\bar\Sigma A^\top + KRK^\top"/>과 같고 — 비트 단위 산술을 못 박는 것이 바로 “대수적으로 같다”가 같음이 아닌 경우다. 방위각은 시뮬레이터가
                    감싸는 것처럼 감싸고 거리는 절대 감기지 않는다.
                </p>}
            />
            <Pseudocode code={`state  x = [px, py, θ | l₀x, l₀y, l₁x, …]        # pose + one (x,y) pair per landmark discovered so far
       Σ — ONE dense covariance over all of it; at t=0 the 3×3 ZERO matrix (the gauge is known)

step(t):
    t = 0: x ← gauge (x₀, y₀, θ₀); Σ ← zeros(3×3)        # pose uncertainty grows ONLY from Q
    t ≥ 1: predict — x̂ ← x ⊕ u (the SAME pose_compose the simulator ran on ground truth)
           J = [[1, 0, −(s·dx + c·dy)], [0, 1, c·dx − s·dy], [0, 0, 1]] at the PRIOR mean
           Σ ← FΣFᵀ + Q    (F = blockdiag(J, I) — landmark rows/cols are identity: a static
                            landmark needs no motion model; upper triangle ascending, lower COPIED)
    for each observation in ascending id:
        r̂ = √(dx² + dy²), β = atan2(dy, dx)               # at the CURRENT estimate
        KNOWN id → update:  y_r = z_r − r̂ (range never wraps); y_b = wrap(z_b − wrap(β − θ̂))
            G ← ΣHᵀ; S ← HΣHᵀ + R; K ← GS⁻¹               # explicit 2×2 inverse — no solver anywhere
            x += Ky componentwise (θ re-wrapped after its own add)
            Σ ← AΣAᵀ + KRKᵀ with A = I − KH                # Joseph form, not the shortcut
        NEW id → augmentation, NOT an update:
            mean ← robot_to_world((z_r cos z_b, z_r sin z_b), pose)   # innovation exactly 0 by construction
            H's columns for EVERY landmark slot are ZERO — birth cannot move the pose
            W ← HΣHᵀ + R; new block ← B⁻¹WB⁻ᵀ (det B = 1/r̂ ≠ 0); cross columns ← −GB⁻ᵀ (±0 at t=0)
    emit pose_estimated(cov = √diag Σ), then landmarks_updated in SLOT order (slot ≠ id — birth order)`}
            />
            <T
                en={<ol>
                    <li><strong>Birth is augmentation.</strong> The mean is the exact back-projection (innovation exactly 0 — an unknown landmark carries no information about
                        the pose), and the covariance grows by <InlineMath math="B^{-1}WB^{-\top}"/> with cross columns <InlineMath math="-GB^{-\top}"/>; at t = 0 those crosses are
                        exactly ±0 because Σ is still the zero matrix.</li>
                    <li><strong>The update is Joseph, not the shortcut:</strong> <InlineMath math="\Sigma \leftarrow A\Sigma A^\top + KRK^\top"/> with{" "}
                        <InlineMath math="A = I - KH"/>. Algebraically equal to <InlineMath math="(I-KH)\Sigma(I-KH)^\top + KRK^\top"/> — bit-for-bit not, and bit-for-bit is the contract.</li>
                    <li><strong>±0 is algebra:</strong> zero times finite IS exactly ±0. A component whose Q entry is 0 starts with an exactly-zero covariance row, and only F's
                        coupling terms (the j02/j12 entries carrying θ's variance into x/y) can ever make it nonzero — where nothing couples and nothing is declared, K's row stays
                        ±0 forever and no observation can correct that component: dead reckoning itself, not an approximation of it.</li>
                    <li><strong>Emission</strong>: pose_estimated first (cov = the Σ diagonal's square roots), landmarks_updated second in SLOT order with sx/sy — an honest trace of
                        what each landmark believes and how sure it is.</li>
                </ol>}
                ko={<ol>
                    <li><strong>출생은 증분이다.</strong> 평균은 정확한 역투영(혁신은 정확히 0 — 알려지지 않은 랜드마크는 포즈에 대한 정보를 싣지 않는다)이고 공분산은{" "}
                        <InlineMath math="B^{-1}WB^{-\top}"/>만큼 크로스 열 <InlineMath math="-GB^{-\top}"/>과 함께 자란다; t = 0에서 그 크로스들은 Σ가 아직 영 행렬이라 정확히 ±0이다.</li>
                    <li><strong>갱신은 지름길이 아니라 Joseph다:</strong> <InlineMath math="\Sigma \leftarrow A\Sigma A^\top + KRK^\top"/>, <InlineMath math="A = I - KH"/>.
                        대수적으로야 <InlineMath math="(I-KH)\Sigma(I-KH)^\top + KRK^\top"/>과 같지만 비트 단위로는 아니고 — 계약은 비트 단위다.</li>
                    <li><strong>±0은 대수다:</strong> 영 × 유한은 정확히 ±0이다. Q 항목이 0인 성분은 공분산 행이 정확히 영으로 시작하고 F의 결합 항(j02/j12가 θ의 분산을 x/y로 실어
                        나간다)만이 그것을 영이 아니게 할 수 있다 — 결합도 선언도 없는 곳에서 K의 그 행은 영원히 ±0이고 어떤 관측도 그 성분을 고칠 수 없다: 근사가 아니라 데드레커닝 그 자체.</li>
                    <li><strong>방출</strong>: pose_estimated 먼저(cov = Σ 대각의 제곱근), landmarks_updated 나중(slot 순서, sx/sy 포함) — 각 랜드마크가 무엇을 얼마나 확신하는지
                        정직한 trace다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox runs this exact estimator live — same operation order as the repository's Python and C++ (parity checked to{" "}
                        <InlineMath math="10^{-9}"/>). On office01_landmarks with the defaults, five landmarks are born at t = 0 in ascending id into slots 0…4, id 1 arrives late at
                        t = 3 as slot 5, and the summary reads ATE 0.0408 / RPE 0.0530 against raw odometry drift 0.7664 — that gap IS landmarks estimating the pose back — with{" "}
                        <InlineMath math="\text{landmark\_rmse} = 0.0496"/>.
                    </p>
                    <p>
                        Cycle the chips to see what actually does the work. At <code>sigma_xy</code> 0.05 → 0 the x/y covariance rows start at exactly ±0 and can only become nonzero
                        through coupling — F's j02/j12 terms carry θ's variance into them, so x and y are corrected only indirectly, via a heading that still gets corrected: ATE jumps
                        to 0.2224 and landmark rmse to 0.1520. At <code>sigma_theta</code> 0.035 → 0 the mirror holds — θ's row stays exactly ±0 forever (no Q entry, and F can never
                        manufacture θ variance out of nothing), no observation ever corrects heading, and its drift contaminates every birth and re-observation: landmark rmse explodes to
                        0.9443 while ATE reads 0.6409. Both at once: Σ stays the zero matrix forever, K is ±0 everywhere, pose is pure dead reckoning (ATE 0.7664) and landmarks are raw
                        back-projections through that drifting pose (rmse 0.9925). The chips change what the FILTER declares; the scenario keeps injecting its own noise either way.
                    </p>
                </>}
                ko={<>
                    <p>
                        아래 sandbox가 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++과 같은 연산 순서(패리티 <InlineMath math="10^{-9}"/>까지 검사). 기본값의
                        office01_landmarks에서 다섯이 t = 0에 id 오름차순으로 slot 0…4에 태어나고 id 1은 t = 3에 slot 5로 늦게 도착하며, 요약은 날 오도메트리 표류 0.7664 against ATE
                        0.0408 / RPE 0.0530 — 그 간격이 곧 랜드마크가 포즈를 역으로 추정하는 일이다 — <InlineMath math="\text{landmark\_rmse} = 0.0496"/>.
                    </p>
                    <p>
                        칩을 돌려 실제로 일이 일어나는 곳을 봐라. <code>sigma_xy</code> 0.05 → 0에서 x/y 공분산 행은 정확히 ±0으로 시작해 결합으로만 영이 아니게 될 수 있다 — F의 j02/j12
                        항이 θ의 분산을 그들 위로 실어 나르고, 그래서 x와 y는 여전히 보정되는 헤딩을 통해서만 간접적으로 고쳐진다: ATE는 0.2224로 랜드마크 rmse는 0.1520으로 뛴다.
                        <code>sigma_theta</code> 0.035 → 0에서는 거울상이 성립한다 — θ의 행은 영원히 정확히 ±0으로 남고(Q 항목 없고 F는 무에서 θ 분산을 만들 수 없다), 어떤 관측도 헤딩을
                        고치지 못하고 그 표류가 모든 출생과 재관측을 오염시켜 랜드마크 rmse는 0.9443으로 터지고 ATE는 0.6409를 읽는다. 둘 다 영이면 Σ는 영원히 영 행렬로 남고 K는 어디서나
                        ±0이고 포즈는 순수 데드레커닝(ATE 0.7664)이고 랜드마크는 그 표류하는 포즈로 되던진 날 역투영(rmse 0.9925)이다. 칩이 바꾸는 것은 필터가 <em>선언하는</em> 쪽이고
                        시나리오는 어느 쪽이든 자기 노이즈를 계속 주입한다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "office01_landmarks"}]}
                run={runEkfSlam}
                params={{x0: 0.875, y0: 4.625, theta_deg: 0, sigma_xy: 0.05, sigma_theta: 0.035, sigma_range: 0.05, sigma_bearing: 0.03}}
                chips={[{key: "sigma_xy", values: [0.05, 0]}, {key: "sigma_theta", values: [0.035, 0]}]}
                label={t(
                    "ekf_slam live — the map inside the filter: sigma_xy → 0 pins the pose x/y rows to exactly zero (ATE 0.0408 → 0.2224, only θ-coupling still corrects them), sigma_theta → 0 makes heading uncorrectable and every landmark pays for it (rmse 0.0496 → 0.9443)",
                    "ekf_slam 라이브 — 필터 안의 지도: sigma_xy → 0은 포즈 x/y 행을 정확히 영로 못 박고(ATE 0.0408 → 0.2224, θ 결합만이 여전히 고친다), sigma_theta → 0은 헤딩을 고칠 수 없게 만들어 랜드마크 전부가 그 대가를 치른다(rmse 0.0496 → 0.9443)",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other operation for operation: ascending-k matmuls into a +0.0 accumulator, upper
                    triangle computed in ascending order and copied below the diagonal, the same fixed-order sums inside S and K, θ re-wrapped after its own add. C++ routes every
                    same-argument (cos, sin) pair through the dlsym-resolved scalar libSystem pair — Apple clang's vectorized fold differs by 1 ulp on rare inputs — while atan2 and sqrt
                    need no routing; Python avoids the builtin sum() for the same compensated-summation reason as every page before it. The web engine is the third mirror, pinned to the
                    C++/Python pair within 10⁻⁹ by the parity checker. The scenario's seed enters the simulator only — the algorithm itself reads no parameters but the gauge and four noise values.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: +0.0 누적기로의 오름차순 k 행렬곱, 오름차순으로 계산해 대각 아래로 복사하는 상삼각, S와
                    K 안의 같은 고정 순서 합들, 자기 덧셈 뒤에 다시 감싸는 θ. C++은 같은 인수의 (cos, sin) 쌍을 전부 dlsym으로 해석된 스칼라 libSystem 쌍으로 라우팅한다 — Apple clang의
                    벡터화 폴드가 드문 입력에서 1 ulp 다르다 — atan2와 sqrt는 라우팅이 필요 없다(pairs 폴드가 없고 sqrt는 하드웨어 명령). python은 앞의 모든 페이지와 같은 보정 합산 이유로
                    내장 sum()을 피한다. 웹 엔진은 세 번째 미러이고 패리티 체커가 C++/Python 쌍에 10⁻⁹ 이내로 못 박는다. 시나리오의 seed는 시뮬레이터에만 들어가고 — 알고리즘 자체는 게이지와
                    노이즈 네 값 외의 파라미터를 읽지 않는다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/filter_based/ekf_slam.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/filter_based/ekf_slam.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/filter_based/ekf_slam.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/slam/filter_based/ekf_slam.hpp`,
                            },
                            {
                                name: "cpp/src/filter_based/ekf_slam.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/filter_based/ekf_slam.cpp`,
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
                    R. C. Smith and P. Cheeseman,{" "}
                    <a href="https://doi.org/10.1177/027836498600500404" target="_blank" rel="noopener noreferrer">
                        <em>On the Representation and Estimation of Spatial Uncertainty</em>
                    </a>,
                    International Journal of Robotics Research 5(4):56–68, 1986 — the original augmented-state formulation: one Gaussian over pose AND landmarks, and the cross-covariance
                    block that lets landmarks estimate the pose back.
                </li>
                <li>
                    H. Durrant-Whyte and T. Bailey,{" "}
                    <a href="https://doi.org/10.1109/MRA.2006.1638022" target="_blank" rel="noopener noreferrer">
                        <em>Simultaneous localization and mapping: part I</em>
                    </a>,
                    IEEE Robotics &amp; Automation Magazine 13(2):99–110, 2006 — the tutorial that states this branch's machinery in modern form, including the augmented-state update
                    this page implements (and its sparsity argument, which graph_based will cash in).
                </li>
            </ol>
        </>
    )
}

export default EkfSlamPage
