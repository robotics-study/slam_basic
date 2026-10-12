import {T, useTr} from "../../../libs/i18n"
import {BlockMath, InlineMath} from "../../../components/math/Tex"
import Pseudocode from "../../../components/Pseudocode"
import CodeTabs from "../../../components/CodeTabs"
import {SandboxScene} from "../../../components/panels/Sandbox"
import {runDbscan} from "../../../libs/algorithms/dbscan"
import pyImpl from "../../../../../python/slam/features/dbscan.py?raw"
import cppImpl from "../../../../../cpp/src/features/dbscan.cpp?raw"
import cppHeader from "../../../../../cpp/include/slam/features/dbscan.hpp?raw"

const REPO = "https://github.com/robotics-study/slam_basic"

const DbscanPage = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    This is the features branch's first member, and it answers a question every earlier page on this site
                    quietly assumed away. The landmark-based branches (filter_based: EKF-SLAM, FastSLAM) consume{" "}
                    <InlineMath math="(id, bearing, range)"/> observations with association <em>given</em> — but the id list has to
                    come from somewhere. Here is where an id is BORN: raw beam-scan points are density-clustered into stable
                    candidates, each candidate's id its discovery order, its position the mean of its members. Like the
                    registration branch before it, this estimator measures <strong>no motion</strong>: the pose that lifts scan
                    points into world frame is the declared gauge composed with the scenario's odometry (
                    <InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/>, so the integrated pose equals ground truth bit-for-bit).
                    ATE and RPE read exactly 0 by construction — that is the lesson, not a defect: feature-based SLAM stands on
                    odometry it does not question, and this page shows what sits under the assumption. The only real number here
                    is <InlineMath math="\text{landmark\_rmse}"/> over ids present on both sides.
                </p>}
                ko={<p>
                    이것은 features 갈래의 첫 회원이고, 이 사이트의 모든 이전 페이지가 조용히 가정해 넘긴 질문에 답한다.
                    랜드마크 계열(filter_based: EKF-SLAM, FastSLAM)은 <InlineMath math="(id, bearing, range)"/> 관측을
                    association이 <em>주어진 것</em>으로 소비하지만 — 그 id 목록은 어딘가에서 와야 한다. 여기가 그 태어난 자리다:
                    날 빔 스캔 점을 밀도 클러스터로 묶어 안정적인 후보를 만들고, 후보마다 id는 발견 순서, 위치는 멤버의 평균이다.
                    앞선 registration 갈래처럼 이 추정기도 <strong>운동을 측정하지 않는다</strong>: 스캔 점을 world로 올리는 자세는
                    선언된 게이지에 시나리오 오도메트리를 합친 것(<InlineMath math="\sigma_{xy} = \sigma_\theta = 0"/>이라 적분한
                    포즈가 정본과 비트 단위로 같다)이고, ATE와 RPE는 구성상 정확히 0이다 — 이것은 교훈이지 결함이 아니다:
                    feature 기반 SLAM은 자신이 묻지 않는 오도메트리 위에 서 있고, 이 페이지는 그 가정 아래에 무엇이 있는지 보여준다.
                    여기서 유일한 실수 값은 양쪽에 있는 id에 대한 <InlineMath math="\text{landmark\_rmse}"/> 하나다.
                </p>}
            />

            <h2>{t("Where Landmarks Come From", "랜드마크는 어디서 오는가")}</h2>
            <T
                en={<>
                    <p>
                        The scenario is a wall-less field of three single-cell blobs at (4.75, 1.25), (6.25, 1.75) and (4.75, 2.25);
                        the ground-truth landmark list is written in bearing order from the start pose (A at −7.1°, B at 0°, C at +7.1°),
                        so <em>ground-truth id order equals scan-index order</em> and an id can be earned by being discovered first. The
                        robot walks straight east along{" "}
                        <InlineMath math="y = 1.75"/> in 0.25 m steps, and every step's beam hits (misses are no points at all) are lifted
                        into world frame with the pose and appended to one accumulating point list — step-major, beam-minor. Every step
                        re-clusters <em>all</em> accumulated points: a cluster seeded by an early index keeps its id forever, and a blob
                        whose cloud is still too sparse has no id at all until density crosses the threshold. That is what “stable
                        candidates” honestly means here — not a tracker, just order.
                    </p>
                    <p>
                        Measured with the default parameters: all three blobs are already core at step 0 (the fan's angular spacing at
                        ~4–5 m range is{" "}
                        <InlineMath math="r \cdot \Delta\theta \approx 0.07\,\text{m}"/> , comfortably chained by{" "}
                        <InlineMath math="\varepsilon = 0.2"/>), so ids 0, 1, 2 exist from the first frame in GT order, and the final centroids
                        sit on the blobs' <em>faces</em>: B's estimate lands at x = 5.9999 against the cell <em>center</em> 6.25 — a beam stops
                        at the face it hits, so half a cell of bias is baked into every landmark this branch will ever know. That half-cell
                        plus range noise is exactly what the pinned{" "}
                        <InlineMath math="\text{landmark\_rmse} = 0.2389"/> measures.
                    </p>
                </>}
                ko={<>
                    <p>
                        시나리오는 벽 없는 블롭 세 개의 필드 — (4.75, 1.25), (6.25, 1.75), (4.75, 2.25)의 셀 하나짜리 블롭들이고, 정본
                        랜드마크 목록은 출발 자세 기준 방위각 순서(A −7.1°, B 0°, C +7.1°)로 적혀 있어 <em>정본 id 순서가 스캔 인덱스
                        순서와 같다</em>. 그래서 id는 먼저 발견되어 얻을 수 있다. 로봇이 <InlineMath math="y = 1.75"/> 를 따라 동쪽으로
                        0.25 m씩 직진하고, 스텝마다의 빔 반사점(미스는 점조차 안 낸다)을 포즈로 world에 올려 하나의 누적 점 목록에
                        붙인다 — 스테프 우선, 빔 오름차순. 스텝마다 <em>누적된 모든</em> 점을 다시 군집한다: 이른 인덱스가 씨앗인
                        클러스터는 id를 영구히 지키고, 아직 드문 블롭은 밀도가 문턱을 넘는 순간까지 id가 아예 없다. “안정적인 후보”가
                        정직하게 뜻하는 것이 이것 — 트래커가 아니라 순서다.
                    </p>
                    <p>
                        기본 파라미터로 실측: 세 블롭 모두 스텝 0에서 이미 core 이다(4–5 m 거리에서 부채꼴 각간격{" "}
                        <InlineMath math="r \cdot \Delta\theta \approx 0.07\,\text{m}"/>를 <InlineMath math="\varepsilon = 0.2"/>가 여유로이
                        체인한다), 그래서 id 0, 1, 2가 첫 프레임부터 정본 순서로 존재하고, 최종 중심점들은 블롭의 <em>면</em> 위에 앉는다:
                        B의 추정은 셀 <em>중심</em> 6.25 앞에서 x = 5.9999 에 닿는다 — 빔은 맞은 면에서 멈추므로 반 셀 치우침이 이 갈래가
                        알게 될 랜드마크 전부에 굽혀 있다. 그 반 셀과 거리 노이즈가 못 박힌 <InlineMath math="\text{landmark\_rmse} = 0.2389"/>가
                        재는 것이다.
                    </p>
                </>}
            />

            <h2>{t("Density, Not Distance", "거리가 아니라 밀도")}</h2>
            <T
                en={<>
                    <p>
                        DBSCAN (Ester, Kriegel, Sander &amp; Xu, KDD 1996) asks no distance-to-a-center question. A point is{" "}
                        <em>core</em> iff its closed ball — the implementation compares squared distances,{" "}
                        <InlineMath math="dx^2 + dy^2 \le \varepsilon^2"/>; a sqrt appears nowhere in this module — holds at least{" "}
                        <InlineMath math="\text{min\_pts}"/> points <em>including itself</em>. A core point seeds a cluster and expands it: every
                        unvisited point reached joins the cluster, but only a reached point whose own neighborhood also reaches the threshold
                        extends the frontier. Border points join without expanding; points sparse at their visit moment are labeled noise,{" "}
                        <em>and noise is sticky</em>.
                    </p>
                    <BlockMath math="N_\varepsilon(p) = \{q : dx^2 + dy^2 \le \varepsilon^2\}, \qquad \text{core} \iff |N_\varepsilon(p)| \ge \text{min\_pts}, \qquad c_k = \frac{1}{n}\sum_{j \in C_k} x_j"/>
                    <p>
                        Stickiness is not an implementation accident — on static data, re-querying a noise point returns the same sub-threshold
                        neighborhood, so sticky-noise-by-visit-order equals the paper's pseudocode exactly. What the visit order does change is
                        membership itself, and the unit tests pin that arbitrariness rather than hide it: four collinear points one unit apart at{" "}
                        <InlineMath math="\varepsilon = 1.0, \text{min\_pts} = 3"/> in order ((0,0),(1,0),(2,0),(3,0)) give one cluster{" "}
                        [(1,0),(2,0),(3,0)] — (0,0) saw only two points at its visit and is noise forever. Reorder the same four points to
                        ((5,0),(0,0),(1,0),(2,0)) and now (0,0) <em>is</em> a member: order was the only difference. Cluster ids are the ascending
                        index of each cluster's first core point — not any spatial order — which is why they stay identical across all three
                        language implementations and stable across steps as points accumulate.
                    </p>
                </>}
                ko={<>
                    <p>
                        DBSCAN(Ester, Kriegel, Sander &amp; Xu, KDD 1996)은 중심까지 거리 질문을 하지 않는다. 한 점이 <em>core</em>인 조건은
                        닫힌 공이 — 구현은 제곱 거리를 비교한다, <InlineMath math="dx^2 + dy^2 \le \varepsilon^2"/>; sqrt는 이 모듈 어디에도
                        없다 — 자기 자신을 포함해 <InlineMath math="\text{min\_pts}"/> 개 이상을 담는 것이다. core 점은 클러스터의 씨앗이 되어
                        확장한다: 도달한 미방문 점은 모두 클러스터에 합류하지만, 자기 근방이 역시 문턱을 넘는 점(core)만 전선을 밀고 나간다.
                        경계 점은 확장 없이 합류하고, 방문 순간에 드물었던 점은 노이즈로 라벨되고,{" "}
                        <em>그리고 노이즈는 sticky하다</em>.
                    </p>
                    <BlockMath math="N_\varepsilon(p) = \{q : dx^2 + dy^2 \le \varepsilon^2\}, \qquad \text{core} \iff |N_\varepsilon(p)| \ge \text{min\_pts}, \qquad c_k = \frac{1}{n}\sum_{j \in C_k} x_j"/>
                    <p>
                        Sticky는 구현 사고가 아니다 — 정적 데이터에서 노이즈 점을 다시 조회하면 같은 문턱 아래 근방이 돌아오므로, 방문 순서
                        sticky 노이즈는 논문의 의사코드와 정확히 같다. 방문 순서가 실제로 바꾸는 것은 멤버십 자체이고, 단위 테스트는 그 자의성을
                        숨기지 않고 못 박는다: 간격 1인 공선 네 점을 순서 ((0,0),(1,0),(2,0),(3,0)),{" "}
                        <InlineMath math="\varepsilon = 1.0, \text{min\_pts} = 3"/> 으로 하면 클러스터 하나 [(1,0),(2,0),(3,0)] — (0,0)은 방문 때
                        둘밖에 못 봐 영원히 노이즈다. 같은 네 점을 ((5,0),(0,0),(1,0),(2,0))로 순서만 바꾸면 이번엔 (0,0)이 멤버다: 차이는 순서뿐이었다.
                        클러스터 id는 각 클러스터 첫 core 점의 오름차순 인덱스 — 공간 순서가 아니다 — 그래서 세 언어 구현 전부에서 동일하고 점이
                        쌓이며 스텝이 지나도 안정적이다.
                    </p>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The loop below is the whole estimator, with the same fixed operation order as every page in this repo — ascending-index visit
                    order (the accumulation order), squared-distance compares only, the growing member list walked by index pointer, centroids summed
                    with explicit <InlineMath math="+="/> loops in append order — pinned so Python, C++ and TypeScript reproduce identical values (the web
                    engine within parity tolerance). Note what is absent again: no noise model, no covariance on the output —{" "}
                    <InlineMath math="s_x/s_y"/> are simply not emitted. An id with an uncertainty attached is what filter_based adds on top of this.
                </p>}
                ko={<p>
                    아래 루프가 추정기 전체이고, 이 저장소의 모든 페이지와 같은 고정 연산 순서를 지킨다 — 오름차순 인덱스 방문(= 누적 순서), 제곱
                    거리 비교만, 인덱스 포인터로 걷는 성장 멤버 목록, append 순서대로 명시적 <InlineMath math="+="/> 루프로 더하는 중심점 — Python·C++·TypeScript가
                    같은 값을 재현하도록 못 박혀 있다(웹 엔진은 패리티 허용 오차 안에서). 없는 것에 다시 주목하라: 노이즈 모델 없고 출력에 공분산 없다 —{" "}
                    <InlineMath math="s_x/s_y"/>는 그냥 방출되지 않는다. id에 불확실성을 얹는 것이 filter_based 갈래가 이 위에 더하는 일이다.
                </p>}
            />
            <Pseudocode code={`cluster(points, eps, min_pts):                                   # Ester's Algorithm 1 — fixed operation order
    labels ← UNVISITED for all points                              # visit = ascending index (= accumulation order)
    for i ascending:
        if labels[i] ≠ UNVISITED: continue                         # already a member or already sticky noise
        N ← {j : dx² + dy² ≤ eps²}                                 # closed ball — no sqrt anywhere; self counts
        if |N| < min_pts: labels[i] ← NOISE; continue              # sparse at visit time → forever (sticky)
        k ← next id; members ← [p_i]; labels[i] ← k                # discovery order IS the id (seed first)
        j ← 0                                                      # ExpandCluster — growing list, index pointer
        while j < |N|:
            q ← N[j]; j ← j + 1
            if labels[q] ≠ UNVISITED: continue                     # member already (possibly twice) or sticky noise
            N_q ← neighborhood(q)
            if |N_q| ≥ min_pts:                                    # only a core point extends the frontier
                append every still-UNVISITED m ∈ N_q to N, ascending
            labels[q] ← k; members.append(p_q)                     # border points join without expanding
    return clusters                                                # NOISE points appear nowhere

run(step):
    t = 0: pose ← gauge (x₀, y₀, θ₀)                               # the absolute frame is a declaration …
    t ≥ 1: pose ← pose ⊕ odom_t                                    # … and this branch CONSUMES odometry — the opposite of registration
    for each scan point z in beam order: points.append(robotToWorld(z, pose))
    clusters ← cluster(points, eps, min_pts)                        # re-cluster ALL accumulated points, every step
    landmark k ← (Σx/n, Σy/n) over members in append order          # explicit += sums — no compensated sum anywhere
    emit pose_estimated(t, pose); emit landmarks_updated(t, landmarks)   # sx/sy: none — no uncertainty model here`}
            />
            <T
                en={<ol>
                    <li><strong>Association is born here.</strong> The id a landmark branch consumes is the cluster's discovery order (ascending seed
                        index), and its position is the member mean. Everything upstream of an (id, bearing, range) observation lives on this page.</li>
                    <li><strong>Closed ball, self-counting threshold:</strong> <InlineMath math="\ge"/> not <InlineMath math="&gt;"/> — a point exactly{" "}
                        <InlineMath math="\varepsilon"/> away IS a neighbor, and the center counts itself toward min_pts. Both are pinned by unit tests.</li>
                    <li><strong>This branch measures no motion:</strong> pose = declared gauge ⊕ scenario odometry, so with σ = 0 every estimated pose is
                        ground truth bit-for-bit and ATE/RPE are exactly 0. The contrast that proves the dependency: rebuild the same episode with{" "}
                        <InlineMath math="\sigma_{xy} = \sigma_\theta = 5"/> and the world-frame accumulation shatters — 33 scattered clusters instead of three,
                        ATE 12.878, landmark_rmse 0.2514. Registration never reads odom; this branch lives on it.</li>
                    <li><strong>Emission</strong>: pose_estimated first, landmarks_updated second — no cov field, no sx/sy, an honest trace of a method that
                        carries no uncertainty model.</li>
                </ol>}
                ko={<ol>
                    <li><strong>association은 여기서 태어난다.</strong> 랜드마크 계열이 소비하는 id는 클러스터의 발견 순서(씨앗 인덱스 오름차순)이고 위치는
                        멤버 평균이다. (id, bearing, range) 관측의 상류 전부가 이 페이지에 있다.</li>
                    <li><strong>닫힌 공, 자기 자신을 세는 문턱:</strong> <InlineMath math="\ge"/>이지 <InlineMath math="&gt;"/>가 아니다 — 정확히{" "}
                        <InlineMath math="\varepsilon"/>만큼 떨어진 점은 이웃이고, 중심은 min_pts에 자기를 센다. 둘 다 단위 테스트로 못 박혀 있다.</li>
                    <li><strong>이 갈래는 운동을 측정하지 않는다:</strong> 포즈 = 선언된 게이지 ⊕ 시나리오 오도메트리, 그래서 σ = 0이면 추정 포즈 전부가
                        정본과 비트 단위로 같고 ATE/RPE는 정확히 0. 의존을 증명하는 대조 실측: 같은 에피소드를{" "}
                        <InlineMath math="\sigma_{xy} = \sigma_\theta = 5"/>로 다시 만들면 world 좌표 누적은 산산이 부서진다 — 세 개 대신 흩어진 클러스터 33개,
                        ATE 12.878, landmark_rmse 0.2514. registration은 odom을 전혀 읽지 않고, 이 갈래는 그것 위에 살아 있다.</li>
                    <li><strong>방출</strong>: pose_estimated 먼저, landmarks_updated 나중 — cov 필드 없음, sx/sy 없음 — 불확실성 모델을 싣지 않는 방법의
                        정직한 trace다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<>
                    <p>
                        The sandbox runs this exact estimator live — same operation order as the repository's Python and C++ (parity checked to{" "}
                        <InlineMath math="10^{-9}"/>). On blobs01_line with the defaults, all three clusters exist at step 0 in GT id order, the centroids
                        converge onto the blob faces (B at x = 5.9999 against center 6.25), and the summary reads ATE = RPE = exactly 0 with{" "}
                        <InlineMath math="\text{landmark\_rmse} \approx 0.2389"/> — the half-cell face bias plus σ_range, nothing else.
                    </p>
                    <p>
                        Cycle the chips to see what actually does the work: at <code>eps</code> 0.2 → 0.75 the two blobs 1 m apart in y chain into ONE cluster —
                        its merged centroid sits at (4.518, 1.75), estimated id 0 now means “A∪C”, B slides to id 1, GT's C has no estimate at all, and{" "}
                        <InlineMath math="\text{landmark\_rmse}"/> jumps to 0.4279: the density threshold IS identity here. At <code>min_pts</code> 4 → 16 nothing is
                        core at step 0 — ids are born late (steps 2, 2 and 3 as the robot closes distance) — yet the final landmarks barely move (rmse 0.2404):
                        the threshold decides when a candidate exists, not what it converges to.
                    </p>
                </>}
                ko={<>
                    <p>
                        아래 sandbox가 이 추정기를 브라우저에서 라이브로 돌린다 — 저장소의 Python/C++과 같은 연산 순서(패리티{" "}
                        <InlineMath math="10^{-9}"/>까지 검사). blobs01_line 기본값에서 세 클러스터 모두 스텝 0에 정본 id 순서로 존재하고, 중심점들은 블롭의
                        면으로 수렴하며(B가 중심 6.25 앞에 x = 5.9999), 요약은 ATE = RPE 정확히 0에 <InlineMath math="\text{landmark\_rmse} \approx 0.2389"/> —
                        반 셀 면 편향에 σ_range를 더한 것, 그 이상도 이하도 아니다.
                    </p>
                    <p>
                        칩을 돌려 실제로 일이 일어나는 곳을 봐라: <code>eps</code> 0.2 → 0.75에서 y로 1 m 떨어진 두 블롭이 하나의 클러스터로 체인된다 — 합쳐진
                        중심점은 (4.518, 1.75)에 앉고 추정 id 0은 이제 “A∪C”를 뜻하며 B는 id 1로 밀리고 정본 C엔 추정이 아예 없어{" "}
                        <InlineMath math="\text{landmark\_rmse}"/>는 0.4279로 뛴다: 여기서 밀도 문턱이 곧 정체성이다. <code>min_pts</code> 4 → 16에서는 스텝 0에
                        core가 아무것도 없다 — id는 늦게 태어나고(로봇이 거리를 좁히며 스텝 2, 2, 3) 그래도 최종 랜드마크는 거의 안 움직인다(rmse 0.2404):
                        문턱은 후보가 언제 존재하게 되는지를 결정하지, 무엇이로 수렴하는지는 아니다.
                    </p>
                </>}
            />
            <SandboxScene
                presets={[{name: "blobs01_line"}]}
                run={runDbscan}
                params={{x0: 0.75, y0: 1.75, theta_deg: 0, eps: 0.2, min_pts: 4}}
                chips={[{key: "eps", values: [0.2, 0.75]}, {key: "min_pts", values: [4, 16]}]}
                label={t(
                    "dbscan live — density clustering on the blob field: widen eps to 0.75 and A∪C chain into one cluster (id misalignment, rmse 0.2389 → 0.4279), raise min_pts to 16 and every id is born late (rmse 0.2404)",
                    "dbscan 라이브 — 블롭 필드에서 밀도 군집: eps를 0.75로 넓히면 A∪C가 한 클러스터로 체인되고(id 어긋남, rmse 0.2389 → 0.4279), min_pts를 16으로 올리면 모든 id가 늦게 태어난다(rmse 0.2404)",
                )}
            />

            <h2>Implementation</h2>
            <T
                en={<p>
                    The two implementations below are the real sources, not excerpts. They mirror each other operation for operation: ascending-index visit
                    order over the accumulated point list (step-major, beam-minor), squared-distance compares with no sqrt anywhere, the growing member list
                    walked by index pointer with the label set only after the append loop (so a core q re-appends itself and the duplicate is skipped — pinned in
                    both languages), centroids summed with plain left-to-right <InlineMath math="+="/>. The Python side deliberately avoids the builtin sum(): from
                    CPython 3.12 (gh-100425) that is Neumaier compensated summation, a different arithmetic expression than the loops the C++ mirror runs — and
                    unlike every branch before it this module needs no libm probes at all: its own arithmetic is four operations; cos/sin live only in the shared
                    geometry helpers. The scenario's seed enters the simulator only — the algorithm itself reads no parameters but the gauge and the two clustering knobs.
                </p>}
                ko={<p>
                    아래 두 구현은 발췌가 아니라 실제 소스 그대로다. 연산 단위로 서로를 미러링한다: 누적 점 목록(스테프 우선, 빔 오름차순) 위의 오름차순 인덱스
                    방문, sqrt가 어디에도 없는 제곱 거리 비교, append 루프 뒤에야 라벨을 찍는 인덱스 포인터 성장 목록(그래서 core인 q가 자기를 다시 append 하고
                    중복은 건너뛴다 — 양 언어에 못 박혀 있다), plain 좌결합 <InlineMath math="+="/>로 더하는 중심점. python 쪽은 내장 sum()을 일부러 피한다: CPython 3.12(gh-100425)부터
                    그건 Neumaier 보정 합이고, C++ 미러가 도는 루프와 다른 산술 표현이다 — 그리고 앞의 모든 갈래와 달리 이 모듈은 libm 프로브 자체가 필요 없다: 자체 산술은
                    네 연산이고 cos/sin은 공유 기하 헬퍼 안에만 산다. 시나리오의 seed는 시뮬레이터에만 들어가고 — 알고리즘 자체는 게이지와 군집 노브 두 개 외의 파라미터를 읽지 않는다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/slam/features/dbscan.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/slam/features/dbscan.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/slam/features/dbscan.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/slam/features/dbscan.hpp`,
                            },
                            {
                                name: "cpp/src/features/dbscan.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/features/dbscan.cpp`,
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
                    M. Ester, H.-P. Kriegel, J. Sander, X. Xu,{" "}
                    <a href="https://cdn.aaai.org/KDD/1996/KDD96-037.pdf" target="_blank" rel="noopener noreferrer">
                        <em>A density-based algorithm for discovering clusters in large spatial databases with noise</em>
                    </a>,
                    Proc. 2nd International Conference on Knowledge Discovery and Data Mining (KDD), 226–231, 1996 — the algorithm itself:
                    eps-neighborhoods, core/border/noise, density-reachability, and the pseudocode whose visit-order arbitrariness this repo
                    pins instead of hiding.
                </li>
                <li>
                    H.-P. Kriegel, P. Kröger, J. Sander, A. Zimek,{" "}
                    <a href="https://doi.org/10.1002/widm.30" target="_blank" rel="noopener noreferrer">
                        <em>Density-based clustering</em>
                    </a>,
                    WIREs Data Mining and Knowledge Discovery 1(3):231–240, 2011 — the family's own survey: why density-connectivity (not
                    distance-to-a-center) is the definition, and what stickiness, ordering and parameter sensitivity cost; exactly the questions this page
                    answers with pinned numbers instead of adjectives.
                </li>
            </ol>
        </>
    )
}

export default DbscanPage
