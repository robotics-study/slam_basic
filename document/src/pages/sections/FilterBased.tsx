import {Layer, Shape, Stage} from "react-konva";
import {T, useLang, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import CanvasFigure from "../../components/CanvasFigure";
import {useCanvasColors} from "../../libs/useTheme";

// filter_based 갈래 소개 — 추정기 안에 지도를 넣는다. 정적 피겨: 불확실한 자세(타원)와
// 랜드마크들, 그리고 각 랜드마크로 뻗는 관측 선 — 증분 상태 SLAM의 그림 그대로.

const Figure = () => {
    const colors = useCanvasColors();
    const lang = useLang().lang;
    const t = (en: string, ko: string) => (lang === "ko" ? ko : en);
    return (
        <div className="flex flex-wrap items-start justify-center gap-6 my-5">
            <figure className="m-0">
                <Stage width={220} height={120} listening={false}>
                    <Layer>
                        {/* 관측 선: 추정 자세에서 랜드마크로 */}
                        <Shape listening={false} stroke={colors.muted} opacity={0.5} strokeWidth={1}
                               dash={[4, 3]} sceneFunc={(ctx, shape) => {
                               for (const [lx, ly] of [[60, 28], [170, 34], [150, 92]] as Array<[number, number]>) {
                                   ctx.beginPath();
                                   ctx.moveTo(96, 74);
                                   ctx.lineTo(lx, ly);
                                   ctx.fillStrokeShape(shape);
                               }
                           }}/>
                        {/* 랜드마크 × 마크 */}
                        <Shape listening={false} stroke="#ca8a04" strokeWidth={2} sceneFunc={(ctx, shape) => {
                            for (const [lx, ly] of [[60, 28], [170, 34], [150, 92]] as Array<[number, number]>) {
                                ctx.beginPath();
                                ctx.moveTo(lx - 4, ly - 4); ctx.lineTo(lx + 4, ly + 4);
                                ctx.moveTo(lx + 4, ly - 4); ctx.lineTo(lx - 4, ly + 4);
                                ctx.fillStrokeShape(shape);
                            }
                        }}/>
                        {/* 추정 자세: 점 + 오차 타원 (축 정렬) */}
                        <Shape listening={false} stroke={colors.accent} opacity={0.8} strokeWidth={1.2}
                               dash={[3, 3]} sceneFunc={(ctx, shape) => {
                               ctx.beginPath();
                               ctx.ellipse(96, 74, 22, 12, -0.4, 0, Math.PI * 2);
                               ctx.fillStrokeShape(shape);
                           }}/>
                        <Shape listening={false} fill={colors.accent} sceneFunc={(ctx, shape) => {
                            ctx.beginPath();
                            ctx.arc(96, 74, 5, 0, Math.PI * 2);
                            ctx.fillStrokeShape(shape);
                        }}/>
                    </Layer>
                </Stage>
                <figcaption className="text-xs text-muted text-center mt-1 block">
                    {t("one pose estimate, its covariance, and the landmarks it observes",
                       "하나의 자세 추정, 그 공분산, 그리고 그것이 관측하는 랜드마크들")}
                </figcaption>
            </figure>
        </div>
    );
};

// filter_based 갈래 소개 페이지.
const FilterBased = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The filtering branch kept the map known so you could see a filter work. This branch puts the map{" "}
                    <em>inside</em> the filter: the state grows until it contains what used to be given, and SLAM
                    becomes one joint posterior over pose and landmarks — or pose trajectory and grid. Everything
                    downstream is what "joint" costs and what structure buys back.
                </p>}
                ko={<p>
                    filtering 갈래는 필터가 작동하는 것을 보여 주려고 지도를 알려진 것으로 남겨 뒀다. 이 갈래는 그
                    지도를 필터 <em>안</em>으로 넣는다: 상태가, 예전에 알려진 것이었던 것까지 커지고, SLAM은 자세와
                    랜드마크 — 또는 자세 궤적과 격자 — 의 하나의 결합 사후분포가 된다. 이후의 모든 것은 "결합"이
                    요구하는 대가와 구조가 되돌려 주는 것이다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    The joint posterior <InlineMath math="p(x_t, m \mid z_{1:t}, u_{1:t})"/>. With landmarks as state,
                    EKF-SLAM carries the augmented vector and its covariance, and every observation updates{" "}
                    <em>everything</em>: the landmark's estimate moves and so does the pose belief, through the
                    off-diagonal blocks. That coupling is the whole point — it is what lets a re-observed landmark
                    correct an old drift — and it is also why the covariance grows with the map.
                </p>}
                ko={<p>
                    결합 사후분포 <InlineMath math="p(x_t, m \mid z_{1:t}, u_{1:t})"/>. 랜드마크를 상태로 두면
                    EKF-SLAM은 증분 벡터와 그 공분산을 싣고, 모든 관측이 <em>전부</em>를 갱신한다: 랜드마크의 추정이
                    움직이고, 비대각 블록을 통해 자세 belief도 함께 움직인다. 그 결합이 요점이다 — 다시 관측한
                    랜드마크가 오래된 드리프트를 정정하게 만드는 것이 이것이고, 공분산이 지도와 함께 커지는 이유도
                    이것이다.
                </p>}
            />

            <CanvasFigure label={"Augmented state: pose belief and landmark beliefs share one covariance"} tight>
                <Figure/>
            </CanvasFigure>

            <h2>{t("Rao-Blackwellization", "Rao-Blackwell화")}</h2>
            <T
                en={<p>
                    Here is the theorem that rescues the branch: conditioned on a pose trajectory, landmark dynamics
                    are linear-Gaussian — so the landmark posterior can be solved <em>exactly</em> by a Kalman filter.
                    The trajectory is the only thing worth sampling. That is Rao-Blackwellization: particles carry
                    trajectories, each particle carries exact landmark Gaussians, and the estimator marginalizes over
                    trajectories by weighting samples. FastSLAM 1 draws proposals from the prior; FastSLAM 2 draws
                    them from the measurement too, which is what kills the sample-count blowup on bearing-sparse
                    sensors.
                </p>}
                ko={<p>
                    갈래를 구하는 정리가 이것이다: 자세 궤적이 조건부이면 랜드마크 동선은 선형-Gaussian — 그래서
                    랜드마크 사후분포는 Kalman 필터가 <em>정확하게</em> 푼다. 표본으로 뽑을 가치는 궤적뿐이다.
                    이것이 Rao-Blackwell화다: 입자는 궤적을 싣고, 입자마다 정확한 랜드마크 Gaussian을 싣고, 추정기는
                    가중치로 궤적을 주변화한다. FastSLAM 1은 사전분포에서 제안을 뽑고, FastSLAM 2는 관측에서도 뽑는다 —
                    bearing이 드문 센서에서 표본 수 폭발을 죽이는 것이 후자다.
                </p>}
            />

            <h2>{t("The Grid Inside the Particle", "입자 안의 격자")}</h2>
            <T
                en={<p>
                    Replace each particle's landmark list with a log-odds grid — the filtering branch's mapping half,
                    carried per trajectory sample — and resample adaptively, only when the effective sample count
                    decays: that is GMapping. The scan matcher inside its proposal is this branch's registration
                    quietly doing its job, and the adaptive-resampling theorem (Grisetti, Stachniss &amp; Burgard) is
                    what makes the particle set honest instead of decorative.
                </p>}
                ko={<p>
                    입자마다 랜드마크 목록 대신 log-odds 격자를 싣는다 — filtering 갈래의 매핑 절반을 궤적 표본별로
                    옮기는 것 — 그리고 유효 표본 수가 줄어든 <em>때에만</em> 적응형으로 리샘플링한다: GMapping이다.
                    제안 분포 안에 들어간 스캔 매처는 이 갈래에서 registration이 조용히 일하는 모습이고, 적응형
                    리샘플링 정리(Grisetti, Stachniss &amp; Burgard)가 입자 집합을 장식용이 아니라 정직하게 만드는
                    것이다.
                </p>}
            />

            <h2>{t("Tight Coupling", "긴밀 결합")}</h2>
            <T
                en={<p>
                    The last members drop the wheel entirely: FAST-LIO feeds raw IMU acceleration through an iterated
                    Kalman filter whose observations are scan points themselves (residuals on keyframes), and folds
                    the corrected keyframe poses into a sliding-window iSAM2 graph; FAST-LIO2 drops the window too —
                    an incremental k-d tree becomes the map, updated point by point with no keyframe buffer. These two
                    are where this branch meets the registration branch: their odometry <em>is</em> ICP-style
                    alignment, running at IMU rate.
                </p>}
                ko={<p>
                    마지막 회원들은 바퀴를 아예 버린다: FAST-LIO는 날 IMU 가속도를 반복 Kalman 필터에 넣고 관측은
                    스캔 점 자체(키프레임 잔차)로 받으며, 정정된 키프레임 자세를 슬라이딩 창 iSAM2 그래프로 접는다;
                    FAST-LIO2는 창까지 버린다 — 증분 k-d 트리가 지도가 되어, 키프레임 버퍼 없이 점 단위로 갱신된다.
                    이 둘이 이 갈래와 registration 갈래가 만나는 지점이다: 그들의 오도메트리는 곧 ICP식 정렬이고,
                    IMU 속도로 돈다.
                </p>}
            />
        </>
    )
}

export default FilterBased
