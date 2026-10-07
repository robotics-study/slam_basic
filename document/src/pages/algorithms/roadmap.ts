import {AlgoSection, Localized} from "../../../types/global";

// 알고리즘 한 줄 소개는 페이지를 집필할 때 여기에 함께 적는다 (집필 전 planned 항목은
// blurb 없이 제목만 — 홈 카드는 blurb 없는 항목을 dim + "soon"으로 보여 준다).
export interface AlgoBlurb {
    slug: string;
    blurb: Localized<string>;
}

export const ALGO_BLURBS: AlgoBlurb[] = [
    {
        slug: "histogram_filter",
        blurb: {
            en: "The root of the genealogy: belief as an actual table over free cells × heading bins — " +
                "exhaustive Bayes filtering, exact up to the discretization itself.",
            ko: "계보의 뿌리: 자유 셀 × heading 빈 위에 실제 표가 되는 belief — 이산화 자체까지를"
                + " 정확히 다루는 완전한 Bayes 필터링.",
        },
    },
    {
        slug: "grid_mapping",
        blurb: {
            en: "The other half: belief over the MAP on a given pose — one scalar log-odds per cell, every " +
                "beam endpoint adding +L where it lands and −L where it passes, unknown cells staying honestly unknown.",
            ko: "다른 절반: 주어진 자세 위의 지도에 대한 belief — 셀마다 스칼라 log-odds 하나, 빔 끝점은"
                + " 착지한 곳에 +L, 통과한 길 위에 −L을 더하고, 미지 셀은 정직하게 미지로 남는다.",
        },
    },
    {
        slug: "particle_filter",
        blurb: {
            en: "The same recursion when the state cannot be enumerated: N weighted samples carry the belief,"
                + " the motion model itself is the proposal (bootstrap), and systematic resampling replaces"
                + " the marginalization the lattice could afford — local localization on a declared cell.",
            ko: "상태를 열거할 수 없을 때의 같은 재귀: N개의 가중 표본이 belief를 나르고, 운동 모델 자체가"
                + " 제안 분포가 되고(bootstrap), 체계적 리샘플링이 격자가 감당했던 주변화를 대체한다 —"
                + " 선언된 셀 위의 지역 국소화.",
        },
    },
];

// 대분류 — 홈의 큰 섹션이자 사이드바 disclosure 단위. 계보의 축은 하나다: 추정의 매개체가
// 무엇이냐. filtering 은 두 절반(자세 belief, 지도 belief)을 분리해 가르치고, registration 은
// 바퀴 없는 오도메트리(측정 → 자세)를 가르치며, features 는 랜드마크가 어디서 오는지를
// 가르친다. filter_based 는 추정기 안에 지도를 넣고(free-space unknown 을 입자/확률로 번지게),
// graph_based 는 측정을 제약으로 모아 궤적 전체를 함께 푼다 — 그리고 마지막 두 갈래는 IMU 와
// 키프레임까지 삼킨다. 각 섹션 안의 알고리즘 배치는 항상 계보순. 소스 코드 트리
// (python/slam/<section>/)와 configs(<section>/<slug>.yaml)도 이 구분을 따른다.
export const SECTIONS: Array<{
    key: AlgoSection;
    title: Localized<string>;
    desc: Localized<string>;
}> = [
    {
        key: "filtering",
        title: {en: "Recursive Bayes Filtering", ko: "재귀 Bayes 필터링"},
        desc: {
            en: "SLAM's two halves, separated for teaching. Belief over the ROBOT pose on a " +
                "given map — discrete histogram filter first (exact, exhaustive), then the " +
                "bootstrap particle filter and Monte Carlo Localization — and belief over the " +
                "MAP given a known pose: independent per-cell log-odds occupancy. Everything " +
                "here consumes odometry + one sensor capability and emits per-step estimates.",
            ko: "SLAM 의 두 절반을 가르치기 위해 분리한다. 주어진 지도 위에서 로봇 자세의 " +
                "belief — 이산 histogram filter(정확하고 완전한 열거)로 시작해 bootstrap 입자 " +
                "필터와 Monte Carlo Localization 으로 — 그리고 알려진 자세에서 지도의 belief: " +
                "셀별 독립 log-odds 점유. 여기 있는 것은 전부 오도메트리와 센서 하나를 소비해 " +
                "스텝별 추정을 뱉는다.",
        },
    },
    {
        key: "registration",
        title: {en: "Scan Registration", ko: "스캔 등록"},
        desc: {
            en: "Pose from measurements alone — odometry without wheels. Two scans of the same " +
                "wall are aligned by finding correspondences and solving for the rigid " +
                "transform: point-to-point ICP in closed form, then Normal Distributions " +
                "Transform replacing points with Gaussians glued to cells. This is what the " +
                "LiDAR-inertial branch quietly runs on underneath.",
            ko: "측정만으로 자세를 구한다 — 바퀴 없는 오도메트리. 같은 벽의 두 스캔을 대응과 " +
                "강체 변환 폐형해로 정렬한다: 점-점 ICP 를 닫힌 형태로, 그다음 점을 셀에 붙인 " +
                "Gaussian 으로 바꾼 Normal Distributions Transform. 라이다-관성 갈래가 속으로 " +
                "이것 위에 서 있다.",
        },
    },
    {
        key: "features",
        title: {en: "Feature Extraction", ko: "특징 추출"},
        desc: {
            en: "Where landmarks come from. The landmark-based branches consume (id, bearing, " +
                "range) with association GIVEN — but the id list has to come from somewhere: " +
                "raw scan points density-clustered into stable candidates (DBSCAN), expanded " +
                "deterministically by point index order so ids stay identical across languages.",
            ko: "랜드마크는 어디서 오는가. 랜드마크 계열은 association 을 주어진 것으로 소비하지만 " +
                "그 id 목록은 어딘가에서 와야 한다: 날 스캔 점을 밀도 클러스터로 안정적인 후보를 " +
                "깎는다(DBSCAN). 점 인덱스 순서로 결정적으로 확장해야 언어를 넘어 id 가 동일하다.",
        },
    },
    {
        key: "filter_based",
        title: {en: "Filter-Based SLAM", ko: "필터 기반 SLAM"},
        desc: {
            en: "The map goes inside the estimator. Landmarks as augmented state (EKF-SLAM), then " +
                "Rao-Blackwellization: trajectory on particles, each landmark's position solved " +
                "exactly by a Kalman filter (FastSLAM 1/2); adaptive resampling plus one log-odds " +
                "grid per particle completes the grid branch inside the particle branch (GMapping). " +
                "Then tight coupling: no odometry at all, points straight into an iterated EKF " +
                "(FAST-LIO) and onto an incremental k-d tree (FAST-LIO2).",
            ko: "추정기 안에 지도를 넣는다. 랜드마크를 증분 상태로(EKF-SLAM), 이어서 Rao-Blackwell화: " +
                "궤적은 입자로, 랜드마크 위치는 각 Kalman 필터가 정확하게 푼다(FastSLAM 1/2). 적응형 " +
                "리샘플링과 입자당 log-odds 격자가 매핑 갈래를 입자 갈래 안에서 완성한다(GMapping). "
                + "그리고 긴밀 결합: 오도메트리 없이 점을 반복 EKF 로 직행(FAST-LIO), 증분 k-d 트리 위에 싣는다(FAST-LIO2).",
        },
    },
    {
        key: "graph_based",
        title: {en: "Graph-Based SLAM", ko: "그래프 기반 SLAM"},
        desc: {
            en: "Measurements stop being updates and become constraints. Keep every pose as a " +
                "variable and re-solve the whole trajectory at the end — landmarks marginalized " +
                "out into sparse pose adjustment (SPA) or kept as information-form variables in " +
                "full graph SLAM. Cartographer closes the loop by correlating submaps with branch " +
                "+ bound search, then hands the edges to exactly this machinery.",
            ko: "측정은 갱신이 아니라 제약이 된다. 모든 자세를 변수로 남기고 궤적 전체를 마지막에 " +
                "다시 푼다 — 랜드마크는 소거해 희소 자세 조정(SPA)으로, 또는 완전 그래프 SLAM 에서 " +
                "정보 형식 변수로 남긴다. Cartographer 는 submap 을 branch-and-bound 상관으로 닫고 " +
                "그 간선을 정확히 이 기계에 넘긴다.",
        },
    },
];
