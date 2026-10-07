import {lazy} from "react";
import {IAlgoData} from "../../../types/global";

// 알고리즘 메타데이터는 여기서만 관리한다. 콘텐츠 모듈은 lazy import 로 분리해 홈/목록
// 화면에서는 불러오지 않는다 (초기 번들 축소) — contents 는 해당 알고리즘의 페이지가
// 집필될 때 채워진다. sections 의 en/ko 문자열은 각 언어로 렌더된 본문 h2 헤딩과 정확히
// 일치해야 사이드바/TOC/검색 앵커(slug)가 맞는다.
// 배열 순서가 사이드바·pager의 학습 순서다 — 섹션별로 계보순: filtering 은 이산 belief →
// 격자 매핑 → 입자 → MCL(적응 표본 수까지), registration 은 점-점 → 분포, filter_based 는
// 증분 상태 EKF → Rao-Blackwell(FastSLAM) → 격자를 입자에 건 GMapping → 관성 긴밀 결합
// (FAST-LIO → ikd-tree FAST-LIO2), graph_based 는 랜드마크 소거(SPA) → 완전 그래프 →
// 서브맵 상관을 그 위에 얹은 Cartographer.
const data: IAlgoData[] = [
    // filtering — SLAM 의 두 절반을 분리해 가르치는 뿌리.
    {
        slug: "histogram_filter",
        title: {en: "Histogram Filter", ko: "Histogram Filter"},
        section: "filtering",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./filtering/histogram_filter")),
        sections: [
            {en: "From the Recursion to a Table", ko: "재귀에서 표로"},
            {en: "A Beam Scan as a Likelihood", ko: "빔 스캔을 우도로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "Exact by Construction", ko: "구성으로 정확하다"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    {
        slug: "grid_mapping",
        title: {en: "Log-Odds Grid Mapping", ko: "Log-Odds Grid Mapping"},
        section: "filtering",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./filtering/grid_mapping")),
        sections: [
            {en: "From Bayes per Cell to Log-Odds", ko: "셀별 Bayes 에서 log-odds 로"},
            {en: "A Beam Point Is Two Updates", ko: "빔 점은 두 개의 갱신이다"},
            {en: "The Ray Walks the Grid", ko: "광선은 격자를 걸어간다"},
            {en: "What No Beam Can Reach", ko: "어떤 빔도 닿을 수 없는 곳"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    {
        slug: "particle_filter",
        title: {en: "Particle Filter", ko: "Particle Filter"},
        section: "filtering",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./filtering/particle_filter")),
        sections: [
            {en: "From a Table to Samples", ko: "표에서 표본으로"},
            {en: "The Bootstrap Recursion", ko: "부트스트랩 재귀"},
            {en: "Systematic Resampling", ko: "체계적 리샘플링"},
            {en: "Why σ Is Blunt Here", ko: "왜 여기서 σ는 뭉툭한가"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    {
        slug: "mcl",
        title: {en: "Monte Carlo Localization", ko: "Monte Carlo Localization"},
        section: "filtering",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./filtering/mcl")),
        sections: [
            {en: "How Many Samples Are Enough?", ko: "표본은 얼마나 충분한가"},
            {en: "From Coverage to a Bound", ko: "커버에서 상한으로"},
            {en: "The Quantile in Closed Form", ko: "폐형이 된 양자화"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // registration — 바퀴 없는 오도메트리.
    {
        slug: "icp",
        title: {en: "Iterative Closest Point", ko: "Iterative Closest Point"},
        section: "registration",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./registration/icp")),
        sections: [
            {en: "Odometry Without Wheels", ko: "바퀴 없는 오도메트리"},
            {en: "The Closed Form in 2D", ko: "2D 에서 폐형으로"},
            {en: "Truncation, Aliasing, and the Capture Basin", ko: "절단, 알리아싱, 그리고 포획 영역"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    {slug: "ndt", title: {en: "Normal Distributions Transform", ko: "Normal Distributions Transform"}, section: "registration"},
    // features — 랜드마크는 어디서 오는가.
    {slug: "dbscan", title: {en: "DBSCAN", ko: "DBSCAN"}, section: "features"},
    // filter_based — 추정기 안에 지도를 넣는다.
    {slug: "ekf_slam", title: {en: "EKF-SLAM", ko: "EKF-SLAM"}, section: "filter_based"},
    {slug: "fastslam_1", title: {en: "FastSLAM 1", ko: "FastSLAM 1"}, section: "filter_based"},
    {slug: "fastslam_2", title: {en: "FastSLAM 2", ko: "FastSLAM 2"}, section: "filter_based"},
    {slug: "gmapping", title: {en: "GMapping", ko: "GMapping"}, section: "filter_based"},
    {slug: "fast_lio", title: {en: "FAST-LIO", ko: "FAST-LIO"}, section: "filter_based"},
    {slug: "fast_lio2", title: {en: "FAST-LIO2", ko: "FAST-LIO2"}, section: "filter_based"},
    // graph_based — 측정을 제약으로 모은다.
    {slug: "spa", title: {en: "Sparse Pose Adjustment", ko: "Sparse Pose Adjustment"}, section: "graph_based"},
    {slug: "graphslam", title: {en: "Graph SLAM", ko: "Graph SLAM"}, section: "graph_based"},
    {slug: "cartographer", title: {en: "Cartographer", ko: "Cartographer"}, section: "graph_based"},
];

export default data;
