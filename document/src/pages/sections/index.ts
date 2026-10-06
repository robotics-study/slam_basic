import {ComponentType, lazy} from "react";
import {AlgoSection, Localized} from "../../../types/global";

// 대분류 소개 페이지 레지스트리. 알고리즘 각론 전에 "이 갈래가 무엇인가"류의 개념
// 설명을 담는다. sections의 en/ko 문자열은 렌더된 본문 h2 헤딩과 정확히 일치해야
// TOC/검색 앵커(slug)가 맞는다. 집필된 페이지는 멀티라인 리터럴로 쓴다 (스크립트
// 파싱 규약). 배열 순서가 홈의 섹션 카드 순서이자 계보의 학습 순서다.
export interface ISectionIntro {
    key: AlgoSection;
    contents: ComponentType;
    sections: Localized[];
}

const data: ISectionIntro[] = [
    // 뿌리 — SLAM 의 두 절반(자세 추정, 지도 구축)을 분리해 가르치는 재귀 Bayes 필터.
    {
        key: "filtering",
        contents: lazy(() => import("./Filtering")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "The Recursive Bayes Filter", ko: "재귀 Bayes 필터"},
            {en: "Two Halves of SLAM", ko: "둘로 갈라진 SLAM"},
            {en: "What Is Written, and What Comes Next", ko: "집필된 것과 그다음"},
        ],
    },
    // 두 번째 갈래 — 오도메트리 없이 측정만으로 자세를 구하는 점군 등록.
    {
        key: "registration",
        contents: lazy(() => import("./Registration")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Why Without Odometry", ko: "왜 바퀴 없는 오도메트리인가"},
            {en: "Correspondence, then Alignment", ko: "대응, 그리고 정렬"},
            {en: "From Points to Distributions", ko: "점에서 분포로"},
        ],
    },
    // 랜드마크 계열이 소비하는 (id, bearing, range)의 id 목록이 어디서 오는가.
    {
        key: "features",
        contents: lazy(() => import("./Features")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Why Landmarks at All", ko: "왜 랜드마크인가"},
            {en: "Density Clustering", ko: "밀도 클러스터링"},
        ],
    },
    // 추정기 안에 지도를 넣는 갈래 — Rao-Blackwell화, 입자당 격자, 관성 긴밀 결합.
    {
        key: "filter_based",
        contents: lazy(() => import("./FilterBased")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Rao-Blackwellization", ko: "Rao-Blackwell화"},
            {en: "The Grid Inside the Particle", ko: "입자 안의 격자"},
            {en: "Tight Coupling", ko: "긴밀 결합"},
        ],
    },
    // 계보의 종착점 — 측정을 제약으로 모아 궤적 전체를 함께 푼다.
    {
        key: "graph_based",
        contents: lazy(() => import("./GraphBased")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "From Filter to Graph", ko: "필터에서 그래프로"},
            {en: "Loop Closure in Practice", ko: "실제의 루프 클로저"},
        ],
    },
]

export default data
