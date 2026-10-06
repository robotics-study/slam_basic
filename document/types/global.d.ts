import {ComponentType} from "react";

// 영/한 두 언어 문자열 쌍. 알고리즘 제목·섹션 등 언어에 따라 바뀌는 메타데이터에 쓴다.
export interface Localized<T = string> {
    en: T,
    ko: T,
}

// 대분류(section) — 계보의 갈래 그 자체. 소스 코드 트리(python/slam/<section>)와 1:1 로
// 대응한다. filtering 은 알려진 지도 위의 추정(추정 기초), registration 은 오도메트리 없는
// 등록, features 는 랜드마크 추출, filter_based 는 추정기 안에 지도를 넣는 계열,
// graph_based 는 측정을 제약으로 모으는 그래프 계열이다. 다중 로봇 조율은 자매 저장소
// (mrmp_introduction)가 다루고, 단일 로봇 계획은 navigation_basic 이 다룬다.
export type AlgoSection = "filtering" | "registration" | "features" | "filter_based" | "graph_based";

export interface ISupportedExample {
    python?: boolean,
    "c++"?: boolean,
}

export interface IAlgoData {
    // URL 경로(/algo/<slug>)이자 configs/<section>/<slug>.yaml, 소스 파일명과 동일한 식별자.
    slug: string,
    title: Localized,
    // 이 알고리즘이 속한 계보 갈래 — 홈/사이드바가 섹션별로 묶고 코드 트리도 같은 이름이다.
    section: AlgoSection,
    supportedExample?: ISupportedExample,
    // 지연 로딩(React.lazy)된 컴포넌트일 수 있다. contents 가 없으면 아직 집필되지 않은 페이지.
    contents?: ComponentType,
    // 본문 major 섹션(h2) 제목 목록 — 사이드바/TOC/검색 인덱스가 공유한다.
    // 렌더된 헤딩 텍스트(현재 언어)와 문자열이 일치해야 앵커(slug)가 맞는다.
    sections?: Localized[],
}
