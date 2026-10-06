// spec/trace_schema.json의 TypeScript 대응 + 언어 공용 상태 타입.
// C++/Python 데모가 방출한 trace와 브라우저 라이브 엔진이 만드는 이벤트가 같은 타입을
// 공유한다 — 패널/플레이어 코드는 하나다. SLAM trace에는 wall-clock 시간이 없다:
// 재생은 seq 순서만으로 진행하고, 유일한 시간 필드는 스텝 번호 t 다.

// 파라미터 값 — configs yaml 의 선언 타입과 동일 (int|float 은 JS 에서 number 로 합쳐진다).
export type ParamValue = number | boolean | string;

export type Point = [number, number];
export type Cell = [number, number];
// 자세 [x, y, θ] — x,y 미터, θ 라디안 ([-pi, pi) 로 접힌 값).
export type Pose = [number, number, number];
// 오도메트리 명령 [dx, dy, dθ] — 로봇 프레임 변위(m)와 회전(rad).
export type Twist = [number, number, number];

export interface LandmarkObs {
    id: number;
    bearing: number;
    range: number;
}

// 시나리오 센서 선언 스냅샷 (run_started가 싣는 그대로).
export interface SensorConfig {
    type: "beam" | "landmarks";
    beams?: number;
    fov_deg?: number;
    range_max: number;
    sigma_range: number;
    sigma_bearing?: number;
}

export type TraceEventType =
    | "run_started"
    | "step_observed"
    | "pose_estimated"
    | "belief_updated"
    | "particles_updated"
    | "landmarks_updated"
    | "map_updated"
    | "constraint_added"
    | "trajectory_found"
    | "run_finished";

export interface TraceEvent {
    seq: number;
    event: TraceEventType;
    // run_started 전용.
    algorithm?: string;
    scenario?: string;
    params?: Record<string, unknown>;
    seed?: number;
    sensor?: SensorConfig;
    // run_started 선택 (landmarks 시나리오): GT 랜드마크 좌표 — 목록 순서가 id.
    landmarks?: Point[];
    // 스텝 번호. step_observed 와 추정 이벤트가 같은 t 로 짝을 이룬다.
    t?: number;
    // step_observed 전용: 그 스텝의 ground-truth 자세 (시각화와 metric 만 쓴다).
    gt?: Pose;
    // step_observed 선택: 알고리즘이 소비하는 noisy 오도메트리 명령 (t=0 에는 없다).
    odom?: Twist;
    // step_observed 선택 (beam): hit 된 빔 끝점 목록 — 로봇 프레임. 재생기가 GT 자세로
    // world 에 복원해 레이저 부채꼴을 그린다.
    scan?: Point[];
    // step_observed 선택 (landmarks): id 오름차순 관측.
    obs?: LandmarkObs[];
    // pose_estimated 전용: 추정 자세 + 선택 대각 표준편차 [σx, σy, σθ].
    pose?: Pose;
    cov?: [number, number, number];
    // belief_updated: [row, col, p] 전체 목록. map_updated: [row, col, l] 스파스 diff.
    cells?: Array<[number, number, number]>;
    // particles_updated 전용: [x, y, θ, w] — 리샘플링 후 입자 구름.
    particles?: Array<[number, number, number, number]>;
    // landmarks_updated 전용: 현재 랜드마크 추정.
    estimated?: Array<{ id: number; x: number; y: number; sx?: number; sy?: number }>;
    // constraint_added 전용: 제약 간선 (노드 i 프레임에서 관측된 상대 자세 d).
    i?: number;
    j?: number;
    d?: Twist;
    // loop = 루프 클로저(비인접 재관측) — 오도메트리 간선에는 생략.
    loop?: boolean;
    // trajectory_found 전용 (batch 계열): 인덱스가 스텝 t 와 일치하는 최종 자세 목록.
    poses?: Pose[];
    // run_finished 전용: 지표 (키 사전순).
    metrics?: Record<string, number>;
}
