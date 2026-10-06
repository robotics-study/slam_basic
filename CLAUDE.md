# slam_basic

SLAM 알고리즘 구현체 + demo 모음 — 계보의 다섯 갈래(추정 기초, 스캔 매칭, 특징 추출, 필터 기반 SLAM, 그래프 기반 SLAM)를 C++ / Python 독립 이중 구현으로.

## 프로젝트 개요

**단일 로봇 SLAM/국소화** 알고리즘만 다룬다. 계획(planning)은 이 저장소의 범위가 아니다 — 단일 로봇 내비게이션 계획은 자매 저장소 [navigation_basic](https://github.com/robotics-study/navigation_basic), 다중 로봇 조율(MAPF)은 [mrmp_introduction](https://github.com/robotics-study/mrmp_introduction) 에서 다룬다. 여기서는 **상태 추정**만 다룬다: 센서 측정과 오도메트리 스트림을 받아 자세를 추정하고(국소화), 지도를 만들고(SLAM), 그 두 문제를 푸는 논문들의 계보를 따라간다.

네 갈래의 계보:

| 섹션 (= 코드 디렉토리) | 알고리즘 (⏳ = planned) | 무엇을 가르치는 갈래 |
|---|---|---|
| `filtering` | histogram_filter ⏳, grid_mapping ⏳, particle_filter ⏳, mcl ⏳ | SLAM 의 두 절반을 분리해서 가르친다. 재귀 베이지 필터의 기초 — 이산 상태 histogram filter(Thrun et al. 2005 ch.4)와 셀별 독립 베이지 = log-odds occupancy grid mapping(Moravec/Elfes, log-odds 형식은 Thrun et al.), 그리고 연속 상태로 올라간 bootstrap 입자 필터(Gordon, Salmond & Smith 1993)를 로봇 자세에 적용한 Monte Carlo Localization(Fox et al. 1999; 적응 표본 수 KLD-sampling은 Pfaff et al. 2003) |
| `registration` | icp ⏳, ndt ⏳ | 오도메트리 없이 측정만으로 자세를 구하는 등록(登錄) — 바퀴 없는 오도메트리. 점-점 ICP(Besl & McKay TPAMI 1992: 대응 + 강체 변환 폐형해)에서 정규 분포 변환 NDT(Biber & Strasser IROS 2003) |
| `features` | dbscan ⏳ | 랜드마크는 어디서 오는가 — 날 스캔 점을 밀도 클러스터링으로 안정적인 랜드마크 후보로 깎는 단계(Ester et al. KDD 1996; 해설 논문 Kriegel, Schubert & Zimek WIREs DMKD 2017). 결정성 계약: 점 인덱스 순서로 확장 |
| `filter_based` | ekf_slam ⏳, fastslam_1 ⏳, fastslam_2 ⏳, gmapping ⏳, fast_lio ⏳, fast_lio2 ⏳ | 추정기 안에 지도를 넣는다. 확장 상태 EKF-SLAM(Smith & Cheeseman 1986: 랜드마크를 상태에 증분) → Rao-Blackwell화: 궤적은 입자로 랜드마크 KF는 정확히 푸는 FastSLAM(Montemerlo et al. AAAI 2002)과 개선 제안(JAIR 2003) → 적응 리샘플링 + 입자별 격자 지도(finding 갈래의 log-odds)+ 증분 스무딩으로 완성된 GMapping(Grisetti, Stachniss & Burgard T-RO 2007). 라이다-관성 긴밀 결합(2D 교육화): 바퀴 오도메트리 없이 점 관측을 직접 반복 칼만 필터로 갱신하는 FAST-LIO(Xu et al. T-RO 2021: 키프레임 + iSAM2)와, 키프레임까지 지우고 증분 k-d 트리에 점을 직접 갱신하는 FAST-LIO2(RA-L 2022) |
| `graph_based` | spa ⏳, graphslam ⏳, cartographer ⏳ | 계획을 밀어 넣는 게 아니라 측정을 제약으로 모은다. 같은 랜드마크 공가시 관측을 — 자세만 변수로 남기고 랜드마크를 소거한 희소 자세 조정 SPA(Konolige IROS 2001)와, 랜드마크까지 정보 형식의 변수로 묶는 완전 그래프 SLAM(Grisetti, Kümmerle, Stachniss & Burgard의 tutorial 계보). Cartographer(Hess et al. ICRA 2016): 서브맵 축적 + branch-and-bound 상관 루프 폐쇄 + 희소 자세 그래프 — gmapping 의 격자와 spa 의 그래프를 하나로 |

모든 알고리즘은 추상 클래스 기반으로 다음 세 가지가 자동으로 성립해야 한다:
1. **Performance estimate** — 공통 metric(ATE RMSE, 매핑 계열은 map IoU와 landmark 오차)을 benchmark runner가 수집.
2. **Step-by-step visualization** — 알고리즘이 방출하는 trace 이벤트(공용 JSON 포맷)를 시각화 도구가 재생. GT 궤적 위에 추정 궤적/입자 구름/신뢰 영역/지도 성장/제약 간선이 같은 타임라인으로 굴러간다.
3. **Demo** — 시나리오 + 파라미터 파일만 지정하면 실행되는 데모.

## 저장소 구조

```
.
├── CLAUDE.md
├── spec/                        # 언어 공용 계약 (구현보다 우선하는 single source of truth)
│   ├── trace_schema.json        #   step-by-step trace 이벤트 JSON Schema
│   ├── param_schema.json        #   알고리즘 파라미터 선언(name/type/range/default) 스키마
│   └── data_formats.md          #   맵(pgm/yaml) · 시나리오(궤적+센서) · 센서 모델 · 난수 계약 정의
├── maps/
│   ├── grid/                    #   GT occupancy grid (ROS 스타일 yaml + pgm)
│   └── scenarios/               #   궤적 웨이포인트 + 센서/노이즈/시드 시나리오 (yaml, 맵 참조)
├── configs/<section>/           # 알고리즘별 파라미터 yaml (언어 공용) — section ∈ {filtering, registration, features, filter_based, graph_based}
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/slam/
│   │   ├── core/                # estimator.hpp, params.hpp, trace.hpp, types.hpp, rng.hpp, sim.hpp, metrics.hpp
│   │   ├── maps/                # occupancy_grid.hpp, pgm.hpp, loader.hpp (레이캐스트 포함)
│   │   ├── filtering/  registration/  features/  filter_based/  graph_based/    # 알고리즘 헤더 (사이트 섹션과 1:1)
│   │   └── src/                 # include/와 동일 구조의 구현
│   ├── demos/                   # demo_<algo>.cpp — 실행 시 trace 파일 출력
│   └── tests/                   # GoogleTest
├── python/
│   ├── pyproject.toml
│   ├── slam/
│   │   ├── core/                # estimator.py, params.py, trace.py, types.py, rng.py, sim.py, metrics.py
│   │   ├── maps/                # cpp include/slam/maps/ 와 1:1 미러
│   │   ├── filtering/  registration/  features/  filter_based/  graph_based/    # 알고리즘 모듈 (사이트 섹션과 1:1)
│   │   └── demos/               # demo_<algo>.py — demo_common.run(name, factory) 조립만
│   └── tests/                   # pytest
└── tools/                       # Python. slam 패키지에 의존 (설치 후 사용)
    ├── viz/                     # trace 재생기: replay.py (matplotlib step-by-step / 애니메이션 저장)
    ├── bench/                   # matrix runner: (scenario × algorithm) 조합 실행 + 리포트
    └── web_export/              # 문서 사이트용 맵/시나리오 JSON + gzip trace 내보내기
```

## 아키텍처 원칙

### 의존 방향 (위반은 리뷰 Critical)
- `core` 는 stdlib(+ numpy/Eigen)만 의존한다. 알고리즘 모듈을 알지 못한다. 시뮬레이터(`sim`)도 core 에 있다 — GT 궤적 재샘플, 오도메트리 노이즈, 빔 레이캐스트, 랜드마크 관측 생성은 알고리즘이 아니라 계약의 일부다.
- `maps` 는 `core` 만 의존한다.
- 알고리즘 모듈(다섯 갈래)은 `core` 의 추상 인터페이스(`Estimator`, `Observation`, 시뮬레이터가 만드는 `Step`)에만 의존한다. **구체 맵 클래스 직접 참조 금지**, 알고리즘 모듈 간 상호 의존 금지.
- `tools/viz`, `tools/bench`, `tools/web_export` 는 trace/param/map 포맷(spec)과 `core`/`maps` 로더에만 의존한다. 알고리즘 내부 상태 접근 금지 — 시각화에 필요한 모든 정보는 trace 이벤트로 방출되어야 한다.
- `demos` 는 최상위 조립 계층: 알고리즘 + 시나리오 + configs 를 묶기만 한다. 로직 금지.

### 언어 미러링
- C++과 Python은 **같은 설계를 각자 idiomatic 하게** 구현한다. 클래스/메서드 개념 이름, 파라미터 이름, trace 이벤트는 동일해야 한다 (표기만 언어 컨벤션).
- 알고리즘 추가/변경은 원칙적으로 두 언어 동시 반영. 한쪽만 구현된 상태는 README parity 표에 명시하고 남겨두지 않는 것을 원칙으로 한다.
- 언어 간 공유물(trace schema, param yaml, 맵 데이터, 시나리오)은 반드시 `spec/`, `configs/`, `maps/` 에 두고 양쪽에서 로드한다. 언어 디렉토리 안에 복제 금지.
- **자체 이중 구현이 원칙, 실전 시스템은 래핑 허용**: 기본은 두 언어 자체 구현(비트 동일 계약이 자연히 성립). 원본 실전 시스템이 미러링하기엔 너무 크면(예: 팩터 그래프 최적화기) 유명한 라이브러리를 감싸 쓸 수 있다 — 단 두 언어가 **같은 네이티브 커널**을 부를 때만(bit-identical 계약 보존은 공식 바인딩 = 같은 C++ 코드일 때만 성립). 한쪽 언어에만 있는 라이브러리는 단일 언어 구현으로 parity 표에 명시. 개념 문서는 무엇을 감쌌든 핵심 수식과 알고리즘 골격을 직접 보여준다.
- **난수 결정성**: 입자 필터·샘플링이 개입하는 모든 알고리즘은 `core/rng`(splitmix64 → uniform01 → Box-Muller)만 쓴다. Python/C++/TS 가 비트 단위로 같은 수열을 만든다(정수는 uint64 오버플로 보정 포함 동일 연산, float 는 float64 ). 시드는 시나리오 yaml 이 들고, 드로 순서가 계약의 일부다. 노이즈·입자 드로가 언어마다 다른 수열이면 parity 검증이 무너진다.
- **초월함수 비트 동일 (libm 라우팅)**: Python 은 `math.sin/cos/atan2/log` = libSystem 스칼라 구현을 부르고 sqrt/floor 는 하드웨어 명령(정확한 반올림)이다. Apple clang 은 같은 인자의 `(sin(x), cos(x))` 호출 쌍을 SIMD 구현 `__sincos_stret` 로 접고 이 변형은 스칼라와 드문 입력에서 1 ulp 어긋난다(발견 사례: sin(0x1.f9cbc4269ab30p-2)). 그래서 C++ 은 sin/cos 를 dlsym 으로 해석한 함수 포인터(`core/libm`)로만 호출한다 — 접을 수 없게 만들어야 Python 과 같은 스칼라 구현을 부른다. atan2/log 는 쌍 접기가 없어 직접 libcall 그대로. 이 계약의 회귀 테스트가 양 언어의 libm 골든(test_rng)이다.

### 상태와 측정의 계약
- 자세는 `Pose(x, y, θ)` (world 좌표 미터/라디안), 오도메트리 명령은 `Twist(Δx, Δy, Δθ)` (로봇 프레임). 좌표계 변환(`⊕`, 역원)은 core 의 한 파일에서만 정의하고 양 언어가 같은 연산 순서를 미러한다.
- 관측 타입은 둘 중 하나 — 알고리즘은 `required_capabilities()` 로 소비하는 센서 타입을 선언한다:
  - `BEAM`: 빔 스캔 = 정해진 fov 에 균일 간격 각도, GT 자세에서 DDA 레이캐스트로 얻은 끝점들(로봇 프레임). 노이즈는 빔 방향 거리 가우시안.
  - `LANDMARKS`: 점 랜드마크 관측 `(id, bearing, range)` — association 은 논문처럼 주어진다고 정직하게 가정한다 (FastSLAM 원논문이 그렇다). id 는 시나리오의 목록 순서. 이 랜드마크가 어디서 오는지는 features 갈래(dbscan)가 가르친다.
  - `IMU`(선형 가속도 + 각속도 샘플)는 라이다-관성 계열(fast_lio PR)과 함께 확장된다: 시나리오에 imu 노이즈 파라미터와 dt 가 늘고 Step 이 imu 샘플을 싣는다. 그때까지는 BEAM/LANDMARKS 뿐.
- 시뮬레이터는 GT 궤적(웨이포인트를 `step_meters` 로 등아크 재샘플)에서 스텝마다 `Step(t, gt_pose, u_noisy, z)` 를 만든다. 알고리즘은 `u_noisy` 와 `z` 만 본다 — GT 는 시각화와 metric 의 몫이다.

### 파라미터 추상화
- 각 알고리즘은 자신의 `ParamSet` 을 선언한다: 파라미터 이름, 타입, 기본값, 유효 범위/제약. 선언 형식은 `spec/param_schema.json` 을 따른다.
- 값은 `configs/<section>/<algorithm>.yaml` 에서 로드하고 로드 시점에 선언 기반 검증(범위 밖 → 에러)을 수행한다. 코드에 매직 넘버로 파라미터를 심지 않는다.
- 같은 yaml 을 C++/Python 양쪽이 그대로 읽는다.

### Trace (step-by-step 시각화의 계약)
- trace 는 JSON Lines 파일. 한 줄이 이벤트 하나이고 `seq` 순서가 내러티브다 — wall-clock 시간은 담지 않는다. 유일한 시간 필드는 스텝 번호 `t`.
- 이벤트: `run_started`(algorithm/scenario/params 스냅샷 + seed + 센서 선언 + GT 랜드마크 좌표, demo 가 방출), `step_observed`(demo 가 매 스텝 방출하는 입력 측: t, gt_pose, u_noisy, scan/endpoints 또는 landmark obs), 알고리즘이 방출하는 추정 이벤트 — `pose_estimated`(필터 계열의 단계별 자세 추정 + 대각 공분산), `belief_updated`(histogram: 셀별 확률 양자화), `particles_updated`(입자 구름), `landmark_estimated`(추정 mean+σ), `map_updated`(log-odds 갱신 셀 스파스 목록), `constraint_added`(graph_based: i→j 자세 제약), `trajectory_found`(batch 계열의 최종 궤적) — 마지막 `run_finished`(metrics: ate_rmse, rpe_rmse, map_iou 등 키 사전순).
- float 직렬화는 byte-equality 가 아니라 **parse 후 수치 equality** 계약 (Python 5.0 vs C++ 5 는 같은 값; 정수는 양쪽 모두 정수 바이트).
- trace 방출은 demo·viz 시 on. hot loop 에서 recorder 가 null 이면 zero-cost 여야 한다.
- **데모 산출물 형식 (룰)**: 모든 알고리즘의 demo trace 는 `replay.py` 로 (1) 애니메이션 **GIF** (`--gif`, 스텝 재생 + 최종 프레임 홀드) 와 (2) 중간 과정 **PNG 스냅샷** 세트 (`--snapshots`, 진행률 균등 분할) 로 렌더링 가능해야 한다. 산출물은 두 언어 데모 각각에 대해 `out/viz/<algo>/py/`, `out/viz/<algo>/cpp/` 아래에 둔다 (`out/` 은 gitignore — 커밋하지 않는다).

### Benchmark
- `tools/bench/run_matrix.py` 는 (scenario × algorithm) 조합을 실행하고 metric 을 수집한다: ATE RMSE(궤적 절대 오차), RPE RMSE(상대 오차), 매핑 계열은 map IoU, 랜드마크 계열은 landmark 위치 RMSE. 알고리즘 열은 `configs/<section>/<algo>.yaml` + `python/slam/demos/demo_<algo>.py` 가 둘 다 존재할 때 발견된다.
- C++ demo 는 같은 CLI 인자로 같은 trace 를 출력하므로 언어 비교가 가능하다 (난수 결정성 계약 덕분에 입자 계열도 필드 단위 일치).

## 빌드 / 테스트 / 실행

```bash
# C++ (C++20, CMake ≥ 3.20, GoogleTest)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build

# Python (≥ 3.10)
cd python && pip install -e ".[dev,viz]" && cd ..
PYTHONPATH=$PWD/python .venv/bin/python -m pytest python/tests -q
PYTHONPATH=$PWD/python .venv/bin/python -m ruff check python tools
PYTHONPATH=$PWD/python .venv/bin/python -m mypy python/slam tools

# Demo (예시 — 두 언어가 동일한 인자 형태를 갖는다)
PYTHONPATH=$PWD/python python -m slam.demos.demo_histogram_filter \
    --scenario maps/scenarios/corridor01_back_and_forth.yaml \
    --params configs/filtering/histogram_filter.yaml --trace out/trace.jsonl
./cpp/build/demos/demo_histogram_filter --scenario ... --params ... --trace out/trace.cpp.jsonl

# 시각화 / 벤치마크 / 웹 내보내기
python tools/viz/replay.py out/trace.jsonl                                        # interactive 재생
python tools/viz/replay.py out/trace.jsonl --gif out/viz/x.gif --snapshots out/snaps/
python tools/bench/run_matrix.py --out out/report.md
PYTHONPATH=$PWD/python python tools/web_export/export_web_assets.py \
    --algos histogram_filter,particle_filter,mcl,...   # 사이트용 자산 (시나리오는 각 config의 scenarios: 가 라우팅)
```

## 새 알고리즘 추가 체크리스트

1. `configs/<section>/<algo>.yaml` 에 파라미터 선언 + 기본값 작성.
2. `Estimator` 를 상속해 C++/Python 양쪽 구현 (`required_capabilities()` 선언 포함). 계보 순서 유지.
3. 스텝마다 trace 이벤트 방출. 새 이벤트 타입이 필요하면 `spec/trace_schema.json` 먼저 갱신.
4. 두 언어 각각 demo 추가 (`demo_common.run(name, factory)` 패턴 — 조립만, 로직 금지).
5. 단위 테스트: 최소 (a) 알려진 시나리오에서 추정 정확도 기준값(ATE 등), (b) 노이즈 0 극한에서 GT 복원(또는 논문 성질 검증), (c) 파라미터 검증 실패 케이스. Python 과 C++ 테스트가 같은 계약을 검증한다.
6. `tools/bench/run_matrix.py` 매트릭스에서 전 시나리오 1회 실행 확인.
7. demo trace 를 `replay.py --gif` / `--snapshots` 로 렌더링해 GIF 애니메이션 + 중간 과정 PNG 가 정상 생성되는지 확인.
8. `export_web_assets.py` 로 사이트 자산 갱신 (`document/public/data/`). README parity 표 갱신.

새 **맵/시나리오** 추가 시: `spec/data_formats.md` 포맷 규칙 → `maps/grid/*.pgm+yaml`, `maps/scenarios/*.yaml` → 알고리즘 코드는 수정하지 않는다.

## 코딩 컨벤션

- **C++**: C++20. 헤더는 `include/slam/`, 구현은 `src/` 동일 경로. 네임스페이스 `slam::<module>`. 소유권은 `unique_ptr`/값 타입 우선, raw new/delete 금지. 예외는 로드/검증 단계에서만, 추정 hot path 에서는 사용하지 않는다.
- **Python**: 전 함수 type hint 필수. `numpy` 기반 좌표 연산. 추상 클래스는 `abc.ABC`. `any`/무타입 dict 전달 금지 — 파라미터는 `ParamSet`, 상태는 `types.py` 의 dataclass 를 쓴다.
- 좌표계: world 좌표 (x, y) 와 θ 는 float, grid 인덱스는 (row, col) int — 변환은 맵 클래스만 담당한다. 이 구분을 흐리는 코드 금지.
- 주석은 WHY 만. 알고리즘 수식/모델 선택 근거는 논문 인용(저자, 연도)으로 남긴다.

## 문서 사이트 (document/)

React 18 + Vite + TS + Tailwind SPA. 2D 는 Konva, 수식은 KaTeX, 이중언어는 `<T en ko>`. 사이트 섹션(`filtering` / `registration` / `filter_based` / `graph_based`)은 저장소 코드 디렉토리와 1:1 미러 — 알고리즘 페이지는 `pages/algorithms/<section>/<slug>.tsx`, 카드·사이드바 그룹핑도 같은 섹션 키를 쓴다. 빌드/검증: `cd document && ./node_modules/.bin/tsc --noEmit && node scripts/check-engine-parity.mjs && node scripts/check-tex.mjs && yarn build`, dev 서버 `yarn dev`. check-tex 는 모든 math 문자열을 KaTeX 에 실제 렌더해 파싱 에러와 strict warn 을 잡는다. JSX attribute 문자열은 JS 이스케이프를 처리하지 않으니 math="..." 에는 역슬래시를 단일로 적는다 — 이중으로 적으면 개시 매크로 + 리터럴 텍스트가 렌더되고, checker 는 그 클래스를 warn 으로 잡아낸다.

### 알고리즘 페이지 규칙 (순서 고정)

인트로 → 개념/유도(From X to Y 등) → **Properties and Complexity** → **The Algorithm** → 증명(collapsible) → (반례 등 이론 보조) → **Demo** → **Implementation** → **References**. registry `sections[]` 도 같은 순서로.

- **알고리즘 배치는 항상 계보순**: registry 배열(= 사이드바·pager·홈 카드 순서)은 섹션 순서 filtering → registration → filter_based → graph_based, 섹션 안에서는 계보 순서(예: histogram_filter → particle_filter → mcl). 새 알고리즘도 자기 계보 위치에 끼워 넣는다 (끝에 append 금지).
- **The Algorithm**: 자료구조·루프 요약 문단 → `Pseudocode` 블록(`# 1~n` 스텝 마커) → 바로 아래 "1. ~한다" 번호 목록으로 각 스텝의 무엇/왜 해설 (예: 리샘플링 시점, association 규칙 같은 함정 포함).
- **증명**: 산문 서술 금지. 가정 → BlockMath 부등식 체인 → 모순/결론의 단계형.
- **수식 항 설명 필수 (`Terms` 컴포넌트)**: 모든 display 수식(BlockMath) 바로 아래에 `components/math/Terms` 로 기호별 설명을 붙인다. **모든 기호를 그 자리에서 정의한다** — 이전 페이지에서 정의한 기호도 다시 적어, 독자가 페이지를 왔다 갔다 하지 않게 한다.
- **Parameters 섹션 금지** — 웹은 알고리즘 설명이지 코드 문서가 아니다. parameter 개념(예: 리샘플링 임계)은 이론 산문에서 다룬다.
- **Demo**: 라이브 sandbox(`components/panels/Sandbox.tsx`) — 페이지가 모듈 상수 `runLive`(libs/algorithms 의 TS 엔진 = Python 구현의 정확한 미러)와 preset 시나리오를 넘기면, 브라우저에서 직접 실행하고 재생한다. GT 궤적 위에 추정 궤적·입자 구름·추정 랜드마크·지도 성장이 같은 타임라인으로 굴러간다. TS 엔진은 `scripts/check-engine-parity.mjs` 에 등록해 python trace 와 필드 단위 parity 를 매 빌드 검증하고, 수출 trace(`<algo>/<scenario>.jsonl.gz`)는 parity 의 기준 자료로만 쓰인다.
- **Implementation**: 실제 저장소 소스를 vite `?raw` 로 embed (사본 금지), python/c++ 탭 토글 + 파일별 GitHub 링크.
- **References**: 실제 논문 링크(DOI) 필수.
- **시각 자료 적극 배치**: 페이지·소개마다 Konva figure (CanvasFigure 래핑, 테마 색은 useCanvasColors). 데이터 표는 가운데 정렬(전역 CSS 처리됨).

### 한국어 작문 규칙

- 기술 용어 과잉 번역 금지: 헤딩·UI 는 Demo / Parameters / References 처럼 영어 유지. "인터랙티브 데모" 같은 음차 금지.
- **직역 금지**: 한국어는 영어 번역이 아니라 같은 내용을 한국어로 새로 쓴다. 번역투 표현을 자연스러운 표현으로.
- 한국어 산문에서 em-dash 삽입구("A — B — C") 금지. 문장 분리나 쉼표/괄호로 재구성.
- **조사는 선행 영어 토큰에 붙인다**: "Kalman 은" ❌ → "Kalman은" ✅, "landmark 를" ❌ → "landmark을" ✅. 수식 컴포넌트(`<InlineMath/>`) 뒤 조사도 동일.
- 세미콜론 문장 연결("~한다; ~한다") 금지 — 마침표로 분리.

### PR 워크플로우

- **머지된 브랜치에 후속 커밋을 push 하지 않는다.** push 전에 해당 브랜치 PR 상태를 확인하고, 이미 머지됐으면 main 에서 새 브랜치를 파서 새 PR 로 올린다.
- 알고리즘 wave 는 `feat/<section>-<slug>` 브랜치 → PR (간단한 계획 포함) → 리뷰 코멘트 → 수정 → merge → 브랜치 삭제. base 는 main.
