<div align="center">

# 🧭 slam study

### 🌐 [robotics-study.github.io/slam_basic](https://robotics-study.github.io/slam_basic/)

문서 사이트 — 알고리즘마다 유도·성질·증명·라이브 sandbox·실제 소스를 담는다. (한국어/English 토글 내장)

**단일 로봇 SLAM · 상태 추정 알고리즘 — C++ / Python 독립 이중 구현 스터디**

같은 추상화 설계를 두 언어로 미러링하고, 언어 공용 trace 포맷으로 추정을 재생하며,<br>
(scenario × algorithm) 매트릭스로 벤치마크한다. 경로·경로 계획은 이 저장소가 아니라<br>
자매 저장소 [navigation_basic](https://github.com/robotics-study/navigation_basic) 에서, 여러 로봇의 조율(MAPF)은
[mrmp_introduction](https://github.com/robotics-study/mrmp_introduction) 에서 다룬다.

*Single-robot SLAM and state estimation, mirrored in C++20 and Python — with step-by-step
visualization, deterministic bilingual traces, and a benchmark matrix (ATE · RPE · map IoU).*

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)
![CMake](https://img.shields.io/badge/CMake-%E2%89%A53.20-064F8C.svg)

</div>

---

## ✨ 특징

- **📐 공통 추상화** — 모든 알고리즘은 `Estimator` 를 상속하고, 구체 맵이 아닌 센서 capability(`BEAM` 빔 스캔 / `LANDMARKS` 점 랜드마크 관측)만 요구한다. 시뮬레이터(GT 궤적 재샘플 · 오도메트리 노이즈 · DDA 레이캐스트)는 core 의 계약이고 알고리즘은 `u_noisy` 와 측정만 본다.
- **🎲 결정성 난수 계약** — 입자·노이즈가 개입해도 언어 간 parity 가 무너지지 않도록 splitmix64 → uniform01 → Box-Muller 수열이 spec 의 일부다. Python/C++/TS 가 비트 단위로 같은 수열을 만든다.
- **🪞 언어 미러링** — C++ 과 Python 이 같은 설계·같은 파라미터·같은 trace 이벤트를 각자 idiomatic 하게 구현한다. 공유 계약(`spec/`)·파라미터(`configs/`)·맵과 시나리오(`maps/`)는 언어 밖에 두고 양쪽에서 로드한다.
- **🎬 Trace 기반 시각화** — 알고리즘은 추정을 JSON Lines 이벤트로 방출하고(GT 궤적 위에 추정 궤적 · 입자 구름 · log-odds 지도 성장 · 제약 간선), 재생기는 언어당 하나가 아니라 **하나**(`tools/viz/replay.py`)다.
- **📊 벤치마크 매트릭스** — `tools/bench/run_matrix.py` 가 (scenario × algorithm) 조합을 실행해 ATE RMSE · RPE RMSE · map IoU 를 수집하고 리포트를 쓴다.
- **🌐 인터랙티브 문서 사이트** — 브라우저 엔진은 구현체의 세 번째 미러이며, 저장소가 수출한 trace 는 parity 검증의 기준 자료로만 쓰인다.

## 🗺️ 계보 (planned)

| 섹션 | 알고리즘 | 원 논문 |
|---|---|---|
| filtering | Histogram filter | Thrun, Burgard & Fox 2005 (교과서 ch.4) |
| filtering | Particle filter | Gordon, Salmond & Smith (1993) |
| filtering | Monte Carlo Localization | Fox, Burgard, Dellaert & Thrun (1999) · KLD-sampling Pfaff et al. (2003) |
| registration | ICP | Besl & McKay (TPAMI 1992) |
| registration | NDT | Biber & Strasser (IROS 2003) |
| filter_based | EKF-SLAM | Smith & Cheeseman (ICRA 1986) |
| filter_based | FastSLAM 1.0 | Montemerlo, Thrun, Schlegle & Kuhn (AAAI 2002) |
| filter_based | FastSLAM 2.0 | Montemerlo, Thrun, Koller & Wegbreit (JAIR 2003) |
| filter_based | GMapping | Grisetti, Stachniss & Burgard (T-RO 2007) |
| graph_based | SPA (pose graph) | Konolige (IROS 2001) |
| graph_based | GraphSLAM (information form) | Grisetti, Kümmerle, Stachniss & Burgard (2010/2013) |

> 구현이 진행되면 이 표가 실제 parity 표(✅ ✅)로 바뀌고, 각 행의 알고리즘 페이지가 문서 사이트에 열린다.

## 🚀 빠른 시작

```bash
# Python (>= 3.10) — slam 패키지 + viz/dev extras
cd python && pip install -e ".[dev,viz]" && cd ..
pytest python/tests

# C++ (C++20, CMake >= 3.20, GoogleTest 는 FetchContent 자동)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build
```

### 데모 실행 — 두 언어가 동일한 CLI 인자

```bash
# Python
python python/demos/demo_<algo>.py \
  --scenario maps/scenarios/<scenario>.yaml --params configs/<section>/<algo>.yaml --trace out/trace.jsonl

# C++ (동일 인자)
./cpp/build/demos/demo_<algo> \
  --scenario maps/scenarios/<scenario>.yaml --params configs/<section>/<algo>.yaml --trace out/trace.cpp.jsonl
```

stdout 에 한 줄 JSON metric(ATE RMSE 등), `--trace` 경로에 step-by-step JSONL trace 가 남는다.

### 시각화 — C++/Python trace 를 같은 도구로 재생

```bash
python tools/viz/replay.py out/trace.jsonl                    # interactive 재생
python tools/viz/replay.py out/trace.jsonl --gif out/x.gif --snapshots out/snaps/
```

### 벤치마크

```bash
python tools/bench/run_matrix.py --out out/report.md
```
