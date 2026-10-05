# 언어 공용 데이터 계약 (single source of truth)

이 파일은 맵·시나리오·센서 모델·난수의 **계약**을 정의한다. 구현(Python/C++/TS)은 이 문서를
미러링할 뿐이고, 문서가 코드와 다르면 코드가 틀린 것이다. 계획(planning) 저장소와 달리 여기서는
알고리즘이 지도를 만드는 동시에 센서 스트림의 **소비자**이므로, 시뮬레이터 자체가 계약의 일부다.

## 공통 규칙

- world 좌표는 미터 단위 float `(x, y)` 와 라디안 `θ`. grid 인덱스는 `(row, col)` int이며 `row 0 = 이미지 최상단`.
- 자세 합성과 역원은 고정 공식(양 언어 동일 연산 순서):
  - `⊕`: `x' = x_a + cos(θa)·dx − sin(θa)·dy`, `y' = y_a + sin(θa)·dx + cos(θa)·dy`, `θ' = wrap(θa + dθ)`
  - 오도메트리 명령은 항상 **로봇 프레임** `(dx, dy, dθ)`: `u_t = gt_{t−1}⁻¹ ⊕ gt_t` (아래 정의).
  - `wrap(a)` = 각도를 (−π, π] 로 접는다: `a − 2π·floor((a + π) / 2π)` — 양 언어 동일식.
- 숫자는 parse 후 값으로 비교한다 (Python `5.0` 과 C++ `5` 는 같은 값; 정수는 양쪽 모두 정수 바이트).

## occupancy_grid (`maps/grid/`)

ROS map_server 스타일. yaml + 그레이스케일 이미지(P2 ASCII pgm — git diff 가 읽혀야 하므로 P5 금지).

```yaml
type: occupancy_grid
image: office01.pgm      # 0(검정)=occupied, 255(흰색)=free
resolution: 0.25         # meters / pixel
origin: [0.0, 0.0]       # 이미지 좌하단 픽셀의 world pose (x, y) — θ 는 쓰지 않는다
```

- 판정: `occ = 1 − pixel/255`; `occ > free_thresh(기본 0.65)` 이면 occupied. 애매값 없음(0 또는 255 만 쓴다).
- 좌표 변환(고정식): 셀 `(r, c)` 의 중심 world 점 = `(origin_x + (c+0.5)·res, origin_y + ((H−1−r)+0.5)·res)`,
  역변환 `c = floor((x−origin_x)/res)`, `r = (H−1) − floor((y−origin_y)/res)`.

## scenario (`maps/scenarios/`)

알고리즘이 소비하는 **입력 스트림**의 정의. 알고리즘은 `u_noisy` 와 측정 `z` 만 본다 —
GT 자세는 시각화와 metric 의 몫이다.

```yaml
map: ../grid/office01.yaml   # scenario 파일 기준 상대 경로
path:                        # 웨이포인트 (world 좌표) 2개 이상 — 궤적의 폴리라인
  - [1.5, 1.5]
  - [10.5, 1.5]
step_meters: 0.5             # 등아크 재샘플 간격 (m). 마지막 스텝은 잔여 거리(더 짧을 수 있다)
sensor:
  type: beam                 # beam | landmarks — 알고리즘이 required_capabilities() 로 선언하는 쪽과 일치해야 한다
  beams: 90                  # beam 만: 광선 수. 각도는 θ − fov/2 + i·fov/(beams−1), i=0..beams−1 (양끝 포함)
  fov_deg: 180               # beam 만: 부채꼴 개방각(도). 360 도 허용
  range_max: 6.0             # 광선 최대 거리(m) / landmark 관측 가시 범위(m)
  sigma_range: 0.05          # beam: 빔 방향 거리 가우시안 σ(m). landmarks: range σ(m)
  sigma_bearing: 0.03        # landmarks 만: bearing 가우시안 σ(rad)
landmarks:                   # landmarks 타입만: 점 랜드마크 목록 — id = 목록 순서(0부터)
  - [4.5, 1.5]
odom_noise:                  # 오도메트리 명령에 더하는 가우시안 (θ 는 rad)
  sigma_xy: 0.05
  sigma_theta: 0.035
seed: 42                     # core/rng splitmix64 시드 — 드로 순서가 계약의 일부다 (아래)
```

### 궤적 재샘플 (정확히)

1. 웨이포인트 폴리라인을 호 길이 `step_meters` 로 균일 분할해 `p_0 … p_T` 를 만든다
   (`p_0` = 첫 웨이포인트, 마지막 스텝은 잔여 거리만큼만 이동 — 짧아져도 스텝으로 친다).
2. heading: `θ_k` = 구간 k(= `p_k → p_{k+1}`)의 방향 각. 마지막 점 `θ_T` 는 직전 구간의 방향을 유지한다.
   모서리에서는 해당 스텝의 `dθ` 에 회전 전체가 한 번에 실린다 (회전 스텝 분할 없음 — 계약).
3. 오도메트리 명령: `u_t = gt_{t−1}⁻¹ ⊕ gt_t` (정확한 합성 역원, 노이즈 이전). noisy 오도메트리는
   `u_noisy = (dx + εx, dy + εy, wrap(dθ + εθ))`, `εx, εy ~ N(0, sigma_xy)`, `εθ ~ N(0, sigma_theta)`.

### 드로 순서 (결정성 계약)

스텝마다 **먼저 관측 노이즈, 그다음 오도메트리 노이즈**를 이 순서로 뽑는다:
beam — 빔 0부터 beams−1 까지 순서대로 `gaussian(0, sigma_range)`; landmarks — id 오름차순으로
`range` 노이즈 후 `bearing` 노이즈. 그다음 `(εx, εy, εθ)` 순서로 odom 노이즈. 입자 필터의 입자
샘플링 드로는 알고리즘 내부(알고리즘 코드 소관)지만 **같은 seed 파생 시드**를 쓴다 — demo 가
`seed` 를 파라미터에 그대로 넘기고, 알고리즘은 `Rng(seed)` 를 자기 첫 드로부터 사용한다.

### beam 센서 모델 (DDA 레이캐스트)

광선 원점 = GT 자세 `(x,y)`, 각도 `φ_i`. 격자 DDA(Amanatides & Woo)로 첫 occupied 셀을 찾는다:
- 광선이 occupied 셀의 닫힌 정사각형 경계에 처음 들어서는 지점의까지 거리가 `r_hit < range_max` 이면
  **hit**: 관측 거리 = 그 진입 거리 (+ 노이즈). miss(범위 초과)는 점을 내지 않는다 — 스캔은 hit 점 목록.
- 관측 점은 **로봇 프레임**으로 저장: 세계 좌표 끝점 `e` 에 대해 `z = R(−θ)(e − p)` (노이즈는 빔 방향
  거리에만 더한다). 알고리즘이 자기 추정 자세로 world 복원하는 것이 과제다 — GT 를 새지 않는다.

### landmarks 센서 모델

가시성 = GT pose 에서 랜드마크 점까지의 선분이 occupied 셀을 통과하지 않음(교차 판정 고정식: 세그먼트
vs 셀 정사각형). 가시이면 `range = dist + εr`, `bearing = wrap(atan2(dy, dx) − θ_gt) + εβ`.

## 난수 (splitmix64 — 언어 간 비트 단위 동일)

```
next():  x += 0x9E3779B97F4A7C15            # uint64 오버플로 = mod 2^64 (보정 포함 동일 연산)
         z = x
         z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
         z = (z ^ (z >> 27)) * 0x94D049BB133111EB
         return z ^ (z >> 31)

uniform01(): float(next() >> 11) × 2^-53     # [0, 1) — 정확히 53 비트
gaussian(mu, sigma): u1 = uniform01(), u2 = uniform01()   # 드로 두 번이 계약
                     return mu + sigma · sqrt(−2·ln(1 − u1)) · cos(2π·u2)
```

`cos` 만 쓰고 `sin` 분기는 버린다 — 드로 수가 고정이고 재사용 캐시가 없어 언어 간 어긋남이 없다.
정수는 uint64 오버플로 보정을 포함한 동일 연산, float 는 float64(double).
