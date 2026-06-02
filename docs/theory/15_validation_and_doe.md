# 15. Validation & DOE — 표준 maneuver 와 설계 탐색

> **학습 목표.** ISO 7401 / 4138 / 3888-2 의 표준 maneuver 정의와 각각이
> 측정하는 handling metric 을 안다. understeer gradient K, yaw overshoot,
> response time 의 정의를 식으로 안다. parameter sweep (DOE) 이 설계 검증에서
> 하는 역할과 sensitivity 분석을 이해한다.

## 15.1 왜 표준 maneuver 인가

차량 dynamics 모델의 신뢰성은 *재현 가능한 표준 시험*으로 입증된다.
ISO 시리즈는 OEM / 인증기관이 공유하는 공통 언어 — 같은 maneuver 를 돌려
같은 metric 을 비교하면 모델·실차·경쟁 도구가 대조 가능하다.

VDSim 의 `apps/validation/` 가 세 가지를 자동화:

| 시험 | ISO | 측정 영역 |
|---|---|---|
| Step steer | 7401:2003 | transient yaw response |
| Steady-state circular | 4138:2012 | understeer gradient (정상상태 handling) |
| Double lane change | 3888-2:2011 | severe transient / 한계 회피 |

## 15.2 ISO 7401 — step-steer transient

`apps/validation/iso_7401.py`.

정상 주행 중 steering 을 계단 입력으로 급변 → yaw rate 의 과도응답 측정.
입력: `t_pre` 동안 직진 settle → `δ` step → `t_post` 관찰.

측정 metric (ISO § 7.2):

| 기호 | 정의 |
|---|---|
| `ψ̇_ss` | 정상상태 yaw rate (마지막 20% 평균) |
| `ψ̇_peak` | 최대 yaw rate |
| `U` | overshoot ratio = `ψ̇_peak / ψ̇_ss` |
| `T_max` | step → peak 까지 시간 |
| `T_ψ̇` | step → 90% ψ̇_ss 도달 시간 (response time) |
| 5% settling | ±5% 밴드 진입 후 유지 시작 시각 |

샘플 (sports L3, 80 km/h, 6° steer):
```
U = 1.21 (20.6% overshoot)
T_ψ̇ = 0.20 s  (sports car 의 빠른 응답)
settling = 2.29 s
a_y_ss = 0.82 g
```

`U` 가 1 에 가까울수록 잘 damped. 1.3 이상이면 oscillatory / 불안정 경향.

## 15.3 ISO 4138 — understeer gradient K

`apps/validation/iso_4138.py`.

정상상태 선회의 handling 특성. 일정 속도로 steer 를 천천히 ramp 하며
`a_y` 를 0 → 한계까지 sweep. 핵심 metric 은 **understeer gradient**:

$$
K = \frac{d(\delta - \delta_{\text{kin}})}{d(a_y)}, \qquad
\delta_{\text{kin}} = \frac{L}{R} = \frac{L\, \dot\psi}{v} \quad (\text{kinematic ackermann steer})
$$

- `δ` — 실제 road-wheel steer.
- `δ_kin` — 그 선회 반경을 위한 운동학적 최소 steer.
- `K > 0`: understeer (한계에서 steer 더 필요, "밀림").
- `K < 0`: oversteer.
- `K ≈ 0`: neutral.

선형 영역 (1 ≤ a_y ≤ 4 m/s²) 에서 `(δ − δ_kin)` vs `a_y` 의 기울기를
least-squares fit.

샘플 (sports L3, 80 km/h): `K = +9.69 mrad/g` (understeer). sports car
typical 5–15 mrad/g 범위 안 — 모델이 합리적.

단위 주의: `mrad/g` (g 당 milliradian) 와 `mrad·s²/m` 둘 다 보고.

## 15.4 ISO 3888-2 — double lane change (moose test)

`apps/validation/iso_3888.py`.

한계 영역의 severe 회피. cone 으로 정의된 차선 변경 코스를 일정 진입속도로
통과. 코스 (passenger car):

```
A 진입 12.0 m (y=0)
B 전환 13.5 m (y: 0 → +3.5)
C 오프셋 11.0 m (y=+3.5)
D 복귀 12.5 m (y: +3.5 → 0)
E 탈출 12.0 m (y=0)
```

전환 구간은 raised-cosine S-curve waypoint 로 부드럽게:

$$
y_B(x) = \text{OFF} \cdot 0.5 \cdot \big(1 - \cos(\pi x / L_B)\big)
$$
(초기 linear ramp 는 Pure Pursuit follower 가 overshoot — 부드러운 곡선이
실제 운전자 궤적에 더 가깝고 follower 도 안정).

VDSim 은 Pure Pursuit (Chapter 09) 로 center-line 추종 + 비례 vx 제어.

측정:
| metric | 정의 |
|---|---|
| speed loss | 진입 − 탈출 속도 [km/h] |
| max lateral excursion | target lane 대비 최대 횡 이탈 [m] |
| peak yaw rate, peak a_y | 한계 거동 지표 |

Pass 기준 (단순화): speed loss < 2 km/h **AND** excursion < 1 m.

샘플 (sports L3):
- 60 km/h: FAIL (excursion 1.3 m, speed loss 5.5 km/h).
- 40 km/h: PASS (excursion 0.3 m, speed loss 1.0 km/h).

실차 moose test 와 동일 패턴 — 속도가 오를수록 한계에서 follow 실패.

## 15.5 DOE — parameter sweep

`apps/doe/sweep_runner.py`.

단일 maneuver 가 아니라 **파라미터 격자 × 시나리오** 를 일괄 실행 →
response surface. 설계 검증의 핵심 워크플로:

```yaml
parameters:
  vehicle.cg_height: [0.35, 0.42, 0.50, 0.58]
  tire.D_lat:        [0.8, 0.95, 1.05, 1.20]
scenarios:
  step_30deg_at_25: {type: step_steer, params: {...}}
metrics:
  - peak_yaw_rate
  - yaw_overshoot
  - understeer_gradient_K
```

`vehicle.* / tire.* / solver.*` dotted-path 로 어떤 parameter 든 override.
16 combos × 2 scenarios = 32 runs 가 수 초.

출력 자동 분기:
- 1-param sweep → line plot.
- 2-param → heatmap (response surface).
- 3+ param → sensitivity bar (각 param 의 `std/mean` 상대 민감도).

`metrics.py` 의 추출기는 ISO validation 의 metric 과 공유 — DOE 가 ISO
metric 을 격자 위에서 sweep 가능 (예: 100 tire variant 의 K 분포).

## 15.6 산업적 의의

| 사용자 | DOE/validation 활용 |
|---|---|
| OEM 설계 | tire vendor 후보 × CG variant → spec 만족 조합 탐색 |
| AV lab | vehicle param 변화가 path-tracking 성능에 미치는 영향 |
| 인증 준비 | ISO maneuver metric 의 사전 검증 |

Adams Car 의 DOE 모듈이 하는 일을 가벼운 YAML + Python 으로. full Adams seat
없이 trend 확인.

## 15.7 한계

| 항목 | 현재 |
|---|---|
| ISO 3888-2 cone 판정 | gate width 단순화 (정확한 cone 충돌 판정 아님) |
| Pass 기준 | ISO 의 완전한 통과 조건의 부분집합 |
| Driver model | Pure Pursuit (실제 driver 의 preview/delay 모델 아님) |
| 통계적 DOE | full-factorial 만 (LHS / response-surface DoE 미구현) |
| 측정 표준 준수 | metric 정의는 ISO 기반이나 공식 인증 절차 아님 |

## 15.8 참고

- ISO 7401:2003 — Lateral transient response test methods.
- ISO 4138:2012 — Steady-state circular driving behaviour.
- ISO 3888-2:2011 — Severe lane-change manoeuvre.
- Gillespie, *Fundamentals of Vehicle Dynamics*, SAE 1992 — understeer gradient.
- 구현: `apps/validation/iso_{7401,4138,3888}.py`, `apps/doe/sweep_runner.py`.
