# 15. Validation & DOE (표준 maneuver / 설계 탐색)

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. ISO 7401 / 4138 / 3888-2 의 표준 maneuver 정의와 각각이 측정하는 handling
   metric 을 설명한다.
2. understeer gradient $K$, yaw overshoot, response time 의 정의를 식으로 안다.
3. parameter sweep (DOE) 이 설계 검증/sensitivity 분석에서 하는 역할을
   설명한다.
4. metric 추출기가 ISO validation 과 DOE 에 공유되는 구조의 의의를 안다.

## Prerequisites

- **Chapter 04-06** — Ld1-Ld3 (시험 대상 plant).
- **Chapter 09** — Pure Pursuit (DLC follower).
- **외부** — ISO 7401/4138/3888-2 개요, least-squares fit.

---

## 15.1 동기 — 표준 maneuver

모델 신뢰성은 재현 가능한 표준 시험으로 입증된다. ISO 시리즈는 OEM/인증기관의
공통 언어 — 같은 maneuver 로 같은 metric 을 비교하면 모델·실차·경쟁 도구가
대조 가능하다.

| 시험 | ISO | 측정 영역 |
|---|---|---|
| Step steer | 7401:2003 | transient yaw response |
| Steady-state circular | 4138:2012 | understeer gradient |
| Double lane change | 3888-2:2011 | severe transient / 한계 회피 |

---

## 15.2 가정 / 범위

| 가정 | 의미 | 깨지는 case |
|---|---|---|
| Gate-width 판정 | DLC cone 충돌 단순화 | 정확한 cone 판정 |
| Pure Pursuit follower | preview/delay 없는 단순 추종 | 실제 driver model |
| Full-factorial DOE | 격자 전수 | LHS / response-surface DoE |
| ISO-based metric | 정의는 ISO, 공식 인증 아님 | 인증 절차 |

---

## 15.3 ISO 7401 — step-steer transient

정상 주행 중 steering 을 계단 입력으로 급변 → yaw rate 과도응답. 입력: 직진
settle → $\delta$ step → 관찰.

| 기호 | 정의 |
|---|---|
| $\dot\psi_{ss}$ | 정상상태 yaw rate (마지막 20 % 평균) |
| $\dot\psi_{\text{peak}}$ | 최대 yaw rate |
| $U$ | overshoot ratio $= \dot\psi_{\text{peak}}/\dot\psi_{ss}$ |
| $T_{\max}$ | step → peak 시간 |
| $T_{\dot\psi}$ | step → 90 % $\dot\psi_{ss}$ (response time) |
| 5 % settling | ±5 % 밴드 진입·유지 시각 |

$U$ 가 1 에 가까울수록 잘 damped; 1.3 이상이면 oscillatory/불안정 경향. 샘플은
§15.10 box.

---

## 15.4 ISO 4138 — understeer gradient K

일정 속도로 steer 를 천천히 ramp 하며 $a_y$ 를 0 → 한계까지 sweep.

$$
K = \frac{d(\delta - \delta_{\text{kin}})}{d(a_y)}, \qquad
\delta_{\text{kin}} = \frac{L}{R} = \frac{L\dot\psi}{v}
$$

- $\delta$ — 실제 road-wheel steer.
- $\delta_{\text{kin}}$ — 그 반경의 운동학적 최소 steer.
- $K>0$ understeer (한계에서 steer 더 필요, "밀림"), $K<0$ oversteer,
  $K\approx0$ neutral.

선형 영역 ($1\le a_y\le4$ m/s²) 에서 $(\delta-\delta_{\text{kin}})$ vs $a_y$ 의
기울기를 least-squares fit. 단위는 mrad/g (g 당 milliradian) 와 mrad·s²/m 둘 다
보고한다.

---

## 15.5 ISO 3888-2 — double lane change (moose test)

cone 으로 정의된 차선 변경 코스를 일정 진입속도로 통과 (passenger car):

```
A 진입 12.0 m (y=0)
B 전환 13.5 m (y: 0 → +3.5)
C 오프셋 11.0 m (y=+3.5)
D 복귀 12.5 m (y: +3.5 → 0)
E 탈출 12.0 m (y=0)
```

전환 구간은 raised-cosine S-curve waypoint:

$$
y_B(x) = \text{OFF}\cdot 0.5\,(1 - \cos(\pi x / L_B))
$$

(linear ramp 는 Pure Pursuit follower 가 overshoot — 부드러운 곡선이 실제
운전자 궤적에 가깝고 follower 도 안정.) Pure Pursuit (chapter 09) 로
center-line 추종 + 비례 vx 제어.

| metric | 정의 |
|---|---|
| speed loss | 진입 − 탈출 속도 [km/h] |
| max lateral excursion | target lane 대비 최대 횡 이탈 [m] |
| peak yaw rate, peak $a_y$ | 한계 거동 지표 |

Pass (단순화): speed loss < 2 km/h AND excursion < 1 m. 속도가 오를수록 한계
에서 follow 실패 — 실차 moose test 패턴 (샘플 §15.10 box).

---

## 15.6 DOE — parameter sweep

단일 maneuver 가 아니라 파라미터 격자 × 시나리오를 일괄 실행 → response
surface. 설계 검증의 핵심 워크플로다. `vehicle.* / tire.* / solver.*`
dotted-path 로 임의 parameter override, 출력 자동 분기:

- 1-param → line plot.
- 2-param → heatmap (response surface).
- 3+ param → sensitivity bar ($\text{std}/\text{mean}$ 상대 민감도).

metric 추출기는 ISO validation 과 공유되어 DOE 가 ISO metric (예: 100 tire
variant 의 $K$ 분포) 을 격자 위에서 sweep 할 수 있다.

---

## 15.7 산업적 의의

| 사용자 | 활용 |
|---|---|
| OEM 설계 | tire vendor 후보 × CG variant → spec 만족 조합 탐색 |
| AV lab | vehicle param 변화가 path-tracking 성능에 미치는 영향 |
| 인증 준비 | ISO maneuver metric 의 사전 검증 |

Adams Car 의 DOE 모듈이 하는 일을 가벼운 YAML + Python 으로 — full Adams seat
없이 trend 확인.

---

## 15.8 한계

| 항목 | 현재 |
|---|---|
| ISO 3888-2 cone 판정 | gate width 단순화 |
| Pass 기준 | ISO 완전 조건의 부분집합 |
| Driver model | Pure Pursuit (preview/delay 모델 아님) |
| 통계적 DOE | full-factorial 만 (LHS/RSM 미구현) |
| 측정 표준 | 정의는 ISO 기반, 공식 인증 아님 |

---

## 15.9 다음 chapter 와의 연결

validation/DOE 가 모델의 정확성을 보증한다. chapter 16 은 이 검증된 모델을
FMI 표준으로 패키징하여 산업 도구와 양방향 교환하는 방법을 다룬다.

---

## 15.10 참고문헌

- **ISO 7401:2003** — Lateral transient response test methods.
- **ISO 4138:2012** — Steady-state circular driving behaviour.
- **ISO 3888-2:2011** — Severe lane-change manoeuvre.
- **Gillespie, T.**, *Fundamentals of Vehicle Dynamics*, SAE, 1992 — understeer gradient.

---

## 15.11 Self-check

<details>
<summary>1. overshoot ratio U 가 1.3 이상이면 무엇을 뜻하나?</summary>

yaw rate peak 가 정상값의 1.3 배 이상 — under-damped/oscillatory. transient 가
출렁여 안정성/승차감이 나쁜 setup.
</details>

<details>
<summary>2. understeer gradient 에서 δ_kin 을 빼는 이유?</summary>

$\delta_{\text{kin}}=L/R$ 은 그 반경을 위한 순수 기하 steer. 실제 $\delta$ 에서
이를 빼면 tire slip 때문에 추가로 필요한 steer 만 남아, 그 $a_y$ 기울기가
under/oversteer 성향을 정량화한다.
</details>

<details>
<summary>3. DLC 전환 구간에 raised-cosine 을 쓰는 이유?</summary>

linear ramp 는 곡률이 모서리에서 불연속이라 Pure Pursuit 가 overshoot. cosine
S-curve 는 곡률이 연속이라 실제 운전 궤적에 가깝고 follower 도 안정.
</details>

<details>
<summary>4. metric 추출기를 ISO validation 과 DOE 가 공유하는 이점?</summary>

같은 $K$, overshoot 정의를 격자 위에서 sweep 가능 — 단일 시험과 대량 설계
탐색이 동일 metric 으로 일관 비교된다.
</details>

<details>
<summary>5. DLC 가 속도가 오를수록 fail 하는 물리적 이유?</summary>

고속에서 같은 lateral 변위에 필요한 $a_y$ 가 커져 tire saturation 에 도달 →
follower 가 곡률을 못 내고 excursion/speed loss 증가. 실차 moose test 와 동일.
</details>

---

## 15.12 VDSim 구현 노트

> **[VDSim impl] § 15.3-15.5 — validation 스크립트 + 샘플**
>
> `apps/validation/iso_{7401,4138,3888}.py`.
>
> ISO 7401 (sports L3, 80 km/h, 6° steer): U=1.21 (20.6 % overshoot),
> $T_{\dot\psi}$=0.20 s, settling 2.29 s, $a_{y,ss}$=0.82 g.
>
> ISO 4138 (sports L3, 80 km/h): K=+9.69 mrad/g (understeer). sports typical
> 5-15 mrad/g 범위.
>
> ISO 3888-2 (sports L3): 60 km/h FAIL (excursion 1.3 m, speed loss 5.5 km/h),
> 40 km/h PASS (excursion 0.3 m, speed loss 1.0 km/h).

> **[VDSim impl] § 15.6 — DOE runner**
>
> `apps/doe/sweep_runner.py`. config 예:
>
> ```yaml
> parameters:
>   vehicle.cg_height: [0.35, 0.42, 0.50, 0.58]
>   tire.D_lat:        [0.8, 0.95, 1.05, 1.20]
> scenarios:
>   step_30deg_at_25: {type: step_steer, params: {...}}
> metrics: [peak_yaw_rate, yaw_overshoot, understeer_gradient_K]
> ```
> 16 combos × 2 scenarios = 32 runs 가 수 초. metric 추출은 `metrics.py`
> (validation 과 공유).
