# 05. Ld2-SevenDOF (Per-Tire, 7 DOF)

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. Ld2 가 Ld1 위에 더하는 것 (per-tire Fz, lateral transfer, Ackermann,
   differential) 을 정확히 열거한다.
2. lateral weight transfer 의 roll-stiffness 분배 공식을 roll moment balance
   로부터 유도한다.
3. Ackermann steering geometry 의 inner/outer wheel 각도 식을 유도하고
   percent-interpolation 의 의미를 설명한다.
4. Open / Locked / LSD differential 의 torque 분배 메커니즘과 split-mu 거동
   차이를 설명한다.
5. linear region 에서 Ld2 ↔ Ld1 이 일치하고 nonlinear 에서 갈라지는 이유를
   설명한다.

## Prerequisites

- **Chapter 04** — single-track, longitudinal transfer, 1-step lag.
- **Chapter 02** — body EoM, 3 종류의 $a_x$.
- **Chapter 03** — Pacejka $F_x, F_y, M_z$.

---

## 5.1 동기 — Ld1 → Ld2

| 항목 | Ld1 | Ld2 |
|---|---|---|
| Wheel count | 2 (axle 평균) | 4 (per-tire) |
| Fz | axle static + long | per-tire static + long + lat transfer |
| Pacejka call | 2 | 4 |
| Wheel spin | 2 | 4 (independent) |
| Differential | 없음 | Open / Locked / LSD |
| Ackermann | 평균 δ | per-wheel $\delta_{\text{inner}}/\delta_{\text{outer}}$ |
| Steering rack torque | 없음 | front Mz sum × ratio |
| Roll/pitch 진단 | 0 | quasi-static estimate |

Ld2 는 본격 차량 dynamics validation 의 entry level — ADAS 도메인의 표준
fidelity.

![Weight transfer in a left turn](figures/05_weight_transfer.png)

---

## 5.2 가정

| 가정 | 의미 | 깨지는 case |
|---|---|---|
| Quasi-static transfer | weight transfer 가 즉시 (suspension transient 없음) | chapter 06 (Ld3 dynamic roll) |
| 1-step lag $a_y$ | lateral transfer 가 직전 step $a_y$ 기반 | chapter 11 (iteration 옵션) |
| Symmetric LSD | drive/coast ramp 대칭 | 실차 LSD 비대칭 |
| Zero camber | roll 로부터 camber 없음 ($\gamma=0$) | chapter 06 |
| Rigid roll axis | roll-stiffness 비율로만 분배 | chapter 06 (compliance) |

---

## 5.3 Per-tire Fz — lateral + longitudinal transfer

Static per-tire:

$$
F_{z,f}^{\text{static}} = \frac{m g b}{2 L}, \qquad
F_{z,r}^{\text{static}} = \frac{m g a}{2 L}
$$

Longitudinal transfer (axle 의 절반):

$$
\Delta F_{z,\text{long,half}} = \frac{m\, a_x\, h_{cg}}{2 L}
$$

Lateral transfer — 핵심 식:

$$
\Delta F_{z,\text{lat},f} = \frac{m\, a_y\, h_{cg}}{T_{w,f}} \cdot s_f, \qquad
\Delta F_{z,\text{lat},r} = \frac{m\, a_y\, h_{cg}}{T_{w,r}} \cdot s_r
$$

$$
s_f = \frac{K_{\phi,f}}{K_{\phi,f} + K_{\phi,r}}, \qquad
s_r = \frac{K_{\phi,r}}{K_{\phi,f} + K_{\phi,r}}
$$

$K_{\phi,f}, K_{\phi,r}$ 는 front/rear axle 의 roll stiffness (spring + ARB 합성).

share 의 의미: total roll moment 가 두 axle 에 stiffness 비율로 분배된다.
front 가 더 stiff → front axle 이 더 많은 lateral transfer 흡수 → front
outer 가 더 loaded → understeer 경향.

### 유도 — roll moment balance

좌선회 ($a_y > 0$). sprung mass 의 inertia force $= -m_s a_y \hat{y}$. CG
height $h_{cg}$ lever arm 으로 roll moment $M_{\text{roll}} = m a_y h_{cg}$.

이 moment 가 front/rear roll spring 에 parallel 로 분배 → 각 axle share 는
stiffness 비율. axle share 가 결정되면 좌우 tire 의 Fz 차이를 만든다:

$$
F_{\text{outer}} - F_{\text{inner}} = \frac{M_{\text{roll,axle}}}{T_w / 2}
= 2\, \Delta F_{z,\text{lat}}
$$

$$
\therefore \Delta F_{z,\text{lat}} = \frac{M_{\text{roll,axle}}}{T_w}
= \frac{m\, a_y\, h_{cg}\, s}{T_w}
$$

부호 직관: $a_y > 0$ (좌선회, centripetal $+y$) → 차체가 $-y$ 쪽으로 기울고
→ right side (FR, RR) loaded → $F_{z,FR} > F_{z,FL}$.

### $a_y$ 의 정확한 정의

lateral acceleration 은 $a_y = \dot v_y + v_x r = F_y/m$ 이지 frame 도함수
$\dot v_y$ 가 아니다 (chapter 02 §2.7). 이를 혼동하면 lateral transfer 가 두
자릿수 N 으로 비현실적으로 작아진다. 정량 사례는 §5.13 box.

---

## 5.4 Per-wheel velocity / slip

각 wheel 의 body-frame velocity:

$$
v_{x,\text{body},i} = v_x - r\, r_{y,i}, \qquad
v_{y,\text{body},i} = v_y + r\, r_{x,i}
$$

wheel 의 body-frame 위치 $(r_{x,i}, r_{y,i})$:

- FL: $(+a, +T_{w,f}/2)$
- FR: $(+a, -T_{w,f}/2)$
- RL: $(-b, +T_{w,r}/2)$
- RR: $(-b, -T_{w,r}/2)$

(ISO 8855 RH: $+y$ = leftward → left wheels $+y$, right wheels $-y$.)

front wheels 는 추가로 steer $\delta_i$ (Ackermann 시 wheel 마다 다름) 회전:

$$
v_{x,\text{wheel}} =  v_{x,\text{body}}\cos\delta_i + v_{y,\text{body}}\sin\delta_i, \qquad
v_{y,\text{wheel}} = -v_{x,\text{body}}\sin\delta_i + v_{y,\text{body}}\cos\delta_i
$$

rear: wheel frame = body frame. 이것이 chapter 03 Pacejka input
($\alpha, \kappa$) 의 source.

![Ackermann inner/outer wheel angle vs interpolation %](figures/05_ackermann.png)

---

## 5.5 Ackermann steering geometry

### 문제

low-speed cornering 에서 inner/outer wheel 의 path radius 가 다르다. parallel
steer (양 wheel 동일 $\delta$) 면 inner wheel 이 의도 path 보다 큰 radius 를
그리려 해 불필요한 slip 발생.

### Perfect Ackermann

instantaneous center of rotation (ICR) 이 rear axle 연장선과 front steering
plane 의 교점. 양 front wheel 의 angle 이 그 ICR 로 align:

$$
R = \frac{L}{\tan\delta_{\text{avg}}}, \qquad
R_{\text{inner}} = R - \tfrac{T_{w,f}}{2}, \qquad
R_{\text{outer}} = R + \tfrac{T_{w,f}}{2}
$$

$$
\delta_{\text{inner}} = \arctan\frac{L}{R_{\text{inner}}}, \qquad
\delta_{\text{outer}} = \arctan\frac{L}{R_{\text{outer}}}
$$

$\delta_{\text{inner}} > \delta_{\text{outer}}$ (inner 가 더 꺾임).

### Percent-interpolation

$$
f = \frac{\text{ackermann\_percent}}{100}, \qquad
\delta_{i,\text{eff}} = \delta + f\,(\delta_{i,\text{ack}} - \delta)
$$

- $0$ → parallel steer (Ld1 호환).
- $100$ → perfect Ackermann.

차종 default: sedan 60 %, sports 85 %, FSK formula 100 %, race 90 %.

---

## 5.6 Differential model

### Open

$$
T_L = T_R = \frac{T_{\text{axle}}}{2}
$$

split-mu 면 low-mu side 가 spin up, 차량 traction = lower-mu side 한계.

### Locked (spool)

좌우 $\omega$ 동등화 (실제는 algebraic constraint/DAE). smooth approximation:

$$
\Delta\omega = \omega_L - \omega_R, \quad
\text{bias} = 0.45\tanh(2\Delta\omega), \quad
T_L = (0.5 - \text{bias})T_{\text{axle}}, \quad
T_R = (0.5 + \text{bias})T_{\text{axle}}
$$

$\Delta\omega>0$ (L faster) → bias$>0$ → R 가 더 받음 (slower wheel gets more
torque). 0.45 cap 으로 발산 방지.

### LSD

preload + ramp:

$$
\text{mag} = \operatorname{clamp}(\text{preload} + \text{ramp}\cdot|\Delta\omega|,\;0,\;0.45), \quad
\text{bias} = \text{mag}\cdot\tanh(2\Delta\omega)
$$

sedan default preload 0.10, ramp 0.20. 실차 ramp angle (drive/coast 비대칭)
보다 단순화된 대칭 모델.

split-mu accel 에서 종단 $\Delta\omega$ ordering: Open > LSD > Locked (정성
정확). 정량은 §5.13 box.

---

## 5.7 Steering rack torque

$$
M_{\text{rack}} = (M_{z,FL} + M_{z,FR}) \cdot \text{steering\_ratio}
$$

좌선회 ($\delta>0$) 에서 $M_z>0$ (self-aligning) → rack torque$>0$ → 운전자가
놓으면 centering. DriverModel force-feedback 통합용.

---

## 5.8 Body force / moment + EoM

$$
\begin{aligned}
F_{x,\text{total}} &= \textstyle\sum_i F_{x,\text{body},i} - F_{\text{aero}} - F_{rr} \\
F_{y,\text{total}} &= \textstyle\sum_i F_{y,\text{body},i} \\
M_{z,\text{total}} &= \textstyle\sum_i (r_{x,i} F_{y,\text{body},i} - r_{y,i} F_{x,\text{body},i}) + \textstyle\sum_i M_{z,\text{wheel},i}
\end{aligned}
$$

$$
m\dot v_x = F_{x,\text{total}} + m v_y r, \quad
m\dot v_y = F_{y,\text{total}} - m v_x r, \quad
I_{zz}\dot r = M_{z,\text{total}}
$$

$$
a_{x,\text{body}} = F_{x,\text{total}}/m, \qquad
a_{y,\text{body}} = F_{y,\text{total}}/m \quad (\text{1-step lag input})
$$

per-wheel spin EoM:

$$
I_w\,\dot\omega_i = T_{\text{drive},i} + T_{\text{brake},i} - F_{x,\text{wheel},i}\, R,
\qquad I_w = 0.5\, m_{\text{unsprung}}\, R^2
$$

---

## 5.9 검증 전략

| 검증 | 케이스 |
|---|---|
| Static Fz | at-rest sum $= mg$, per-axle 정확 |
| Hard brake | $F_{z,f} > 1.05\,F_{z,f}^{\text{static}}$ |
| Left turn | $F_{z,FR} > F_{z,FL}$, $F_{z,RR} > F_{z,RL}$ |
| Ld1↔Ld2 linear | $v_x\le 15, |\delta|\le 0.04$ 에서 $|r_{L2}-r_{L1}|/r_{L1}\le 0.5\%$ |
| Differential ordering | split-mu $\Delta\omega$: Open > LSD > Locked |
| Ackermann | tight turn 에서 percent $0\to100$ 시 SS yaw rate 증가 |

linear region 에서 Ld2≈Ld1, nonlinear ($v_x=20,\delta=0.10$) 에서 outer-tire
saturation 으로 최대 ~89 % 차이 — Ld2 의 가치는 nonlinear region 에 있다.

---

## 5.10 한계

| 항목 | 한계 | 다루는 chapter |
|---|---|---|
| Suspension dynamics | quasi-static (transient 없음) | chapter 06 |
| Roll/pitch dynamic response | 1-step estimate 만 | chapter 06 |
| Wheel hop / unsprung 진동 | 없음 | chapter 06 |
| Anti-dive / anti-squat | 없음 | chapter 06 |
| Camber from roll | $\gamma=0$ | chapter 06 |

---

## 5.11 다음 chapter 와의 연결

Ld2 는 weight transfer 를 quasi-static 으로 처리한다. chapter 06 (Ld3,
14-DOF) 는 sprung/unsprung mass 를 분리하여 suspension 의 dynamic response
(roll/pitch transient, wheel hop, camber) 를 추가한다.

---

## 5.12 참고문헌

- **Genta, G.** *Motor Vehicle Dynamics*, §6 per-tire and weight transfer.
- **Milliken & Milliken**, *Race Car Vehicle Dynamics*, §6 weight transfer, §15 Ackermann/diff.
- **Reimpell, J.** *The Automotive Chassis*, §3 steering geometry.

---

## 5.13 Self-check

<details>
<summary>1. front roll stiffness 를 키우면 understeer/oversteer 중 어느 쪽?</summary>

understeer. front share $s_f$ 증가 → front lateral transfer 증가 → front
outer tire 과대 loading → tire saturation 으로 front cornering 능력 저하 →
understeer.
</details>

<details>
<summary>2. <code>ay_prev</code> 를 <code>v̇y</code> 로 두면 무슨 일이 생기나?</summary>

$a_y = \dot v_y + v_x r$ 인데 $\dot v_y$ 만 쓰면 $v_x r$ centripetal 항이
빠져 lateral transfer 가 수십 N 으로 비현실적으로 작아진다. $a_y = F_y/m$ 로
써야 한다.
</details>

<details>
<summary>3. split-mu 에서 Open diff 가 Locked 보다 traction 이 나쁜 이유?</summary>

Open 은 좌우 torque 동일 → low-mu wheel 이 먼저 spin up 하고 high-mu wheel
도 같은 (낮은) torque 만 받음. Locked 는 slower(high-mu) wheel 로 torque 를
몰아줘 traction 확보.
</details>

<details>
<summary>4. parallel steer (0 % Ackermann) 의 low-speed 문제는?</summary>

inner wheel 의 필요 steer 각이 outer 보다 큰데 둘이 같으면 inner 가 의도보다
큰 radius 를 그리려 해 slip/타이어 마모 발생. tight/low-speed turn 에서 두드러짐.
</details>

<details>
<summary>5. Ld2 가 Ld1 대비 의미를 갖는 영역은?</summary>

nonlinear region. linear 에서는 lateral transfer 효과가 작아 SS yaw rate 가
거의 동일 (≤0.5 %). 고 $a_y$ 에서 outer-tire saturation 이 본격화되며 갈라진다.
</details>

---

## 5.14 VDSim 구현 노트

> **[VDSim impl] § 5.3 — Lateral transfer 코드**
>
> `core/src/seven_dof_dynamics.cpp:158-176`:
>
> ```cpp
> const double dFz_lat_f = (Tw_f > 1e-3)
>     ? m * ay_prev_ * h_cg / Tw_f * share_f : 0.0;
> const double dFz_lat_r = (Tw_r > 1e-3)
>     ? m * ay_prev_ * h_cg / Tw_r * share_r : 0.0;
> Fz[WHEEL_FL] = Fz_static_f - dFz_long_half - dFz_lat_f;
> Fz[WHEEL_FR] = Fz_static_f - dFz_long_half + dFz_lat_f;
> Fz[WHEEL_RL] = Fz_static_r + dFz_long_half - dFz_lat_r;
> Fz[WHEEL_RR] = Fz_static_r + dFz_long_half + dFz_lat_r;
> ```
>
> `+dFz_lat_f` 가 FR 에 더해져 좌선회 시 $F_{z,FR}>F_{z,FL}$ 부호 정확.

> **[VDSim impl] § 5.3 — ay 정의 버그 history**
>
> 초기 구현이 `ay_prev = k.dvy` (frame 도함수). 격자 sweep 에서 lateral
> transfer 가 37 N 으로 비현실적 (예상 ~2000 N). `ay_prev = Fy_total/m` 로
> 수정 후 3693 N 으로 수렴. chapter 02 §2.7 "3 종류 ax" 의 실전 사례.
> `core/src/seven_dof_dynamics.cpp:353-354` 의 `ax_body/ay_body` 가 명시적으로
> `Fx/m, Fy/m`.

> **[VDSim impl] § 5.5 — Ackermann 코드**
>
> `core/src/seven_dof_dynamics.cpp:212-228`. 좌선회 ($\delta>0$) 시 inside=FL,
> outside=FR. 검증: tight turn ($R=20, v_x=2, \delta=0.35$) 에서 percent
> $0\to100$ 시 SS yaw rate +37 %.

> **[VDSim impl] § 5.6 — Differential 검증**
>
> split-mu accel 종단 $\Delta\omega$: Open 0.605, LSD 0.353, Locked 0.207 rad/s.
> 정성 ordering 일치.

> **[VDSim impl] § 5.7 — Rack torque 코드**
>
> `core/src/seven_dof_dynamics.cpp:114-117`:
>
> ```cpp
> double steering_rack_torque() const override {
>     return (mz_front_sum_ * vp_.steering_ratio);
> }
> ```

> **[VDSim impl] § 5.9 — 검증 test**
>
> `SevenDOF.*` 18 tests: `ConstructionAndLevel`, `AtRestStaticFz`,
> `HardBrakeLoadsBothFrontWheels`, `LeftTurnLoadsRightWheels`,
> `SmallSteerYawRateMatchesBicycle`, `AeroDownforceIncreasesFzAtSpeed`,
> `OpenDifferentialSplitMu`, `LockedDifferentialReducesSpread`,
> `LSDBetweenOpenAndLocked`, `AckermanInfluencesTurningRadius`,
> `AckermanZeroReproducesBaseline`, `RollAngleSignAndScale`,
> `PitchAngleSignDuringBrake`, `SteeringRackTorqueSignOpposesSteer`,
> `IndependentWheelSpinUnderSplitMu` 외. 전 18 pass.
