# 08. Lc5-AxTarget / Lc6-VTarget (PI + Feed-Forward Cascade)

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. longitudinal control 에 PI + FF 가 적합하고 PID/pure-FF 가 부적합한 이유를
   plant 특성으로 설명한다.
2. anti-windup 의 필요성과 integrator clamp 의 효과를 기술한다.
3. cascade (Lc6 → Lc5 → Lc4) 에서 inner-loop bandwidth 가 outer 보다 빨라야
   하는 이유를 설명한다.
4. saturation 전파 (outer integrator windup) 의 거동과 해결책을 판단한다.

## Prerequisites

- **Chapter 07** — control 사다리, Lc4-Lc6 의 위치.
- **Chapter 04** — longitudinal dynamics ($\dot v_x = F_x/m$).
- **외부** — PI/PID, cascade control 기초 (Aström & Murray).

---

## 8.1 동기 — PI + FF (not PID, not pure FF)

longitudinal dynamics 는 작은 perturbation 영역에서 1차에 가깝다.

$$
\dot v_x \approx \frac{T_{\text{drive}}\, \eta_{\text{drivetrain}} - F_{\text{aero}} - F_{rr}}{m}
$$

$F_{\text{aero}}\sim v_x^2$ 는 weak nonlinearity.

- Pure P — SS error 존재 (drag 보상 불가).
- PI — integrator 가 SS error 를 0 으로.
- PID — D term 이 noise amplify, $v_x$ noise 대비 이득보다 손해.
- PI + FF — FF 가 transient, integrator 가 SS bias.

Lc5 의 일반형 (default $K_d=0$):

$$
u = K_p\, e + K_i \int e\, dt + K_d\, \dot e + K_{ff}\, a_{x,\text{target}}
$$

---

## 8.2 가정

| 가정 | 의미 | 깨지는 case |
|---|---|---|
| 1차 plant | linear PI 로 충분 | gain scheduling 필요 영역 |
| Pedal mutually exclusive | throttle·brake 동시 비영 없음 | regen / hybrid braking |
| Clamp anti-windup | integrator $\pm i_{\max}$ cap | back-calculation 필요 |
| Sedan calibration | $K_{ff}, K_p$ 가 sedan default | FSK/race 재튜닝 |

---

## 8.3 Lc5-AxTarget

$a_{x,\text{target}}$ → throttle/brake $\in[0,1]$.

$$
\begin{aligned}
e &= a_{x,\text{target}} - a_{x,\text{meas}} \\
\text{integ} &\mathrel{+}= e\, dt \quad (\text{clamp } \pm i_{\max}) \\
u &= K_p e + K_i\,\text{integ} + K_d \dot e + K_{ff}\, a_{x,\text{target}} \\
&\begin{cases}
u\ge0: & \text{throttle}=\operatorname{clamp}(u,0,1),\; \text{brake}=0 \\
u<0:   & \text{throttle}=0,\; \text{brake}=\operatorname{clamp}(-u,0,1)
\end{cases}
\end{aligned}
$$

### Throttle/brake 자동 분기

$u$ 의 부호로 dispatch — mutually exclusive. 단순 가정으로, 실제는 (1) regen
EV 에서 $u<0$ 이 throttle<0 일 수 있고, (2) 급격한 sign 전환 시 actuator
dead-time 으로 진동 가능. follow-up: overlap dead-band, regen mode.

### Anti-windup

integrator 를 $\pm i_{\max}$ 로 clamp. saturation 동안 integrator 가 무한
누적되어 desaturation 후 overshoot 를 만드는 windup 을 방지한다. 정량 사례는
§8.10 box.

### Feed-forward

$K_{ff}\, a_{x,\text{target}}$ 가 transient 를 빠르게 한다. 직관: 목표
가속도에 필요한 throttle 을 모델이 미리 출력하고 PI 가 보정. gain 추정:

$$
K_{ff} \approx \frac{m R}{T_{\max}} \approx \frac{1500\cdot0.32}{300} \approx 1.6\;\text{s}^2/\text{m}
$$

normalized throttle 로 변환하면 $K_{ff,\text{norm}}\approx0.10$ (sedan). 차종별
calibration 필요.

---

## 8.4 Lc6-VTarget — Cascade PI

$v_{\text{target}}$ → $a_{x,\text{target}}$.

$$
e = v_{\text{target}} - v_{x,\text{meas}}, \quad
\text{integ} \mathrel{+}= e\,dt\;(\text{clamp}), \quad
a_{x,\text{target}} = \operatorname{clamp}(K_p e + K_i\,\text{integ},\, \pm a_{\text{clamp}})
$$

cascade:

```
v_target → Lc6 PI → ax_target → Lc5 PI+FF → throttle/brake → Plant
```

- outer (Lc6) 가 inner (Lc5) 보다 느림.
- inner 가 빠르게 $a_x$ 를 tracking → outer 입장에서 $a_x$ 가 거의 즉시
  따라가는 1차 system 으로 보임.
- gain/saturation 분리 → 개별 tuning 용이. industrial cascade 의 표준.

bandwidth 분리 가이드:

$$
\omega_{\text{inner}} \ge 3\, \omega_{\text{outer}}
$$

default 에서 inner ~5-10 rad/s, outer ~1-2 rad/s (ratio ~5, 충분 separation).

---

## 8.5 Cascade 한계 — saturation 전파

$a_{x,\text{target}}$ 가 차량 max $a_x$ 를 초과하면:

- Lc5 가 throttle=1.0 saturate.
- 실제 $a_x$ 가 못 따라가 Lc6 의 vx error 가 누적 → integrator windup.
- anti-windup clamp 로 막히지만 outer wind-down 까지 지연.

해결 (follow-up): Lc6 의 $a_{\text{clamp}}$ 를 차종 max $a_x$ 로 자동 조정,
conditional integration (saturation 시 hold), back-calculation anti-windup.

---

## 8.6 검증 전략

| 검증 | 케이스 |
|---|---|
| Lc5 부호 분기 | target>0 throttle, <0 brake |
| Integrator | 누적 + reset clear |
| Anti-windup | sustain error 후 $|\text{integ}|\le i_{\max}$ |
| Clamp | 출력 $[0,1]$ / $\pm a_{\text{clamp}}$ |
| Lc6 부호 | error>0 → $a_x>0$ |
| End-to-end | step ax target 추종 (saturation 노출) |

---

## 8.7 한계

| 항목 | 한계 |
|---|---|
| Linear PI | gain scheduling 없음 (vx, μ별 별도 tuning) |
| Pedal exclusive | regen/hybrid braking 미반영 |
| Saturation feedback | conditional/back-calc 가 더 정확 |
| Disturbance FF | road slope, wind 미반영 |
| Actuator dynamics | first-order throttle/brake lag 미반영 |
| 차종 calibration | sedan default, FSK/race 재튜닝 |

---

## 8.8 다음 chapter 와의 연결

chapter 08 은 longitudinal cascade (Lc5/Lc6) 를 다뤘다. chapter 09 는 lateral
+ path 의 Lc7/Lc8 (Pure Pursuit, curvature → steer) 을 전개하며, 둘이 합쳐져
full path tracking (chapter 10 driver model) 을 구성한다.

---

## 8.9 참고문헌

- **Aström, K.J. & Murray, R.M.**, *Feedback Systems*, Princeton, 2008, §10 PID.
- **Aström & Hägglund**, *Advanced PID Control*, ISA, 2006, §6 anti-windup, §10 cascade.
- **Skogestad & Postlethwaite**, *Multivariable Feedback Control*, Wiley, 2005.

---

## 8.10 Self-check

<details>
<summary>1. longitudinal control 에 D term 을 빼는 이유?</summary>

$v_x$ 신호의 noise 가 D term 으로 증폭되는데, plant 가 1차에 가까워 D 의
phase-lead 이득이 작다. 손해(noise)가 이득(damping)보다 커서 $K_d=0$.
</details>

<details>
<summary>2. anti-windup 없이 saturation 에 오래 머물면?</summary>

integrator 가 계속 누적되어 desaturation 후에도 큰 적분값이 남아 과도한
overshoot/지연 desaturation 을 만든다. clamp 또는 conditional integration 필요.
</details>

<details>
<summary>3. cascade 에서 inner loop 가 outer 보다 느리면?</summary>

outer 가 명령한 $a_x$ 를 inner 가 제때 못 따라가 두 loop dynamics 가 결합되어
진동/불안정. $\omega_{\text{inner}}\ge3\omega_{\text{outer}}$ 로 decouple 해야 한다.
</details>

<details>
<summary>4. feed-forward gain 을 너무 크게 잡으면?</summary>

PI 보정 전에 FF 가 과도 입력을 줘 overshoot. $K_{ff}$ 는 plant inverse 의
근사($mR/T_{\max}$)에 맞춰야 하고 차종별로 다르다.
</details>

<details>
<summary>5. ax_target 이 차량 한계를 넘으면 end-to-end 에서 무엇이 보이나?</summary>

throttle saturate + 실측 $a_x$ 가 target 에 미달 + outer integrator windup.
이는 controller 버그가 아니라 plant limitation 의 정상 노출이다.
</details>

---

## 8.11 VDSim 구현 노트

> **[VDSim impl] § 8.3 — LongAxController 코드**
>
> `core/src/control_converter.cpp` 의 `LongAxController::update`. default
> $K_p=0.4, K_i=0.6, K_{ff}=0.10, K_d=0, i_{\max}=2.5$. $K_i\cdot i_{\max}=1.5$
> 로 integrator 단독 contribution 이 이미 throttle clamp 안에 들어와 작은
> $i_{\max}$ 로 충분.

> **[VDSim impl] § 8.4 — LongVxController 코드**
>
> `core/src/control_converter.cpp` 의 `LongVxController::update`. default
> $K_p=0.8, K_i=0.20, a_{\text{clamp}}=3.5$.

> **[VDSim impl] § 8.5 — DriverModel cascade gain**
>
> DriverModel (chapter 10) 이 내부 cascade 사용: vx PID $K_p=0.6, K_i=0.15$
> (auto cascade 보다 느림 — 인간 reaction 모사), ax PID 는 Lc5 default.

> **[VDSim impl] § 8.6 — 검증 test + end-to-end**
>
> `LongAxController.*` 7 (`ZeroErrorZeroOutput`, `PositiveTargetUsesThrottle`,
> `NegativeTargetUsesBrake`, `IntegratorAccumulates`, `ResetClearsState`,
> `OutputsAreClamped`, `IntegratorAntiwindup`) + `LongVxController.*` 4
> (`ZeroErrorZeroOutput`, `PositiveErrorPositiveAx`, `NegativeErrorNegativeAx`,
> `OutputClamped`). 11 pass.
>
> `vdsim_ax_track_demo` step target (0→+2→0→−3→0, 10 s):
>
> | Phase | ax_target | mean ax | RMSE | 해석 |
> |---|---:|---:|---:|---|
> | accel | +2 | +0.58 | 1.42 | actuator saturation |
> | coast | 0 | +0.47 | 0.48 | momentum + integ residual |
> | brake | −3 | −1.51 | 2.24 | tire saturation |
> | idle | 0 | 0 | 0.02 | drift 최소 |

> **[VDSim impl] § 8.x — 사용 예제**
>
> ```cpp
> vdsim::LongAxController axc;  axc.initialize({.kp=0.4,.ki=0.6,.kff=0.10});
> vdsim::LongVxController vxc;  vxc.initialize({});
> while (...) {
>     double ax_t = vxc.update(v_target, dyn->state().vx(), dt);
>     auto [thr, brk] = axc.update(ax_t, dyn->ax_body_est(), dt);
>     vdsim::CmdL4 cmd{.throttle=thr, .brake=brk,
>                      .steer_angle_wheel=steer_from_planner};
>     dyn->step(cmd, contacts, dt);
> }
> ```
> `vdsim_path_tracking` 의 main loop 와 거의 동일.
