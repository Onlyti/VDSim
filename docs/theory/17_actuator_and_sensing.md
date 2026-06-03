# 17. Actuator Dynamics & Sensing

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. 제어기와 plant 사이의 actuator boundary 가 commanded → realized 명령을 어떤
   비선형으로 변형하는지 (transport delay, lag, rate, saturation, dead-zone) 를
   식 단위로 기술한다.
2. 스티어링 액추에이터의 **두 모드** — first-order lag vs **torque servo (PD +
   inertia + LuGre)** — 가 무엇이 입력/출력/오차이고 왜 PD(적분 없음)인지 설명한다.
3. LuGre friction 의 bristle/Stribeck 항과 semi-implicit 적분이 stick-slip·
   presliding 을 어떻게 표현하는지 안다.
4. brake dead-zone(pad clearance) + mu(T) thermal fade, throttle dead-zone
   (pedal tip-in), sensor feedback delay 의 물리적 의미와 식을 기술한다.

## Prerequisites

- **Chapter 11** — first-order lag, transport delay, substep 적분.
- **Chapter 07/08** — control ladder 가 actuator 상류에서 CmdL4 를 만드는 구조.
- **Reference** — `docs/references/actuator_nonlinearity.md` (taxonomy + 실측 ID
  문헌: LuGre #2, steering friction #1, backlash #4/#15, EMB clamp #11).

---

## 17.1 동기 — plant-side actuator boundary

제어기는 이상적 명령(throttle/brake/steer)을 내지만, 실제 차량은 그 명령을 즉시·
정확히 따르지 못한다. 모터·유압·EPS 가 유한 대역폭·마찰·지연·포화를 가진다. VDSim
은 이 비선형을 plant 직전의 **actuator layer** 로 모은다 (plant-non-invasive):

```
controller --(CmdL4)--> [ActuatorModel.apply] --(realized CmdL4)--> dyn.step()
                                                                        |
controller <--(measured State)-- [SensorDelay.apply] <--(true State)---+
```

핵심 원리: 물리 비선형성은 actuator 의 물리 인터페이스(torque/force/steer-angle)에
존재하므로, 어떤 control ladder 레벨이든 lowering 후 같은 CmdL4 경계에서 한 번만
적용하면 된다. 모든 효과의 default 는 OFF(identity) 이라 backward-compatible.

---

## 17.2 가정

| 가정 | 의미 | 깨지는 case |
|---|---|---|
| 채널 독립 | throttle/brake/steer 가 서로 결합 없음 | brake-steer torque-steer 결합 |
| 1D per-channel | 각 명령은 scalar | 4-corner 개별 brake/torque (Lc1 에서 분배) |
| ZOH 명령 latch | tick 사이 명령 유지 | 고주파 명령 손실 |
| 등가 steering inertia | 조향계를 단일 관성으로 환산 | 컬럼 compliance·다물체 |
| 마찰 = LuGre | presliding+Stribeck+viscous | 온도·마모 의존 마찰 |

---

## 17.3 Per-channel: FOPDT + rate limit + saturation

각 채널의 기본 파이프라인 (throttle 이 순수 예):

$$
u_d(t) = u_{\text{cmd}}(t - L) \quad\text{(transport delay, ring buffer)}
$$
$$
\tau\,\dot y + y = u_d \quad\text{(first-order lag)}
$$
$$
\dot y_{\text{rl}} = \mathrm{clip}\!\left(\frac{y - y_{\text{prev}}}{\Delta t},\, -r,\, +r\right),\quad
y_{\text{sat}} = \mathrm{clip}(y_{\text{rl}},\, y_{\min},\, y_{\max})
$$

- $L$ = `dead_time_s` (순수 수송 지연), $\tau$ = `tau_s` (1차 lag), $r$ =
  `rate_limit` (slew, ≤0 이면 off), $[y_{\min}, y_{\max}]$ = saturation.
- $L$ 과 $\tau$ 의 조합 = **FOPDT** $G(s)=e^{-Ls}/(1+\tau s)$ — powertrain 의
  표준 저차 모델 (ch15 ref #19, #22).

---

## 17.4 Dead-zone — pedal tip-in / pad clearance

명령 크기가 임계 $d$ 이하면 출력 0, 초과분은 0..1 로 **재정규화**:

$$
u' = \begin{cases} \dfrac{u - d}{1 - d} & u > d \\[4pt] 0 & u \le d \end{cases}
$$

- throttle: 페달 초기 유격 / 모터 torque threshold (tip-in).
- brake: 패드가 디스크에 닿기 전 채워지는 유격 (pad clearance fill).
- 0..1 one-sided 신호 전용 (부호 있는 steer 에는 부적용 — 그건 backlash 의 영역,
  ch15 ref #4/#16 참조, 현재 미구현).
- $d$ 직후는 0 에서 연속 출발하고 full 명령 1.0 은 여전히 full 출력.

---

## 17.5 Steering — 두 모드 (lag vs torque servo)

스티어링은 마찰(LuGre) on/off 로 모델이 갈린다.

### Mode A — first-order lag (LuGre off)

$$
\tau\,\dot\delta + \delta = \delta_{\text{cmd}}(t-L), \quad
\delta \leftarrow \mathrm{clip}(\delta, \delta_{\min}, \delta_{\max})
$$

수동적 1차 추종. 빠른 근사가 필요할 때.

### Mode B — torque servo + inertia + LuGre (권장)

조향계를 road-wheel 각 $\theta$ 기준 등가 관성 $I$ 로 보고, **PD 위치 서보**가
명령각으로 끌고 가며 LuGre 마찰 $T_f$ 가 저항한다.

```
 cmd ─►(+)─ e=cmd−theta ─►[kp]──┐                         Tf (LuGre)
        ▲                       +                            │
        │                    (sum)─► Tservo ─►(−)─► acc=(Tservo−Tf)/I ─►∫─► theta_dot ─►∫─► theta
        │                       −                                                          │
        │                    [kd]◄─ theta_dot                                              │
        └──────────────── feedback: theta ◄────────────────────────────────────────────────┘
```

- **입력(reference)**: $\delta_{\text{cmd}}$ (dead-time 지연 후 명령각).
- **출력**: $\theta$ = 실제 road-wheel 각 → tire 모델로 전달.
- **오차 → 출력**: 위치오차 $e=\delta_{\text{cmd}}-\theta$ 로 **토크**를 낸다.

$$
T_{\text{servo}} = k_p\,(\delta_{\text{cmd}} - \theta) - k_d\,\dot\theta
$$
$$
I\,\ddot\theta = T_{\text{servo}} - T_f
$$

닫힌 루프는 **mass–spring–damper**:

$$
I\,\ddot\theta + k_d\,\dot\theta + k_p\,\theta = k_p\,\delta_{\text{cmd}} - T_f
$$
$$
\omega_n = \sqrt{k_p/I}, \qquad \zeta = \frac{k_d}{2\sqrt{k_p\,I}}
$$

정상상태 ($\ddot\theta=\dot\theta=0$):

$$
\theta_{ss} = \delta_{\text{cmd}} - \frac{T_f}{k_p}
$$

**왜 PD(적분 없음)인가**: $I$(적분)항을 넣으면 정상오차 $T_f/k_p$ 를 0 으로 강제해
마찰을 지운다. PD 로 두면 마찰이 만드는 작은 정적 조향오차(stiction, on-center
dead-band) 가 그대로 남아 실제 EPS 거동을 표현한다. travel stop 은 위치 clamp +
속도 kill (anti-windup) 로 처리하고 적분기 상태 $\theta$ 는 보존한다.

### 등가 inertia 의 구성 (ratio² 환산)

`steer.inertia` 는 모든 조향부품을 $\theta$ 좌표로 환산한 등가값: $I_{\text{eff}} =
\sum_i I_i\,(\omega_i/\dot\theta)^2$.

| 요소 | 환산 | 비중 |
|---|---|---|
| 앞바퀴+허브+너클 (kingpin yaw) | ×1 | load 측 주관성 |
| rack (병진 $m$) | $m\,(\partial x_{\text{rack}}/\partial\theta)^2$ | 중 |
| 컬럼·핸들 (steering ratio $G$) | $\times G^2$ | $G^2{\approx}225$ 로 증폭 |
| **EPS 모터 rotor** | $\times G_{\text{motor}}^2$ | **보통 지배적** |

작은 모터 rotor(~$10^{-4}$)도 기어비 수백의 제곱이면 수 kg·m² 급이 되어 등가관성을
지배한다. default 0.02 kg·m² 는 이 합을 $\theta$ 기준으로 모은 작은 값.

---

## 17.6 LuGre dynamic friction

bristle 변형 $z$ 의 internal state 로 presliding hysteresis + Stribeck + stick-slip
을 표현한다.

$$
g(\omega) = T_c + (T_s - T_c)\,e^{-(\omega/\omega_s)^2}
$$
$$
T_f = \sigma_0\,z + \sigma_1\,\dot z + \sigma_2\,\omega
$$

stiff 한 $\sigma_0$ 항 때문에 explicit Euler 는 불안정 → **semi-implicit** bristle
업데이트 (무조건 안정):

```
z_{k+1} = (z + h*w) / (1 + h*sigma0*|w|/g(w))
```

- $\sigma_0$ bristle stiffness, $\sigma_1$ bristle damping, $\sigma_2$ viscous,
  $T_c$ Coulomb, $T_s$ static(breakaway), $\omega_s$ Stribeck velocity.
- $\dot z = (z_{k+1}-z)/h$ 로 $T_f$ 계산. (참고: ch15 ref #2 Canudas-de-Wit LuGre,
  #1 Beal&Brennan 의 실차 steering friction ID.)

---

## 17.7 Brake thermal fade — mu(T)

연속·강한 제동에서 디스크 온도가 올라 마찰계수가 떨어지는 fade:

$$
\dot T = c_{\text{heat}}\,|u_b|\,|v| - c_{\text{cool}}\,(T - T_{\text{amb}})
$$
$$
\mu_{\text{scale}} = \mathrm{lerp}\big(T;\ \{T_k\}, \{\mu_k\}\big)
$$

- 발열 ∝ brake 명령 × 속도, 냉각 ∝ 온도차. $\mu_{\text{scale}}$ 는 breakpoint
  테이블 보간 (비면 1.0). disabled 면 fade 없음. (ch15 ref #11/#12 EMB clamp.)

---

## 17.8 Sensor delay — feedback transport

제어기가 보는 상태는 측정·통신 지연만큼 과거다. State snapshot 을 ring buffer 에
담아 `delay_s` 만큼 지연시켜 반환한다 (controller feedback path).

$$
x_{\text{meas}}(t) = x_{\text{true}}(t - \tau_{\text{sensor}})
$$

UKF/MPC 의 feedback latency robustness 시험에 직결 (thesis 연계).

---

## 17.9 VDSim 구현 노트

> **[VDSim impl] § 17.5 — steering servo (LuGre on)**
>
> `core/src/actuator_model.cpp`:
> ```cpp
> const double Tservo = p_.steer.servo_kp * (s_cmd - steer_pos_)
>                     - p_.steer.servo_kd * w;            // w = steer_vel_
> const double acc = (Tservo - Tf) / std::max(1e-6, p_.steer.inertia);
> steer_vel_ += acc * h;  steer_pos_ += steer_vel_ * h;   // double integrator
> ```
> LuGre off 이면 `first_order(steer_lag_, s_cmd, tau_s, dt)` 경로.

> **[VDSim impl] § 17.4 — dead-zone (throttle 와 brake 공통)**
>
> `ChannelActuator::dead_zone` 한 곳에 통합: `(u - dz)/(1 - dz)`. throttle 은
> `p_.throttle.dead_zone`, brake 는 `p_.brake.ch.dead_zone` (pad clearance).

> **[VDSim impl] § 17.6 — semi-implicit LuGre**
>
> explicit Euler 가 sigma0 로 발산해 bristle 만 implicit:
> `z=(z+h*w)/(1+h*sigma0*|w|/g)`. 나머지(servo 적분)는 명시적.

> **[VDSim impl] — GUI**
>
> Plant 창 "Actuator & feedback" 에서 채널별 파라미터 편집 + **step-response
> plot** (명령 대 realized). LuGre 토글로 servo/lag 모드가 바뀌며 무관 파라미터는
> grey-out (tau ↔ servo kp/kd).

---

## 17.10 Self-check

<details>
<summary>1. 서보가 PID 아니라 PD 인 이유?</summary>

적분항은 정상오차 $T_f/k_p$ 를 0 으로 강제해 마찰의 정적 조향오차(stiction)를
지운다. PD 로 두어야 on-center dead-band 같은 실제 EPS 거동이 남는다.
</details>

<details>
<summary>2. 작은 EPS 모터 rotor 가 왜 등가 steering inertia 를 지배할 수 있나?</summary>

$\theta$ 기준 환산이 속도비의 제곱이라, 모터가 road-wheel 보다 $G_{\text{motor}}$
(수백) 배 빨리 돌면 $I_{\text{motor}}\,G_{\text{motor}}^2$ 로 증폭된다.
</details>

<details>
<summary>3. dead-zone 을 steer 에 그대로 쓰면 안 되는 이유?</summary>

dead-zone 식 $(u-d)/(1-d)$ 은 0..1 one-sided 신호용. 부호 있는 steer 의 방향
반전 lost-motion 은 backlash 연산자가 필요 (별개 물리, 현재 미구현).
</details>

<details>
<summary>4. LuGre 를 semi-implicit 으로 적분하는 이유?</summary>

$\sigma_0$ 가 매우 커서 explicit Euler 의 안정영역을 벗어난다. bristle 항만
implicit 으로 풀면 무조건 안정.
</details>

---

## References

상세 bibliography 와 실측 ID 문헌은 `docs/references/actuator_nonlinearity.md`.
핵심: LuGre(#2), steering friction physics(#1), steering/driveline backlash
(#4·#15·#16), EMB/EHB clamp+dead-zone(#9·#11·#12), FOPDT for MPC(#19·#22).
