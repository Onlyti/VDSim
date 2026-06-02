# 08. Lc5-AxTarget / Lc6-VTarget — PI + Feed-Forward Cascade

> **학습 목표.** PI controller 의 anti-windup 이 왜 필요한지, feed-forward 가 어떻게 transient response 를 빠르게 하는지 식 단위로 안다. cascade 구조 (Lc6 → Lc5 → Lc4) 에서 inner loop bandwidth 가 outer loop 보다 빨라야 하는 이유를 안다. PoC 의 throttle / brake 자동 분기가 어떤 가정 위에 있는지 명확히 한다.

## 8.1 왜 PI + FF 인가 (not PID, not pure FF)

차량의 longitudinal dynamics 는 (PoC range 내에서) **1차에 가까운 비선형 시스템**:

$$
\dot v_x \approx \frac{T_{\text{drive}}\, \eta_{\text{drivetrain}} - F_{\text{aero}} - F_{rr}}{m}
$$

$F_{\text{aero}} \sim v_x^2$ 는 weak nonlinearity. 작은 perturbation 영역에서는 1차 system.

- **Pure P** — steady-state error 존재 (drag 보상 못 함).
- **PI** — integrator 가 SS error 0 으로 만듦.
- **PID** — D term 이 noise amplify, vx 신호의 noise 가 크지 않아 D 의 이득보다 손해 큼.
- **PI + FF** — feed-forward 가 transient 부분, integrator 가 SS bias.

본 PoC 의 `LongAxController` (Lc5):

$$
u = K_p\, e + K_i \int e\, dt + K_d\, \dot e + K_{ff}\, a_{x,\text{target}}
$$

PoC default $K_d = 0$, $K_{ff} = 0.10$ (typical).

## 8.2 Lc5-AxTarget 의 식

`ax_target` (m/s²) → throttle / brake [0, 1].

```cpp
e        =  ax_target − ax_meas
integ   +=  e · dt              // clamped to ±i_max
de_dt    =  (e − prev_e) / dt   // first call: 0
u        =  Kp · e + Ki · integ + Kd · de_dt + Kff · ax_target

if u ≥ 0:   throttle = clamp(u, 0, 1),  brake = 0
else:       throttle = 0,                brake = clamp(−u, 0, 1)
```

코드 `core/src/control_converter.cpp` 의 `LongAxController::update`.

### Throttle / brake 자동 분기

`u` 의 sign 으로 dispatch. throttle 과 brake 가 동시에 0 이 아닌 경우 없음 (mutually exclusive).

이게 단순한 가정. 실제로는:
- regenerative brake 가 있는 EV 에서 `throttle < 0` 의미 가능.
- 갑작스러운 sign 전환 시 actuator dead-time 으로 진동 가능.

본 PoC 는 simple bang-bang 분기. follow-up:
- `throttle_brake_overlap_band` — sign 전환 시 dead-zone.
- regen mode (negative throttle).

### Anti-windup

`integ` 를 ±`i_max` 로 clamp. saturation 영역에서 integrator 가 무한히 누적되어 desaturation 후 overshoot 만드는 현상 (windup) 방지.

cap 값:
```
i_max = 2.5
```

`Ki = 0.60` 이면 integrator 단독 최대 contribution = `Ki · i_max = 1.5` (throttle 기준 1.5x — 이미 clamp 됨).
즉 i_max 가 작아도 충분.

### Feed-forward 의 의미

`Kff · ax_target` 항이 transient 응답 빠르게.

직관: ax_target = 3 m/s² 입력 시, 모델이 throttle ≈ 0.6 (sedan 추정) 가 필요함을 미리 안다. FF 가 0.10 · 3 = 0.30 을 즉시 출력 + PI 가 보정.

FF gain 결정:

$$
K_{ff} \approx \frac{m R}{T_{\max}} \approx \frac{1500 \cdot 0.32}{300} \approx 1.6 \;\text{s}^2/\text{m}
$$

이 값을 normalized throttle 으로 변환: throttle = T / T_max → Kff_norm = 0.10 정도.

차종별 calibration 필요. sedan 기준 default.

## 8.3 Lc6-VTarget — Cascade PI

`v_target` (m/s) → `ax_target` (m/s²).

```cpp
e        =  v_target − vx_meas
integ   +=  e · dt          // clamped to ±i_max
ax_out   =  Kp · e + Ki · integ
ax_target =  clamp(ax_out, −ax_clamp, +ax_clamp)
```

코드 `core/src/control_converter.cpp` 의 `LongVxController::update`.

default gains: `Kp = 0.8, Ki = 0.20, ax_clamp = 3.5`.

### Cascade 구조의 이점

```
v_target  →  Lc6 PI  →  ax_target  →  Lc5 PI+FF  →  throttle/brake  →  Plant
```

- **Outer loop (Lc6)** 의 dynamics 가 inner loop 보다 느림.
- **Inner loop (Lc5)** 가 빠르게 ax 를 tracking → outer loop 입장에서 ax 가 거의 즉시 따라가는 1차 system 으로 보임.
- gain 분리 가능 → 개별 tuning 쉽다.
- saturation 분리 (ax 한계와 throttle 한계 별도).

이게 industrial cascade control 의 표준 pattern.

### Inner / outer bandwidth 분리

empirical guide:

$$
\omega_{\text{inner}} \ge 3\, \omega_{\text{outer}} \quad (\text{transient decoupling})
$$

본 PoC default:
- Lc5: Kp = 0.4 (ax control loop dominant pole ~ 5-10 rad/s)
- Lc6: Kp = 0.8 (v control loop ~ 1-2 rad/s)

ratio ~ 5. 충분 separation.

## 8.4 Cascade 의 한계 — saturation 의 전파

만약 ax_target > 실제 차량의 max ax (e.g., FSK 의 가속 한계 ~ 6 m/s²) 이면:
- Lc5 가 throttle = 1.0 saturate.
- Lc6 의 integrator 가 계속 누적 (실제 ax 가 못 따라가니 vx error 누적).
- Anti-windup 으로 cap 되지만 outer-loop wind-down 까지 시간 지연.

본 PoC 의 `vdsim_ax_track_demo` (Task 25) 의 결과:
- accel phase (ax_target=+2): mean 0.58 (cap), RMSE 1.42.
- 차량 한계 노출.

해결책 (follow-up):
- Lc6 의 `ax_clamp` 를 차종 max ax 로 자동 조정.
- Conditional integration (saturation 영역에서 integrator hold).
- back-calculation anti-windup.

## 8.5 Driver model 의 PI gain (참고)

`DriverModel` (Chapter 10 상세) 가 내부에서 cascade 사용:
- vx PID: Kp = 0.6, Ki = 0.15 (slower than auto cascade)
- ax PID: default (Lc5 default 그대로)

인간 driver 의 reaction 이 controller 보다 느림 → outer loop gain 도 낮게.

## 8.6 검증 (Task 25)

`LongAxController.*` 7 tests:
- `ZeroErrorZeroOutput`
- `PositiveTargetUsesThrottle`
- `NegativeTargetUsesBrake`
- `IntegratorAccumulates`
- `ResetClearsState`
- `OutputsAreClamped`
- `IntegratorAntiwindup` — sustain error 입력 후 `|integ| ≤ i_max`.

`LongVxController.*` 4 tests:
- `ZeroErrorZeroOutput`
- `PositiveErrorPositiveAx`
- `NegativeErrorNegativeAx`
- `OutputClamped`

11 tests pass.

### End-to-end (vdsim_ax_track_demo)

step-target ax = (0 → +2 → 0 → −3 → 0) 으로 10 s.

| Phase | ax_target | mean ax | RMSE | 해석 |
|---|---:|---:|---:|---|
| accel | +2 | +0.58 | 1.42 | actuator saturation |
| coast | 0 | +0.47 | 0.48 | momentum + integrator residual |
| brake | −3 | −1.51 | 2.24 | tire saturation |
| idle | 0 | 0 | 0.02 | drift 최소 |

위 결과가 saturation 의 실전 거동을 보여줌. controller correctness + plant limitation 동시 확인.

## 8.7 한계 / follow-up

| 항목 | 한계 |
|---|---|
| Pure linear PI | gain scheduling 없음 (vx, mu 별 별도 tuning) |
| Throttle/brake mutually exclusive | regen 모드, hybrid braking 미반영 |
| Saturation feedback 없음 | conditional integration / back-calc 가 더 정확 |
| Disturbance feedforward 없음 | road slope, wind 등 외란 미반영 |
| Actuator dynamics 무시 | first-order throttle / brake lag 미반영 |
| 차종 calibration | Kff / Kp 가 sedan 기준 default, FSK / race 에서 별도 tuning 필요 |

## 8.8 사용 패턴

```cpp
vdsim::LongAxController axc;
vdsim::LongAxController::Gains g;
g.kp = 0.4; g.ki = 0.6; g.kff = 0.10;
axc.initialize(g);

vdsim::LongVxController vxc;
vxc.initialize({});

while (...) {
    double ax_target = vxc.update(v_target, dyn->state().vx(), dt);
    auto [thr, brk] = axc.update(ax_target, dyn->ax_body_est(), dt);
    vdsim::CmdL4 cmd; cmd.throttle = thr; cmd.brake = brk;
    cmd.steer_angle_wheel = steer_from_path_planner;
    dyn->step(cmd, contacts, dt);
}
```

`vdsim_path_tracking` 의 main loop 가 거의 그대로.

## 8.9 참고

- Aström, K.J. & Murray, R.M., *Feedback Systems*, Princeton, 2008 — §10 (PID 표준).
- Aström & Hägglund, *Advanced PID Control*, ISA, 2006 — §6 (anti-windup), §10 (cascade).
- Skogestad, S. & Postlethwaite, I., *Multivariable Feedback Control*, Wiley, 2005 — cascade structure 일반.
