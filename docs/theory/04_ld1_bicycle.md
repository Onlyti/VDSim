# 04. Ld1-Bicycle (Single-Track, 5 DOF)

> **학습 목표.** Single-track bicycle 의 가정과 한계를 명확히 한다. 5 DOF state vector 의 의미를 안다. linear-bicycle 의 analytical steady-state yaw rate 공식을 유도하고, 그게 왜 simulator-side 의 검증 baseline 이 되는지 안다. weight transfer (longitudinal) 의 1-step lag 처리 이유를 안다.

## 4.1 왜 Single-Track 부터 시작하나

차량 dynamics 의 모든 핵심 거동 (cornering, braking, accel) 이 single-track 만으로 정성 정확하게 표현된다. 4-wheel per-tire (Ld2) 는 weight transfer 의 quantitative 정확도를 더하지만, **policy / controller 개발 단계에서는 Ld1 이면 충분**. controller 가 Ld1 에서 동작하면 Ld2-Ld3 로 옮겨도 거의 그대로 동작 (안 그러면 controller 가 invariance 가 부족).

VDSim 의 Ld1 = "the L4 controller can drive any vehicle" 의 testbed.

## 4.2 가정 (정리)

| 가정 | 의미 |
|---|---|
| Single-track | front 양 wheel 을 axle 평균으로, rear 도 동일. wheel 수 = 2 (axle 단위). |
| Planar | z 방향 모션 없음. roll, pitch = 0. yaw 만. |
| Static Fz | weight transfer 가 있어도 axle 단위. lateral transfer 무시. |
| Per-axle Pacejka | 각 axle 의 Fz 와 평균 slip 으로 Pacejka 1 회 call. |
| Rigid wheel-spin | wheel 회전 에너지는 모델링되지만 wheel-axle 의 spin inertia 는 단순한 disk approximation. |

## 4.3 State vector (5 DOF)

본 PoC 의 Ld1 적분 상태는:
```
y = [x_w, y_w, ψ, vx, vy, r, ω_f, ω_r]
```
실제 DOF count 는 5 (vx, vy, ψ + 2 wheel spin), 적분기 입장에서는 8 개 first-order ODE (포지션 3 개는 kinematic).

`State` struct 매핑:
- `position.x`, `position.y` ← `x_w`, `y_w`
- `orientation` (quat) ← `quat_from_euler(0, 0, ψ)`
- `velocity.x`, `velocity.y` ← `vx`, `vy`
- `angular_velocity.z` ← `r`
- `wheel_spin[FL] = wheel_spin[FR] = ω_f`
- `wheel_spin[RL] = wheel_spin[RR] = ω_r`

코드 `core/src/bicycle_dynamics.cpp:89-99` 의 `Deriv` struct 이 위와 1-to-1.

## 4.4 Bicycle EoM (식)

Body-frame Newton-Euler (Chapter 02 결론) + per-axle tire force:

```
m · v̇x  =  Fx_total  +  m · vy · r                   (body-x EoM)
m · v̇y  =  Fy_total  −  m · vx · r                   (body-y EoM)
Izz · ṙ =  a · Fy_body_f  −  b · Fy_body_r  +  ΣMz_wheel  (yaw)

I_wheel · ω̇_f  =  T_drive_f  +  T_brake_f  −  Fx_wheel_f · R   (wheel-spin)
I_wheel · ω̇_r  =  T_drive_r  +  T_brake_r  −  Fx_wheel_r · R
```

여기서:
- `Fx_total = Fx_body_f + Fx_body_r − F_aero − F_rr` (drag + rolling 합쳐서).
- `Fy_total = Fy_body_f + Fy_body_r`.
- `Fx_body_f = Fx_wheel_f · cos(δ) − Fy_wheel_f · sin(δ)` (wheel frame → body frame, 회전).
- `Fy_body_f = Fx_wheel_f · sin(δ) + Fy_wheel_f · cos(δ)`.
- Rear: `Fx_body_r = Fx_wheel_r`, `Fy_body_r = Fy_wheel_r` (un-steered).
- `a, b` = CG-to-front, CG-to-rear distance.

코드 `core/src/bicycle_dynamics.cpp:165-203` 가 정확히 위 식.

## 4.5 Per-axle Fz with longitudinal weight transfer

Static Fz:
```
Fz_f_static  =  m · g · b / L
Fz_r_static  =  m · g · a / L
```

Longitudinal weight transfer (Ld1 는 axle 단위):
```
ΔFz_long  =  m · ax · h_cg / L

Fz_f  =  m·g·b/L  +  Fz_aero_f  −  ΔFz_long
Fz_r  =  m·g·a/L  +  Fz_aero_r  +  ΔFz_long
```

부호 직관:
- 가속 (ax > 0) → ΔFz_long > 0 → rear 가 더 loaded, front 가 덜 loaded.
- 제동 (ax < 0) → ΔFz_long < 0 → front 가 더 loaded (nose dive).

### 1-step lag — 왜 필요한가

`ax` 는 `Fx_total / m`. `Fx_total` 은 tire Fx 의 합. tire Fx 는 Pacejka 가 Fz 의 함수. Fz 는 ax 의 함수.
즉 self-referential: `ax = f(Fz(ax))`. 풀려면 fixed-point iteration.

본 PoC 는 **1-step lag** 사용:
```
Fz(t)  =  Fz_static  +  m · ax_prev · h_cg / L
```

`ax_prev` 는 직전 substep 의 `ax` 값. 다음 substep 에서 update. RK4 substep dt = 1 ms 이라 1-step lag bias 가 작음 (verified: Task 17 의 brake step 에서 분석값 vs 측정값 < 0.1 %).

코드 `core/src/bicycle_dynamics.cpp:120-127`:
```cpp
const double dFz_long = m * ax_prev_ * h_cg / L;
double Fz_f = m * kGravity * b / L + Fz_aero_f - dFz_long;
double Fz_r = m * kGravity * a / L + Fz_aero_r + dFz_long;
```

### Aero downforce 더하기

```
q  =  0.5 · ρ_air · A · vx · |vx|
Fz_aero_f  =  Cl_f · q
Fz_aero_r  =  Cl_r · q
```

`vx · |vx|` 표기 — 후진 시 부호 반전.
default Cl_f = Cl_r = 0 (sedan 은 거의 효과 없음). sports / race / FSK 에서 nonzero.

## 4.6 Drive / brake torque 분배

Drive split 은 `drive_type`:
- FWD: front axle 만.
- RWD: rear axle 만 (default for sedan).
- AWD: front/rear 50:50.

Brake 는 `brake_bias_front`:
```
Tb_f  =  bias · cmd.brake · max_brake_torque
Tb_r  =  (1 − bias) · cmd.brake · max_brake_torque
```

`brake_ebd_enabled = true` 이면 dynamic bias:
```
bias  =  clamp(Fz_f / (Fz_f + Fz_r),  0.05,  0.95)
```

brake 시 front 가 더 loaded → bias 가 자동 증가 → front 가 더 많이 brake → ABS 효과 유사.

Smooth sign 함수 `tanh(ω / w)` 로 wheel spin 부호 부드럽게 처리 — 정지 근처 발진 방지.

## 4.7 Linear-bicycle analytical steady-state

검증 baseline 이 되는 식. **Cornering 의 linear region** 에서 SS yaw rate.

가정:
- 작은 α (linear region) → `Fy = −Cα · α` (Pacejka 의 linear-region slope, `Cα = B·C·D·Fz·μ`).
- `vx = const`.
- v̇y = 0, ṙ = 0 (SS).
- vy ≪ vx.

EoM:
```
m · vx · r  =  Fy_total                  (body-y SS)
Izz · 0    =  a · Fy_f − b · Fy_r        (yaw SS)
```

α_f, α_r 의 linear 표현:
```
α_f  =  atan2(vy + a · r, vx) − δ  ≈  (vy + a · r) / vx − δ
α_r  =  atan2(vy − b · r, vx)      ≈  (vy − b · r) / vx
```

(주의: ISO 8855 RH 부호. SAE convention 의 `δ − atan(...)` 와 다름.)

`Fy_f = −Cf · α_f`, `Fy_r = −Cr · α_r`. 위 EoM 에 대입:

```
m · vx · r  =  −Cf · (vy + a·r)/vx + Cf · δ  −  Cr · (vy − b·r)/vx
0           =  a · [−Cf · (vy + a·r)/vx + Cf · δ]  −  b · [−Cr · (vy − b·r)/vx]
```

정리:
```
[ (Cf + Cr)/vx               (a·Cf − b·Cr)/vx − m·vx ]  [vy]   [Cf · δ  ]
[ (a·Cf − b·Cr)/vx           (a²·Cf + b²·Cr)/vx       ]  [r ] = [a·Cf · δ]
```

이 2×2 선형 시스템의 r 해:
```
r  =  (A11 · B2 − A21 · B1) / det(A)
```

코드 `tests/integration/test_bicycle_steady_state.cpp:42-70` 의 `analytical_yaw_rate` 가 위 식 그대로.

### Special case: neutral steer

`a · Cf = b · Cr` (front and rear cornering stiffness times lever arm 동일) 이면 understeer gradient 가 0 — neutral steer.
이 경우:
```
r_neutral  =  vx · δ / L          (Ackerman steady-state)
```

VDSim default tire 의 경우 `Cf = B_lat · C_lat · D_lat · Fz_f · μ`, `Cr = B_lat · C_lat · D_lat · Fz_r · μ`. ratio `Cf / Cr = Fz_f / Fz_r = b / a`. 따라서 `a · Cf = b · Cr` 자동 성립 → neutral steer.

이게 우연이 아니라 **default 의 tire stiffness 가 mass 분포에 맞춰 linearly scale 되기 때문**. 차종이 다르면 깨짐.

### Understeer gradient

```
K_us  =  m / L · (b / Cf − a / Cr)
```

`K_us > 0` → understeer. `K_us < 0` → oversteer. neutral = 0.

SS yaw rate:
```
r_ss  =  vx · δ / [ L · (1 + K_us · vx²) ]
```

`vx → ∞` 한계 — understeer 차량은 r 의 증가가 둔화, oversteer 는 발산.

## 4.8 검증 — analytical vs simulator

`BicycleSteadyState.LeftTurnYawRateMatchesAnalytical`:
- vx0 = 10 m/s, δ = 0.05 rad, 5 s 적분.
- `r_sim ≈ 0.176 rad/s`, `r_ana ≈ 0.185 rad/s`. 오차 −5 %.
- analytical 이 linear bicycle (no Mz aggregation) 이라 −2.6 % 는 Mz 기여, 나머지 −2-3 % 는 small non-linearity 영역에 들어선 효과.

`StepSteerSweep.LinearRegionWithinTenPercent`:
- 3 × 5 grid (vx, δ) 에서 ay < 3 m/s² 인 cell 모두 `|err| ≤ 10 %`.

`LongScenarios.DragCoastMatchesAnalytical`:
- aero drag only (RR=0). 20 m/s 에서 10 s coast. analytical `vx(t) = vx0 / (1 + vx0 · k · t)` (k = ½ρCdA / m) 과 ±5 %.

이런 closed-form 비교가 simulator-side 의 "correctness" 의 강한 evidence.

## 4.9 한계

| 항목 | 한계 |
|---|---|
| Lateral weight transfer | 없음 (Ld2 부터) |
| Per-tire diff / differential | 없음 (axle 평균) |
| Combined slip 의 advanced 식 | friction-ellipse rescale 만 |
| Suspension dynamics | 없음 (Ld3 부터) |
| Driver model | 외부 controller 사용 |

이 한계 모두 Ld2-Ld5 의 이유. 그러나 Ld1 자체가 controller dev 용으로는 sufficient.

## 4.10 사용 패턴 (code)

```cpp
vdsim::VehicleParams vp = vdsim::VehicleParams::from_yaml("configs/vehicles/sedan.yaml");
vdsim::TireParams    tp = vdsim::TireParams::from_yaml("configs/tires/default_pacejka.yaml");
vdsim::SolverParams  sp;

auto dyn = vdsim::create_bicycle();
dyn->initialize(vp, tp, sp);

vdsim::State s0; s0.velocity = {10, 0, 0};
dyn->reset(s0);

vdsim::ContactArray contacts;          // flat ground, mu = 1
for (auto& p : contacts) { p.is_valid = true; p.normal = {0,0,1}; p.mu_long = p.mu_lat = 1; }

vdsim::CmdL4 cmd; cmd.steer_angle_wheel = 0.05;
for (int i = 0; i < 1000; ++i) dyn->step(cmd, contacts, 0.005);

std::cout << "yaw rate = " << dyn->state().yaw_rate() << "\n";
```

이게 PoC 의 minimal controller-on-bicycle 루프. Lc4-Pedal level. Lc5+ 는 ControlConverter 위에 cascade.

## 4.11 참고

- Genta, *Motor Vehicle Dynamics*, §4 (single-track), §5 (understeer/oversteer).
- Rajamani, *Vehicle Dynamics and Control*, §2 — bicycle derivation (단, SAE).
- VDSim 의 검증 코드: `tests/integration/test_bicycle_steady_state.cpp`.
