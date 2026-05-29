# Task 04 — D8 동역학 사다리 L1 / L2 / L3 명세

| Field | Value |
|---|---|
| Task ID | D8 |
| Type | Design |
| Date | 2026-05-28 |
| Commit | `1e15c5e` (헤더) / `04ce58e` (L1 impl) |
| Status | completed (L1 impl + L2/L3 spec) |

## 1. 목적

세 fidelity 레벨의 state vector, equations of motion, integration 정책을 PoC 시작 전에 lock. 이게 모호하면 L1 / L2 / L3 사이 transition 시 ABI 가 깨진다. 사용자 결정: PoC W12 안에 L3 14-DOF 까지 구현 (인수인계 문서 권장 L2 7-DOF 보다 욕심 있는 목표).

## 2. 구현 방법

### L1 Bicycle (single-track planar, 2D)

| 항목 | 값 |
|---|---|
| Effective DOF | 5 (vx, vy, yaw + 2 wheel spin) |
| State 활용 | position(x,y,0), orientation(yaw-only), velocity(vx,vy,0), angular_velocity(0,0,r), wheel_spin[FL=FR=ωf, RL=RR=ωr] |
| Tire input | per-axle Fz (정적 m·g·b/L, m·g·a/L), no weight transfer |
| Pose integration | quaternion ODE — yaw-only |
| Integrator | RK4, dt_substep = 1 ms |

Equations (body frame, ISO 8855 RH):

```
m·v̇x = m·vy·r + Fx_body − F_aero
m·v̇y = −m·vx·r + Fy_body
Izz·ṙ = a·Fy_body_f − b·Fy_body_r
I_wheel·ω̇_i = T_drive_i + T_brake_i − Fx_wheel_i · R

α_f = atan2(v_fy_wheel, v_fx_wheel)        # ISO 8855: NOT "δ − atan(...)"  (Rajamani's SAE Y-right convention)
α_r = atan2(vy − b·r, vx)
κ_i = (R·ω_i − v_long_i) / max(|v_long_i|, 0.5)
```

### L2 7-DOF

| 항목 | 값 |
|---|---|
| Effective DOF | 7 (vx, vy, yaw rate + 4 wheel spin) |
| State 활용 | L1 + 4 wheel_spin 독립 |
| Tire input | per-tire Fz with **lateral & longitudinal weight transfer** |
| Suspension | 없음 (정적 + 보정) |

Fz weight transfer:
```
Fz_static_f = m·g·b/(2L)
Fz_static_r = m·g·a/(2L)
ΔFz_long  = m·ax·h_cg/L
ΔFz_lat_f = m·ay·h_cg/Tw_f · K_φ_f/(K_φ_f+K_φ_r)
ΔFz_lat_r = m·ay·h_cg/Tw_r · K_φ_r/(K_φ_f+K_φ_r)
Fz_FL = Fz_static_f − ΔFz_long/2 − ΔFz_lat_f
...
```
Roll-stiffness ratio 로 lateral transfer 분배 (D8 결정 — `roll_stiffness_front/rear` 사용).

### L3 14-DOF (Genta 표준)

| DOF 분해 | 수 |
|---|---|
| Sprung translation (x, y, z) | 3 |
| Sprung rotation (φ, θ, ψ) | 3 |
| Unsprung vertical (FL/FR/RL/RR) | 4 |
| Wheel spin | 4 |
| **Total** | **14** |

Suspension equation (per corner i):
```
z_sprung_corner_i = z_cg + r_i,x·sin(θ) − r_i,y·sin(φ) + r_i,z·cos(θ)·cos(φ)
susp_compression_i = z_sprung_corner_i − z_unsprung_i − nominal
F_susp_i = k_i · compression + c_i · velocity
m_unsprung·z̈_unsprung_i = F_susp_i + Fz_tire_i − m·g
```

Anti-roll bar: `K_roll_eff = K_spring + K_arb` (linear approx).

Pose integration: quaternion ODE `q̇ = 0.5·q ⊗ [0, ω_body]`, normalize every step.

### 사다리 비교 표

| Level | DOF | Weight transfer | Susp dynamics | Roll/pitch | Use case |
|---|---|---|---|---|---|
| L1 Bicycle | 5 | 없음 (static Fz) | 없음 | 무 | 빠른 검증, controller 개발 |
| L2 7-DOF | 7 | 있음 (정적 보정) | 없음 | 무 | 일반 자율주행 평가 |
| L3 14-DOF | 14 | dynamic (susp force 기반) | 있음 | 있음 | OEM 동역학 검증, ride/handling |

## 3. 검증 방법 (근거)

각 level 의 검증은 별도 task:
- L1: task 11 (steady-state cornering 해석해)
- L2: 미구현 (W9-W10 계획)
- L3: 미구현 (W11-W12 계획)

명세 자체는 self-consistency:
- L1 state subset ⊂ L2 state subset ⊂ L3 state subset (struct 호환)
- 모든 level 에서 같은 `step()` 시그니처
- ISO 8855 RH frame 일관 (Rajamani SAE 변환 주의)

## 4. 검증 결과

### L1 구현 검증 (task 11 보고서 인용)

| 시나리오 | 통과 기준 | 결과 |
|---|---|---|
| Left turn yaw rate (δ=0.05, vx=10) | 해석해 대비 10% 이내 | pass (자세한 수치는 task 11) |
| Right turn yaw rate | 부호 + 크기 대칭 | pass |
| Zero steer = straight | yaw rate / y 위치 1e-6 | pass |
| Low mu reduces yaw rate | r(μ=0.3) ≤ r(μ=1.0) | pass |

### L2 / L3 명세 self-consistency

| 항목 | 결과 |
|---|---|
| State struct 가 L3 까지 한 번에 cover | pass (`State::wheel_spin`, `susp_*`) |
| L2/L3 factory throw on call | pass (`BicycleStubs.SevenAndFourteenDoFThrow`) |

## 5. 판단

- 결과: **partial** (L1 구현 완료 + 검증 통과, L2/L3 는 명세만)
- 근거: L1 의 analytical agreement 가 10% 이내 (실제 측정값은 task 11 §4 참조). L2/L3 명세는 표준 Genta 모델 + ISO 8855 적용으로 self-consistent. 단 PoC W12 timeline 으로 L3 까지 가는 것은 빡빡함을 인지.
- Follow-up:
  - L2 7-DOF impl (W9-W10).
  - L3 14-DOF impl + roll/pitch validation (W11-W12).
  - L2 의 ax / ay 가 self-referential (Fz → tire force → ax → Fz). 첫 iteration 은 ax_prev 사용 (1-step lag) 검토.
