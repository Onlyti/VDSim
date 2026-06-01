# 06. Ld3-FourteenDOF — Sprung 3 + Unsprung 4 Vertical DOF

> **학습 목표.** Lumped 14-DOF 모델의 state 가 어떻게 구성되는지, sprung body 의 heave / roll / pitch 와 unsprung mass 의 z 동역학이 어떻게 결합되는지 식 단위로 안다. anti-dive geometry 의 effective h_cg 의 의미를 안다. wheel-hop frequency 의 이론값과 simulator 의 실측값이 왜 차이 나는지 (corner damper 의 overdamped 한계) 정확히 설명한다. quasi-static (Ld2) 와 dynamic (Ld3) 의 차이가 어디서 드러나는지 정리한다.

## 6.1 왜 14-DOF 라고 부르나

D8 명세에서 14-DOF 의 분해:
- Sprung body translation (x, y, z) — 3
- Sprung body rotation (φ, θ, ψ) — 3
- Unsprung vertical z (FL, FR, RL, RR) — 4
- Wheel spin (FL, FR, RL, RR) — 4

합계 14.

본 VDSim 의 Ld3 는 위의 14 DOF 중:
- Planar motion (x, y, ψ, vx, vy, r) + 4 wheel spin — Ld2 inner 로 위임.
- 추가로 sprung vertical 3 (z_s, φ, θ) + unsprung vertical 4 (z_u_i) — 본 챕터의 핵심.

planar 7 + vertical 7 = **14 DOF (= 14 second-order ODE 또는 28 first-order)**.

## 6.2 Lumped vs full multibody

본 PoC 는 **lumped suspension**:
- sprung body 의 corner displacement = `z_s + ry_i · φ − rx_i · θ` (small-angle linearized).
- unsprung mass = lumped point mass, vertical 1 DOF.
- spring + damper 가 sprung corner 와 unsprung mass 사이에 작용.
- tire vertical stiffness `k_tire` 가 unsprung mass 와 ground 사이.

본격 multibody (Ld4-Ld5) 는 lumped 가정을 풀고 hardpoint + joint + bushing 으로 분해. 본 챕터는 lumped 14-DOF.

## 6.3 Sprung body EoM (heave / roll / pitch)

Per-corner spring/damper between sprung corner and unsprung mass.

corner displacement (sprung side):
```
z_corner_i  =  z_s  −  rx_i · θ
v_corner_i  =  ż_s  −  rx_i · θ̇
```

(roll φ 는 별도로 axle-level roll stiffness 처리 — 아래 §6.4 참조.)

spring + damper force on sprung corner (upward positive):
```
δ_i  =  z_corner_i  −  z_u_i           (compression deviation from static)
v_i  =  v_corner_i  −  ż_u_i
F_i  =  − k_i · δ_i  −  c_i · v_i      (spring + damper)
```

Sprung body EoM:
```
m_s · z̈_s   =  Σ F_i
Iyy · θ̈   =  − Σ rx_i · F_i  +  m_s · ax · h_cg · (1 − anti)
Ixx · φ̈   =  − K_phi · φ  −  C_phi · φ̇  +  m_s · ay · h_cg
```

코드 `core/src/fourteen_dof_dynamics.cpp:128-156`.

### Heave EoM (z_s)

가장 단순. spring 합력이 sprung mass 의 vertical 가속도 만든다.

### Pitch EoM (θ)

```
Iyy · θ̈  =  − Σ rx_i · F_i  +  m_s · ax · h_cg · (1 − anti_dive)
```

- `−Σ rx_i · F_i` 는 spring/damper 의 pitch moment. front spring 이 더 압축되면 nose down.
- `m_s · ax · h_cg` 는 inertia 의 pitch contribution. 가속 시 nose up, 제동 시 nose down.
- `(1 − anti_dive)` 가 suspension geometry 의 anti-dive 효과 — bypass 일부 inertia moment.

### Anti-dive / anti-squat

real suspension 은 brake 시 brake reaction force 가 일부 suspension link 를 통해 chassis 로 직접 전달 (suspension 의 instant center 이용). 그 만큼 pitch moment 가 감소.

VDSim 의 단순 모델:
```
anti  =  ax < 0 ? anti_dive_front : anti_squat_rear  (clamped [0, 1])
M_inertia_pitch  =  m_s · ax · h_cg · (1 − anti)
```

`anti = 1` 이면 brake 시 pitch 가 발생 안 함. `0` 이면 full pitch.

### Roll EoM (φ)

```
Ixx · φ̈  =  − K_phi_total · φ  −  C_phi · φ̇  +  m_s · ay · h_cg
```

- `K_phi_total = roll_stiffness_front + roll_stiffness_rear` (axle-level roll stiffness, springs + ARB combined).
- `C_phi` 는 axle 의 damper 의 roll-equivalent stiffness: `Σ (c_corner · arm²)`.

quasi-static SS:
```
φ_ss  =  m_s · ay · h_cg / K_phi_total
```

이게 Task 22 (`Roll/pitch state diagnostics`) 의 quasi-static estimator. Ld3 의 dynamic 적분이 SS 에서 위와 일치.

### Why heave/pitch via spring corner, roll via axle K_phi?

heave 와 pitch 는 per-corner spring 의 linear sum 으로 자연스럽게 표현. 그러나 roll 의 경우 ARB (anti-roll bar) 가 추가 contribution 을 줘서 sum 이 단순 spring·arm² 보다 큼. 따라서 axle-level `K_phi` 를 별도로 받아 통합 — 이게 textbook 표준 lumped 모델.

만약 spring·arm² 만 쓰면 ARB 효과 누락 → roll 이 실측보다 작게 나옴. 본 PoC 의 sedan default 의 경우 `K_phi = 30 + 25 = 55 kN·m/rad`, 반면 spring·arm²·sum = `30000 · 2·(0.775)² ≈ 36 kN·m/rad`. ARB 가 약 35 % 추가 기여.

## 6.4 Unsprung mass EoM

per-corner unsprung mass 의 vertical 동역학:
```
m_u_i · z̈_u_i  =  − F_susp_on_sprung_i  −  k_tire · z_u_i
                =  + k_i · δ_i + c_i · v_i  −  k_tire · z_u_i
```

(Newton III: spring/damper 가 sprung 위로 미는 force 의 반대 = unsprung 아래로 미는 force.)

`k_tire` 는 TireParams 의 `tire_vertical_stiffness` (default 220 kN/m).

코드 `core/src/fourteen_dof_dynamics.cpp:155-160`:
```cpp
const double k_tire = std::max(1.0, tp_.tire_vertical_stiffness);
for (int i = 0; i < NUM_WHEELS; ++i) {
    const double m_u = std::max(1.0, vp_.unsprung_mass[i]);
    d.dz_u[i]     = zu_dot[i];
    d.dz_u_dot[i] = (- F_susp[i] - k_tire * zu[i]) / m_u;
}
```

### Tire vertical stiffness 의 의미

tire 가 vertical 방향으로 spring 처럼 작용. radial deformation 이 `Fz / k_tire`. typical 200-300 kN/m for passenger tires.

이게 wheel hop frequency (sprung 과 분리된 unsprung 의 vertical resonance) 의 dominant stiffness:
```
ω_hop  =  sqrt(k_tire / m_u)
f_hop  =  ω_hop / (2π)
```

sedan default: `sqrt(220000 / 40) / (2π) ≈ 11.8 Hz`.

### Damping ratio 문제 (Task 51 의 분석)

unsprung mass 의 critical damping:
```
ζ_u  =  c_corner / (2 · sqrt(k_spring · m_u))
```

sedan default: `c = 3000, k = 30000, m_u = 40` → `ζ_u = 3000 / (2·sqrt(1.2e6)) = 1.37`. **overdamped**.

실차의 corner damper 는 sprung body resonance (~1-2 Hz) 에 fit 되어 있어 ζ ~ 0.3-0.5. 그 동일 c 를 unsprung 에 쓰면 wheel-hop 영역 (10+ Hz) 에서 너무 stiff.

본 PoC 는 corner damper 한 값만 (sprung + unsprung 분리 안 됨). wheel-hop FFT (Task 51) 에서 11.8 Hz peak 가 명확히 안 보임 (5 Hz coupled mode 만 보임).
실제 차량 simulator (Adams Car 등) 는 sprung damper 와 unsprung damper 를 분리. 본 PoC follow-up: `corner_damper` 를 `damper_sprung` / `damper_unsprung` 로 분할.

## 6.5 RK4 적분 — 14 vertical DOF

State vector (planar 7 은 inner Ld2 가 처리):
```
y = [z_s, ż_s, φ, φ̇, θ, θ̇, z_u_FL, ż_u_FL, z_u_FR, ż_u_FR, z_u_RL, ż_u_RL, z_u_RR, ż_u_RR]
```

14 first-order ODE.

derivative `f(y, t)`:
```
ẏ = [ż_s, z̈_s, φ̇, φ̈, θ̇, θ̈, ż_u_FL, z̈_u_FL, ..., ż_u_RR, z̈_u_RR]
```

각 second-order 가 위에서 본 식대로 계산.

`integrate_vertical()` (코드 `core/src/fourteen_dof_dynamics.cpp:182-238`) 가 RK4:
```
k1  =  f(y, t)
k2  =  f(y + h/2 · k1, t + h/2)
k3  =  f(y + h/2 · k2, t + h/2)
k4  =  f(y + h · k3, t + h)

y_{n+1}  =  y_n + h/6 · (k1 + 2k2 + 2k3 + k4)
```

substep dt = 1 ms (sp_.max_substep_dt). outer dt = 5 ms 면 5 substeps per outer.

`ax`, `ay` 의 1-step lag — outer step 의 input (Ld2 의 직전 값) 으로 vertical 적분 동안 constant. 더 정확하게는 매 substep 마다 update 해야 하지만 본 PoC 는 outer-step lag.

## 6.6 Pose 의 quat encoding

```cpp
state_.orientation = quat_from_euler({phi_, th_, yaw});
```

yaw 는 planar Ld2 가 이미 적분. roll φ, pitch θ 는 Ld3 의 적분 결과.
ZYX intrinsic Euler 로 quat 구성.

이렇게 quat 에 RPY 모두 encode 하면 외부 시각화 (CARLA, viewer) 가 차체 자세 그대로 표현 가능. Ld1-Ld2 는 yaw 만 encode (RPY 의 0, 0, yaw).

## 6.7 Susp_compression / susp_velocity 의 의미

```cpp
state_.susp_compression[i]  =  static_compression_[i] + δ_i
state_.susp_velocity[i]     =  v_corner_i − ż_u_i
```

`static_compression_[i] = Fz_static_i / k_i` — sprung 의 정적 weight 로 인한 spring 압축.
`δ_i` 는 동적 deviation (sprung corner z − unsprung z).

총 susp_compression > static_compression → spring 이 압축됨.
< static_compression → spring 이 신장됨 (떨림 또는 wheel 들림).

ride 분석에서 이 두 값이 핵심 진단.

## 6.8 Ld2 ↔ Ld3 의 거동 차이

quasi-static 의 SS yaw rate / Fz 는 Ld2 와 Ld3 가 일치 (분석값 동일). 차이는:
- **Transient**: Ld3 가 roll overshoot, settle 시간, wheel-hop oscillation 표현 가능.
- **dynamic Fz**: Ld3 의 Fz 가 매 step 마다 spring 의 transient 반영. Ld2 는 quasi-static.

검증 (Task 30 FourteenDOF dynamic):
- step steer (sports, vx=10, δ=0.05): Ld3 의 roll oscillation, settle ~ 1.5 s.
- DLC: roll range ±4° (sport 거동).
- brake step: pitch transient + nose dive.

`PlanarMotionMatchesL2Closely` test 는 Ld3 의 planar (vx, vy, r) 가 Ld2 와 bit-equal 임을 보장 — Ld3 가 Ld2 위에 vertical 만 추가했음을 검증.

## 6.9 한계와 follow-up

| 항목 | 본 PoC | follow-up |
|---|---|---|
| Sprung 6-DOF integration | small-angle linearized | full nonlinear (Featherstone) |
| Corner damper 분리 | 단일 c | sprung / unsprung 분리 |
| Anti-dive geometry | scalar factor | instant-center 기반 정확 모델 |
| Suspension link friction | 무시 | bushing nonlinearity |
| Wheel camber from roll | `γ = 0` (Pacejka API 만) | hardpoint → wheel kinematics |
| Tire transient (relaxation) | 무시 | 1st-order tire dynamics |
| Lateral load transfer 동적 | 1-step lag | self-consistent 통합 |

Ld4-Ld5 가 본격 multibody 로 위 한계 대부분 해소.

## 6.10 사용 패턴

```cpp
auto dyn = vdsim::create_fourteen_dof();
dyn->initialize(vp, tp, sp);
// ... reset, step (Ld1 / Ld2 와 동일 API)

dyn->roll_angle_qs();    // dynamic roll (rad)
dyn->pitch_angle_qs();   // dynamic pitch
dyn->state().susp_compression[WHEEL_FL];   // FL spring 압축 (m)
dyn->state().susp_velocity[WHEEL_FL];      // FL ride velocity (m/s)
```

`ax_body_est()`, `ay_body_est()` 도 Ld2 와 동일하게 사용 가능. Ld3 는 inner Ld2 의 값을 전달.

## 6.11 검증

`FourteenDOF.*` (Task 24, 30, 41 통합):
- `ConstructionAndLevel`
- `StaticSuspensionPopulatedAtRest` — `susp_compression = static_compression` at rest.
- `CompressionGrowsUnderBrakeOnFront`
- `PlanarMotionMatchesL2Closely` — Ld3 planar 의 vx, vy, r 가 Ld2 와 1e-9 일치.
- `PoseEncodesRollAndPitch` — quat 의 roll 추출이 `roll_angle_qs` 와 0.02 rad 이내.
- `RollOscillatesAndSettles` — step steer 의 dynamic overshoot.
- `PitchTransientUnderBrake`
- `SuspensionVelocityNonZeroDuringTransient`
- `AntiDiveReducesPitchUnderBrake` — anti_dive 0 vs 0.5 비교, anti-dive 가 pitch 감소.

총 9 tests pass.

## 6.12 참고

- Genta, *Motor Vehicle Dynamics*, §7 (lumped 14-DOF).
- Milliken, *Race Car Vehicle Dynamics*, §17 (anti-dive / anti-squat).
- Reimpell, *The Automotive Chassis*, §6 (suspension kinematics).
