# 03. 타이어 모델 — Pacejka Magic Formula 1996

> **학습 목표.** Magic Formula 의 `D · sin(C · atan(B · s − E · (B·s − atan(B·s))))` 가 왜 그런 모양인지, 각 계수 B/C/D/E 의 물리적 역할이 무엇인지, combined slip 의 friction-ellipse rescale 이 어떤 가정 위에 있는지, aligning moment Mz 와 pneumatic trail 의 의미가 무엇인지 — 모두 독립적으로 설명할 수 있다. PoC 가 단순한 MF96 simple form 만 쓰는 이유와 MF2002 까지 가지 않은 trade-off 를 명확히 안다.

## 3.1 타이어가 차량 동역학의 비선형 핵심인 이유

차량 동역학은 본질적으로 **타이어 한계 동역학** 이다. 정상적 운전 조건 (linear region) 에서는 거의 모든 차량이 비슷하게 거동한다. cornering 한계, brake 한계, traction 한계 — 모두 타이어의 비선형 saturation 에서 결정된다.

vehicle dynamics 의 sensitivity 를 따져보면, mass / wheelbase / aero 보다 **tire 의 (D, μ, Cα)** 에 훨씬 강하게 의존. 그래서 Pacejka 가 타이어 modeling 에 한 평생을 쓴 것이다.

VDSim 은 Pacejka MF96 의 **simple form (semi-empirical, simplified)** 을 채택. MF2002 의 full form (combined slip 의 자체 식, Mzr 잔류 모멘트, camber-Mz 결합 등) 은 Phase 2.

## 3.2 Magic Formula 의 모양

기본형:
```
F(s) = D · sin( C · atan( B · s − E · (B · s − atan(B · s)) ) )
```

- `s` — 입력 (lateral 의 경우 slip angle α, longitudinal 의 경우 slip ratio κ).
- `D` — 피크 force amplitude. `D = D_param · Fz · μ`.
- `B` — stiffness factor. linear-region slope 결정. `slope_0 = B · C · D`.
- `C` — shape factor. 곡선의 폭 / 비선형 정도.
- `E` — curvature factor. 피크 주변의 형상.

### 왜 sin(atan(...))?

`atan(B·s)` 는 `s → ∞` 에서 `π/2` 로 saturate. `sin(C · π/2)` 가 1 이면 peak 위치 결정.
`sin(C · atan(...))` 형태로 합치면 (1) 작은 s 에서 linear, (2) saturate 후 자연스러운 descent, (3) `D` 가 peak.

직관: tire 의 force-slip 곡선은 작은 slip 에서 linear, 중간에서 peak, 큰 slip 에서 sliding 영역으로 감소. `sin(atan)` 이 이 모양을 4 개 parameter 로 표현하는 가장 간결한 함수 family.

### E 항의 역할

`B·s − E · (B·s − atan(B·s))` 의 second term `(B·s − atan(B·s))` 는 `s > 0` 영역에서 음수 (atan 이 항상 더 작음). E < 1 이면 boost, E > 1 이면 over-saturation.
VDSim default:
```
B_long = 10.0,  C_long = 1.65,  D_long = 1.0,  E_long = +0.97
B_lat  =  8.0,  C_lat  = 1.30,  D_lat  = 1.0,  E_lat  = -1.00
```

`E_lat = −1` 가 음수인 이유: lateral 곡선이 peak 후 더 sharp 하게 떨어지도록.

## 3.3 부호 약속 (lateral)

ISO 8855 RH 에서 lateral 식은:
```
Fy(α) = − D · sin( C · atan( B·α − E · (B·α − atan(B·α)) ) )
```

leading minus. 이유:
- α > 0 (위치상 wheel 의 velocity 가 +y_wheel 쪽으로 기울어진 상황) → tire 가 restoring force 를 −y 방향 으로 생성 → `Fy < 0`.
- VDSim 이 `Fy = −Dy · sin(...)` 로 구현하면 부호 자동.

코드 `core/src/pacejka_mf96.cpp:42-48`:
```cpp
{
    const double s   = in.alpha;
    const double t   = tp_.B_lat * s;
    const double phi = t - tp_.E_lat * (t - std::atan(t));
    out.Fy = -Dy * std::sin(tp_.C_lat * std::atan(phi));
}
```

## 3.4 Combined slip — friction ellipse rescale

### 문제

decoupled `Fx(κ)` and `Fy(α)` 가 동시에 비-zero 일 때 `(Fx, Fy)` 의 합력이 `μ · Fz` 의 friction circle 을 넘어갈 수 있다. 실제 tire 는 그게 불가능하므로 어떤 형태로든 cap 이 필요하다.

### 해법 — friction-ellipse rescale (VDSim 채택)

각 축의 peak: `Fx_max = D_long · Fz · μ_long`, `Fy_max = D_lat · Fz · μ_lat`.

```
r² = (Fx_pure / Fx_max)² + (Fy_pure / Fy_max)²
if r² > 1:
    scale = 1 / sqrt(r²)
    Fx, Fy *= scale
else:
    Fx = Fx_pure, Fy = Fy_pure
```

ellipse (or circle if `Fx_max = Fy_max`) 내부면 그대로, 외부면 가장 가까운 ellipse boundary 로 끌어당김.

### 왜 단순 rescale 이 OK 인가

pure-slip 케이스 (`κ = 0` or `α = 0`) 에서는 `Fy_pure = 0` or `Fx_pure = 0`. `r ≤ 1` 이 항상 보장 → rescale 발생 안 함. 즉 본 PoC 의 단순 rescale 은 모든 pure-slip 테스트 (Task 11/13 의 8 unit + 4 integration test) 를 깨지 않는다.

검증: `core/tests/unit/test_tire_models.cpp:170-186` 의 `FrictionEllipseBound` 가 1024 점 격자에서 `(Fx/Fx_max)² + (Fy/Fy_max)²  ≤ 1 + 1e-9` 통과.

### MF2002 의 더 정확한 모델

MF2002 는 normalized slip `σ_x = κ / (1 + κ)`, `σ_y = tan(α) / (1 + κ)` 으로 통합 후 한 식으로 푼다. 이게 더 정확하지만 (1) `κ = -1` 의 brake-locked 발산 처리 필요, (2) 파라미터 5 개 더, (3) implementation complexity. PoC 는 friction-ellipse rescale 로 충분.

## 3.5 Aligning moment Mz

### 정의

타이어가 받는 lateral force 의 application point 가 contact patch 중심에서 **t_p (pneumatic trail)** 만큼 뒤에 있다. 따라서:

```
Mz_wheel = − t_p(α) · Fy
```

- `Mz > 0` (위에서 본 CCW) when `α > 0` (Fy < 0). 즉 self-aligning — wheel 을 α 가 줄어드는 방향으로 회전시킨다.

### t_p 의 falloff

low α 영역에서는 `t_p ≈ t_p_0` (constant, 보통 50 mm).
α 가 커지면 contact patch 의 pressure distribution 이 앞쪽으로 이동 → t_p 감소.

VDSim 모델:
```
t_p(α) = t_p_0 / sqrt(1 + (α / α_falloff)²)
```

이건 `cos(atan(α/α_fo))` 와 등가. 단순하지만 textbook 의 일반 trend (`α` 작을 때 거의 일정, 큰 α 에서 단조 감소) 잘 표현.

VDSim default:
```
t_p_0 = 0.05 m,  α_falloff = 0.20 rad (≈ 11.5°)
```

### 차량 yaw moment 에 미치는 영향

Mz 가 단순히 tire 가 받는 self-aligning torque 가 아니라, body 의 yaw moment 에 더해진다 (steered wheel 이 받는 torque 가 steering 시스템 통해 chassis 로 전달).

VDSim 의 Mz aggregation:
```
Mz_body = Σ (rx_i · Fy_body_i − ry_i · Fx_body_i)   (force translation)
        + Σ Mz_wheel_i                                (per-tire intrinsic Mz)
```

코드 `core/src/seven_dof_dynamics.cpp:336-340`:
```cpp
for (int i = 0; i < NUM_WHEELS; ++i) {
    Mz_total += r_x[i] * F_body[i].y() - r_y[i] * F_body[i].x();
}
for (int i = 0; i < NUM_WHEELS; ++i) Mz_total += mz_wheel[i];
```

이게 step_steer 의 SS yaw rate 를 약 2.6 % 감소시킨다 (Task 23 검증). analytical linear-bicycle 은 Mz 무시하므로 sim vs analytical 차이 6.8 % 정도. 그게 implementation bug 가 아니라 model-mismatch 임을 명시.

## 3.6 Camber thrust (linear, API only)

### 정의

wheel 이 vertical 에서 γ 만큼 기울었을 때 추가 lateral force 가 생성된다.

```
Fy_camber = − C_γ · γ · Fz · μ_lat
```

VDSim 의 `TireParams::camber_stiffness` default `0` — 즉 disable. enable 시 위 식이 `Fy_lat` 에 가산.

### 왜 default 0

본 PoC 에서 Ld2-Ld3 의 wheel kinematics 에서 `γ` 가 자동 계산되지 않는다 (Ld2 는 wheel 이 항상 vertical 가정, Ld3 는 roll 이 quaternion 에 있지만 wheel-level γ 로 mapping 안 됨). 따라서 `γ = 0` 으로 호출되어도 식이 들어와도 효과 없음 — backward-compat.

camber 효과를 보고 싶으면 (a) `camber_stiffness > 0` 설정, (b) `in.gamma` 를 외부에서 주입. 실제 wheel kinematics 통합은 Ld4 의 forward kinematics 가 들어와야 가능.

## 3.7 Fz 의 mu scaling

```
Fx = (D_long · Fz · μ_long) · sin(C_long · atan(...))
Fy = − (D_lat · Fz · μ_lat) · sin(C_lat · atan(...))
```

`μ_long`, `μ_lat` 는 `ContactPoint` 에서 받는 surface mu (per wheel). 노면 변화 (icy patch 등) 표현.
`Fz_nominal` 은 reference (4000 N for sedan default), `D_param = 1.0` 이면 peak force = `Fz · μ`. 따라서 default 에서 `peak μ = 1.0`.

## 3.8 가정 / 한계 (정리)

| 항목 | 본 모델 | 한계 / Phase 2 항목 |
|---|---|---|
| Combined slip | friction-ellipse rescale | MF2002 의 σ_x, σ_y 통합 |
| Aligning moment | `−t_p(α) · Fy` | MF2002 의 Mzr 잔류 + camber-Mz 결합 |
| Camber | linear `Fy_camber = −C_γ · γ · Fz` | non-linear (peak μ 의 camber 의존) |
| Transient | quasi-static (relaxation length 무시) | 1st-order tire relaxation (Phase 2) |
| Temperature | 무시 (constant μ) | tire temperature evolution (Phase 2) |
| Fz peak shift | constant D | Fz-dependent D / peak shift (Magic Formula extension) |
| Camber-Fx | 무시 | 일부 race tire 에서 의미 |

## 3.9 검증

`test_tire_models.cpp` 의 검증:
- `ZeroSlipZeroForce`, `ZeroFzZeroForce` — 기본 boundary.
- `LinearRegionLateralSlope`, `LinearRegionLongitudinalSlope` — α=1e-4 에서 `Fy/α ≈ −B·C·D·Fz·μ` 의 ±1 %.
- `SignConventions` — α > 0 → Fy < 0, κ > 0 → Fx > 0, etc.
- `PeakBoundedByFzMu` — sweep `[-0.5, 0.5]` 에서 `|Fy| ≤ Fz · μ · D_lat · 1.001`.
- `MuScalesLinearly` — linear region 에서 mu 스케일 정확.
- `FzScalesLinearlyInLinearRegion` — Fz 절반 → Fy 절반.

`PacejkaCombinedFixture`:
- `FrictionEllipseBound` (Task 15) — 1024 점에서 `(Fx/Fx_max)² + (Fy/Fy_max)² ≤ 1`.
- `PureSlipUnchangedByCombinedFlag` — pure slip 케이스 backward-compat.
- `MzZeroWhenAlphaZero`, `MzSignOppositeFy`.
- `MzLinearRegionMatchesPneumaticTrail` — α=1e-4 에서 `Mz ≈ −t_p · Fy`.
- `MzDecreasesAtLargeAlpha` — `|Mz/Fy|` 가 small α 보다 large α 에서 작음.
- `CamberAddsLateralForce`, `CamberZeroByDefault`.

전체 unit-test 가 통과하면 `ITireModel` 의 contract 가 확정.

## 3.10 사용 패턴 (in 차량 EoM)

```cpp
ITireModel::Input in;
in.Fz       = Fz_axle / 2;       // per tire
in.kappa    = (R * omega - vx_wheel) / max(|vx_wheel|, 0.5);
in.alpha    = atan2(vy_wheel, vx_wheel);
in.mu_long  = contact.mu_long;
in.mu_lat   = contact.mu_lat;
in.Vx_wheel = vx_wheel;
in.gamma    = 0.0;               // wheel camber (Ld2-Ld3 에서 0, Ld4 에서 계산)

const auto F = tire_->compute(in);
// F.Fx, F.Fy in wheel frame.  Rotate back to body frame if wheel was steered.
```

이 호출이 차량 EoM 의 모든 lateral / longitudinal force 의 source.

## 3.11 참고

- Pacejka, *Tire and Vehicle Dynamics*, §4 (Magic Formula derivation), §6 (Combined slip), §7 (Mz, Mzr).
- Genta, *Motor Vehicle Dynamics*, §3.4-3.6 (tire models survey).
- Milliken & Milliken, *Race Car Vehicle Dynamics*, §2 (tire force fundamentals, mostly empirical).
