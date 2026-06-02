# 03. 타이어 모델 — Pacejka Magic Formula 1996

> **학습 목표.** Magic Formula 의 `D · sin(C · atan(B · s − E · (B·s − atan(B·s))))` 가 왜 그런 모양인지, 각 계수 B/C/D/E 의 물리적 역할이 무엇인지, combined slip 의 friction-ellipse rescale 이 어떤 가정 위에 있는지, aligning moment Mz 와 pneumatic trail 의 의미가 무엇인지 — 모두 독립적으로 설명할 수 있다. 여기에 더해 **load sensitivity** (μ 가 Fz 에 따라 감소), **relaxation length** (slip 의 1차 지연), **camber thrust + camber Mz** 의 세 가지 확장이 무엇을 모델링하고 왜 추가됐는지 안다. PoC 가 단순한 MF96 simple form 을 베이스로 쓰는 이유와 MF2002 까지 가지 않은 trade-off 를 명확히 안다.

## 3.1 타이어가 차량 동역학의 비선형 핵심인 이유

차량 동역학은 본질적으로 **타이어 한계 동역학** 이다. 정상적 운전 조건 (linear region) 에서는 거의 모든 차량이 비슷하게 거동한다. cornering 한계, brake 한계, traction 한계 — 모두 타이어의 비선형 saturation 에서 결정된다.

vehicle dynamics 의 sensitivity 를 따져보면, mass / wheelbase / aero 보다 **tire 의 (D, μ, Cα)** 에 훨씬 강하게 의존. 그래서 Pacejka 가 타이어 modeling 에 한 평생을 쓴 것이다.

VDSim 은 Pacejka MF96 의 **simple form (semi-empirical, simplified)** 을 채택. MF2002 의 full form (combined slip 의 자체 식, Mzr 잔류 모멘트, camber-Mz 결합 등) 은 Phase 2.

## 3.2 Magic Formula 의 모양

기본형:

$$
F(s) = D \sin\!\Big( C \arctan\big( B s - E (B s - \arctan(B s)) \big) \Big)
$$

- $s$ — 입력 (lateral 의 경우 slip angle $\alpha$, longitudinal 의 경우 slip ratio $\kappa$).
- $D$ — 피크 force amplitude. $D = D_{\text{param}} \cdot F_z \cdot \mu$.
- $B$ — stiffness factor. linear-region slope 결정. $\text{slope}_0 = B C D$.
- $C$ — shape factor. 곡선의 폭 / 비선형 정도.
- $E$ — curvature factor. 피크 주변의 형상.

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

$$
F_y(\alpha) = - D \sin\!\Big( C \arctan\big( B\alpha - E (B\alpha - \arctan(B\alpha)) \big) \Big)
$$

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

각 축의 peak: $F_{x,\max} = D_{\text{long}} F_z \mu_{\text{long}}$, $F_{y,\max} = D_{\text{lat}} F_z \mu_{\text{lat}}$.

$$
r^2 = \left(\frac{F_{x,\text{pure}}}{F_{x,\max}}\right)^2 + \left(\frac{F_{y,\text{pure}}}{F_{y,\max}}\right)^2
$$

$$
(F_x, F_y) = \begin{cases}
(F_{x,\text{pure}}, F_{y,\text{pure}}) & r^2 \le 1 \\[4pt]
\dfrac{1}{r}\,(F_{x,\text{pure}}, F_{y,\text{pure}}) & r^2 > 1
\end{cases}
$$

ellipse (or circle if `Fx_max = Fy_max`) 내부면 그대로, 외부면 가장 가까운 ellipse boundary 로 끌어당김.

### 왜 단순 rescale 이 OK 인가

pure-slip 케이스 (`κ = 0` or `α = 0`) 에서는 `Fy_pure = 0` or `Fx_pure = 0`. `r ≤ 1` 이 항상 보장 → rescale 발생 안 함. 즉 본 PoC 의 단순 rescale 은 모든 pure-slip 테스트 (Task 11/13 의 8 unit + 4 integration test) 를 깨지 않는다.

검증: `core/tests/unit/test_tire_models.cpp:170-186` 의 `FrictionEllipseBound` 가 1024 점 격자에서 `(Fx/Fx_max)² + (Fy/Fy_max)²  ≤ 1 + 1e-9` 통과.

### MF2002 의 더 정확한 모델

MF2002 는 normalized slip `σ_x = κ / (1 + κ)`, `σ_y = tan(α) / (1 + κ)` 으로 통합 후 한 식으로 푼다. 이게 더 정확하지만 (1) `κ = -1` 의 brake-locked 발산 처리 필요, (2) 파라미터 5 개 더, (3) implementation complexity. PoC 는 friction-ellipse rescale 로 충분.

## 3.5 Aligning moment Mz

### 정의

타이어가 받는 lateral force 의 application point 가 contact patch 중심에서 **$t_p$ (pneumatic trail)** 만큼 뒤에 있다. 따라서:

$$
M_{z,\text{wheel}} = - t_p(\alpha) \cdot F_y
$$

- $M_z > 0$ (위에서 본 CCW) when $\alpha > 0$ ($F_y < 0$). 즉 self-aligning — wheel 을 $\alpha$ 가 줄어드는 방향으로 회전시킨다.

### $t_p$ 의 falloff

low $\alpha$ 영역에서는 $t_p \approx t_{p,0}$ (constant, 보통 50 mm).
$\alpha$ 가 커지면 contact patch 의 pressure distribution 이 앞쪽으로 이동 → $t_p$ 감소.

VDSim 모델:

$$
t_p(\alpha) = \frac{t_{p,0}}{\sqrt{1 + (\alpha / \alpha_{\text{falloff}})^2}}
$$

이건 $\cos(\arctan(\alpha/\alpha_{\text{fo}}))$ 와 등가. 단순하지만 textbook 의 일반 trend ($\alpha$ 작을 때 거의 일정, 큰 $\alpha$ 에서 단조 감소) 잘 표현.

VDSim default:
```
t_p_0 = 0.05 m,  α_falloff = 0.20 rad (≈ 11.5°)
```

### 차량 yaw moment 에 미치는 영향

Mz 가 단순히 tire 가 받는 self-aligning torque 가 아니라, body 의 yaw moment 에 더해진다 (steered wheel 이 받는 torque 가 steering 시스템 통해 chassis 로 전달).

VDSim 의 Mz aggregation:

$$
M_{z,\text{body}} = \underbrace{\sum_i \big(r_{x,i} F_{y,\text{body},i} - r_{y,i} F_{x,\text{body},i}\big)}_{\text{force translation}} + \underbrace{\sum_i M_{z,\text{wheel},i}}_{\text{per-tire intrinsic } M_z}
$$

코드 `core/src/seven_dof_dynamics.cpp:336-340`:
```cpp
for (int i = 0; i < NUM_WHEELS; ++i) {
    Mz_total += r_x[i] * F_body[i].y() - r_y[i] * F_body[i].x();
}
for (int i = 0; i < NUM_WHEELS; ++i) Mz_total += mz_wheel[i];
```

이게 step_steer 의 SS yaw rate 를 약 2.6 % 감소시킨다 (Task 23 검증). analytical linear-bicycle 은 Mz 무시하므로 sim vs analytical 차이 6.8 % 정도. 그게 implementation bug 가 아니라 model-mismatch 임을 명시.

## 3.6 Camber thrust + camber Mz

### 정의

wheel 이 vertical 에서 $\gamma$ 만큼 기울었을 때 추가 lateral force 가 생성된다.

$$
F_{y,\text{camber}} = - C_\gamma \cdot \gamma \cdot F_z \cdot \mu_{\text{lat}}
$$

`TireParams::camber_stiffness` ($C_\gamma$) default `0`. enable 시 위 식이 $F_{y,\text{lat}}$ 에 가산.

### camber 가 Mz 에도 기여

camber thrust 의 application point 도 contact patch 중심에서 약간 벗어나 작은
aligning moment 를 만든다. VDSim 의 추가 (`pacejka_mf96.cpp`):

$$
M_{z,\text{camber}} = - t_{p,0} \cdot 0.25 \cdot C_\gamma \cdot \gamma \cdot F_z \cdot \mu
$$

$$
M_{z,\text{total}} = - t_p \cdot F_y + M_{z,\text{camber}}
$$

$0.25 \cdot t_{p,0}$ 은 camber arm 의 근사. $\gamma$ 에 anti-symmetric — $\pm\gamma$ 가 $\mp M_z$
(test `CamberContributesToMz` 검증).

### γ 가 이제 자동 계산됨 (Ld4 연결)

이전 PoC 에서는 Ld2-Ld3 가 wheel-level γ 를 계산하지 못해 항상 `γ=0` 이었다.
Ld4 hardpoint kinematics (Chapter 14) 가 들어오면서 바뀜:

1. Ld3 가 매 substep per-wheel travel → `ISuspensionKinematics::compute` →
   camber γ 를 얻음.
2. `inner_->set_camber_per_wheel(γ)` 로 Ld2 에 전달.
3. Ld2 가 Pacejka `in.gamma = camber_ext_[i]` 로 호출.

hardpoint kinematics 가 attach 안 됐으면 legacy fallback (`camber_per_roll · φ`)
또는 `γ=0`. 즉 backward-compat 유지하면서 geometry-driven camber 가능.

연결 경로: `fourteen_dof_dynamics.cpp` → `seven_dof_dynamics.cpp` →
`pacejka_mf96.cpp`. 자세히는 Chapter 14.8.

## 3.7 Fz 의 mu scaling + load sensitivity

$$
\begin{aligned}
F_x &= (D_{\text{long}} F_z \mu_{\text{eff,long}}) \sin(C_{\text{long}} \arctan(\cdots)) \\
F_y &= -(D_{\text{lat}}  F_z \mu_{\text{eff,lat}})  \sin(C_{\text{lat}}  \arctan(\cdots))
\end{aligned}
$$

$\mu_{\text{long}}$, $\mu_{\text{lat}}$ 는 `ContactPoint` 에서 받는 surface mu (per wheel). 노면 변화 (icy patch 등) 표현.

### Load sensitivity — μ 가 Fz 에 따라 감소

실제 타이어는 수직 하중이 클수록 *단위 하중당 grip* 이 감소한다 (rubber 의
load sensitivity). VDSim 의 `μ_eff`:

$$
\mu_{\text{eff}}(F_z) = \mu_{\text{nominal}} \cdot \big(1 - k_{\text{load}} \cdot (F_z/F_{z,\text{nominal}} - 1)\big)
$$

(단, $0.3\,\mu_{\text{nominal}}$ 로 floor — 극한 $F_z$ 에서 수치 안정.
$k_{\text{load}}$ = `load_sensitivity`.)

- `Fz = Fz_nominal` 이면 `μ_eff = μ_nominal`.
- `Fz > Fz_nominal`: grip 감소 → 코너링 시 외측 휠이 안쪽보다 *상대적으로
  덜* 받쳐줌 → 자연스러운 load transfer 효과의 일부.
- `load_sensitivity = 0`: legacy (Fz 에 비례하는 peak).

`default_pacejka.yaml`: `load_sensitivity: 0.15`, `Fz_nominal: 4000`.
검증 `LoadSensitivityFadeAtHighFz`: 2× Fz 에서 Fy ratio 가 1.6 (선형이면 2.0).

## 3.7b Relaxation length — slip 의 1차 지연 (transient)

이전 PoC 는 quasi-static — slip 이 바뀌면 force 가 즉시 따라갔다. 실제 타이어는
carcass 변형 때문에 **rolling distance σ** 만큼 지연된다 (relaxation length).

transient slip angle $\alpha_{\text{dyn}}$ 이 geometric slip $\alpha_{\text{geom}}$ 을 1차 시스템으로 추종:

$$
\frac{\sigma}{|v_{\text{long}}|}\,\dot\alpha_{\text{dyn}} = \alpha_{\text{geom}} - \alpha_{\text{dyn}}
$$

force 는 $\alpha_{\text{dyn}}$ 으로 계산 (instantaneous $\alpha$ 가 아니라). 닫힌형 적분 (substep 사이):

$$
\alpha_{\text{dyn}}(t+h) = \alpha_{\text{geom}} + \big(\alpha_{\text{dyn}}(t) - \alpha_{\text{geom}}\big)\, e^{-|v|\,h/\sigma}
$$

구현 위치: tire model 이 아니라 **host dynamics** 의 state (`seven_dof`,
`bicycle` 의 `alpha_dyn_[4]`). tire `compute()` 는 stateless 유지.

효과: step steer 시 Fy 가 약 `σ/v` 시간상수로 build-up. `default_pacejka.yaml`
`relaxation_length_lat: 0.6` → 15 m/s 에서 시상수 ~40 ms. 검증
`RelaxationLengthDelaysLateralForce`: t=σ/v 시점 Fy 가 instant 대비 < 85%.

`Fz_nominal` 은 reference (4000 N for sedan default), `D_param = 1.0` 이면 peak force = `Fz · μ_eff`.

## 3.8 가정 / 한계 (정리)

| 항목 | 본 모델 | 한계 / Phase 2 항목 |
|---|---|---|
| Combined slip | friction-ellipse rescale | MF2002 의 σ_x, σ_y 통합 |
| Aligning moment | `−t_p(α) · Fy + Mz_camber` | MF2002 의 Mzr 잔류 모멘트 |
| Camber | linear `Fy_camber` + linear camber Mz | non-linear (peak μ 의 camber 의존) |
| Load sensitivity | linear `μ_eff(Fz)` ✅ 구현 | non-linear load curve |
| Transient | 1st-order relaxation length (lateral) ✅ 구현 | longitudinal relaxation, carcass 동역학 |
| γ 입력 | Ld4 hardpoint kinematics 가 자동 계산 ✅ | force→camber 역방향 compliance |
| Temperature | 무시 (constant μ) | tire temperature evolution (Phase 2) |
| Fz peak shift | constant D (μ_eff 만 Fz 의존) | Fz-dependent B/C/E (Magic Formula extension) |
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
