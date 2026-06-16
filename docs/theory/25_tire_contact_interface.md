# 25. Tire Contact — Effective Rolling Radius, Camber Migration & the Inverted Interface

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. Load-dependent effective rolling radius `Re(Fz)` 의 정의와, free-rolling loaded
   tire 가 `kappa = 0` 을 보고해야 하는 이유를 설명한다.
2. Camber contact-point migration 이 왜 overturning moment `Mx` 를 만드는지,
   그리고 그것이 roll DOF 로 어떻게 들어가는지 유도한다.
3. 모든 dynamics model 이 공유하는 단일 contact-kinematics 정의(`tire_contact`)
   의 역할을 안다.
4. Tire interface 가 "slip-in / force-out" 에서 "kinematics-in / wrench-out" 으로
   역전된 구조(`evaluate` / `advance_bristle` / `advance_relaxation`)와, transient
   state 를 dynamics 가 소유하는 이유를 설명한다.
5. 어떤 항이 tire 안으로 들어가고 어떤 항이 vehicle-side(integrator 안정화)로
   남는지 구분한다.

## Prerequisites

- **Chapter 01** — slip ratio `kappa` / slip angle `alpha` 의 ISO 8855 정의.
- **Chapter 03 / 19 / 21** — MF96, LuGre brush, belt relaxation 의 force law 와
  transient.

## 25.1 Effective rolling radius Re(Fz)

타이어는 강체 바퀴가 아니라 하중을 받으면 눌린다. 접지면의 유효 굴림 반경
`Re` 는 unloaded radius `R0` 보다 작고 하중에 따라 변한다. Pacejka form 으로

```
rho_n = Fz / Fz0
Re    = R0 - (Fz0/Cz) * (DREFF*atan(BREFF*rho_n) + FREFF*rho_n)
```

slip ratio 의 정의에 `Re` 를 쓴다.

```
kappa = (omega * Re - Vx) / max(|Vx|, eps)
```

이렇게 하면 **자유 굴림(free-rolling) 중인 하중 받은 타이어가 `kappa = 0`** 을
보고한다 — `omega = Vx / Re` 이기 때문이다. 이것이 MF-Tyre / CarMaker 와 일치하는
관례다. `R0` 를 쓰면 같은 자유 굴림에서 가짜 slip 이 생긴다.

중요한 결과: 초기 wheel spin 도 `Re` 와 정합해야 한다. `free_roll_wheel_spin`
은 정적 하중에서 wheel 별 `omega = Vx / Re(Fz_static)` 를 준다(앞/뒤 정적 하중
분배가 다르므로 wheel 별로 다름). 이 정합이 없으면 `t = 0` 에서 수천 N 의 phantom
longitudinal force 가 나온다. `Re` 는 opt-in 이다(`reff_* = 0` 이면 `R0` 로 fallback,
legacy 동작 유지).

## 25.2 Camber contact-point migration -> overturning moment Mx

타이어 트레드는 toroidal(원환) 단면을 가진다. Camber `gamma` 로 기울면 접지점이
타이어 중심선이 아니라 기운 쪽으로 옮겨간다.

```
contact_dy = crown_radius * sin(gamma)
contact_dz = crown_radius * (1 - cos(gamma))
```

수직 하중선이 `contact_dy` 만큼 옆으로 옮겨가므로 wheel-forward 축에 대한
overturning moment 가 생긴다.

```
Mx = Fz * contact_dy = Fz * crown_radius * sin(gamma)
```

이 `Mx` 는 roll DOF 가 있는 model(L3 14-DOF / L5)에서 roll 방정식으로 들어간다.
ISO roll-camber 패턴(`+g, -g, +g, -g`)에서는 좌/우 `Mx` 가 부호 반대라 대칭 roll
에서 자연히 상쇄된다. Opt-in 이다(`crown_radius = 0` 이면 중심선 접지, `Mx = 0`).

## 25.3 공유 contact-kinematics 모듈

slip / `Re` / camber migration 의 정의는 한 곳(`vdsim/tire_contact.hpp` 의
`tire_contact_kinematics`)에만 있다. 네 dynamics model 이 모두 이것을 호출하므로
정의가 model 마다 달라질 수 없다. 반환은 `{kappa, alpha, Re, vsx, vsy, contact_dy,
contact_dz}` 이다.

## 25.4 Inverted tire interface — kinematics-in / wrench-out

예전 `ITireModel` 은 "slip-in / force-out" 이었다: dynamics 가 slip 을 계산해서
`compute(Fz, kappa, alpha, ...)` 에 넘겼다. 그래서 새 타이어(다른 slip law, transient,
camber model)를 넣으려면 dynamics 3곳을 고쳐야 했다 — 추상화가 덜 된 것이다.

역전 후에는 "kinematics-in / wrench-out" 이다.

```
struct ContactInput  { Fz, Vx, Vy, omega, gamma, mu_long, mu_lat, R0; };
struct Transient     { belt_kappa, belt_alpha, belt_vlong, belt_vlat,
                       lugre_z_long, lugre_z_lat, alpha_dyn; };
struct Wrench        { Fx, Fy, Mx, My, Mz, Re, kappa, alpha, contact_dy; };

Wrench    evaluate(ContactInput, Transient)            // RK4 stage (frozen)
Transient advance_bristle(ContactInput, Transient, dt) // per stage
Transient advance_relaxation(ContactInput, Transient, dt) // per substep
```

`evaluate()` 가 slip / `Re` / camber migration 을 contact kinematics 에서 직접
계산하고, caller 가 준 transient 를 적용한 뒤 wrench 를 돌려준다. force law 는
virtual `compute()` 로 dispatch 되므로 이 plumbing 은 **base 에 한 번만** 정의되고
(`tire_model.cpp`) 모든 backend(MF96 / linear / MF2002)가 공유한다. 새 타이어는
`compute()` override 하나면 된다.

### Transient ownership 과 두 개의 cadence

Transient(belt / relaxation / LuGre bristle)는 **dynamics 가 wheel 별로 소유**한다.
RK4 integrator 가 stage 안에서는 freeze 하고 substep 사이에서 advance 해야 하기
때문이다(tire 는 stateless / polymorphic 유지, per-wheel clone 불필요).

advance 가 두 cadence 로 나뉘는 이유:

- `advance_bristle()` — LuGre bristle `z` 는 빠른 contact state 라 RK4 stage 마다
  (sub-substep `dt`)에서 적분한다.
- `advance_relaxation()` — carcass/belt slip lag + relaxation-length lag 는 느린
  state 라 substep 당 한 번, 마지막 stage 의 geometric slip 기준으로 적분한다.

둘을 한 `dt` 로 합치면 byte-equivalent 가 아니다(느린 lag 를 stage별 target 에 대해
N 번 relax 한 것 != 최종 target 에 대해 1번 relax 한 것). 그래서 분리한다.

## 25.5 무엇이 vehicle-side 로 남는가

다음은 tire physics 가 아니라 integrator 안정화이므로 dynamics 에 남는다.

- Stick-blend `lambda`, `Fx_hold` creep, combined-slip clamp — 저속 수치 안정화.
- Wheel-frame velocity projection(steer / yaw-rate / wheel position) — vehicle kinematics.
- Wheel rotational ODE 와 moment arm `Fx * Re` — vehicle(다만 `Re` 는 tire 가 돌려준 값).
- Kinematic-dynamic blend(저속 lateral) 와 free_3d 의 stunt-loop denom 특화.

## 25.6 적용 범위

L2(seven_dof)와 L3(14-DOF, seven_dof 를 wrap)는 이 inverted interface 를 사용한다.
L1(bicycle)과 L5(free_3d)는 의도적으로 공유 contact 경로(§25.3)에 남아 있다 —
이들의 contact kinematics 가 per-wheel 계약과 다르기 때문이다(bicycle 은 축당 2개
타이어를 묶어 `Re` 는 wheel 하중(`0.5*Fz`), force 는 축 하중(`Fz`)을 쓰고 LuGre slip
은 `R0` 기반; free_3d 는 loop 전용 slip denominator). 새 force law 는 둘 다
`compute()` 로 그대로 들어간다.

## 검증

- Re consistency / no-phantom-force: `ctest -R "EffectiveRollingRadius|NoPhantom"`
  (VALIDATION benchmark #16).
- Camber migration -> Mx: `ctest -R CamberMigration` (benchmark #17).
- Inverted interface 등가: `ctest -R TireInversion` — 네 force 경로 + 세 transient
  적분기를 독립 oracle 에 대해 lock.
- Byte-stability: IsoBaseline / Lugre / Belt / ChronoPac02Parity 가 그대로 통과.

설계·핸드오프 상세는 `docs/design/TIRE_INTERFACE_INVERSION.md`.
