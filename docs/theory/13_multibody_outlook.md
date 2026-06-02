# 13. Multibody (Ld4-Ld5) — Overview

> **상태 안내.** 이 챕터는 lumped 14-DOF (Ld3) 와 본격 multibody (Ld4-Ld5) 의
> *동기·데이터모형·로드맵* 을 다룬다. Ld4 의 hardpoint kinematics 가 실제로
> **구현 완료**됐다 (DW / MacPherson / trailing arm / 5-link). 그 구체적 운동학
> 식과 solver 는 **Chapter 14 (Hardpoint Kinematics)** 에서 다룬다. 본 챕터는
> 전체 그림과 Ld5 (compliance, 아직 미구현) 의 outlook 을 유지한다.

> **학습 목표.** Lumped 14-DOF (Ld3) 와 본격 multibody (Ld4-Ld5) 의 차이를 식 단위로 안다. hardpoint + joint + bushing 의 abstraction 으로 어떤 suspension topology 든 표현 가능한 이유를 안다. forward kinematics (FK) vs full DAE (Featherstone) 의 trade-off 를 안다. Adams Car 와의 cross-validation 의 의의를 명확히 한다.

## 13.1 왜 lumped 14-DOF 로 부족한가

Chapter 06 의 Ld3 는:

- sprung body 의 corner displacement 가 `z_s + ry · φ − rx · θ` 로 표현 — small-angle linear.
- per-corner spring/damper 가 sprung corner ↔ unsprung mass 사이에 작용.
- unsprung mass 가 single-DOF (vertical) 의 point mass.

이게 ride 의 quasi-static + transient response 까지는 정확. 그러나:

**Lumped 의 한계** (Ld4-Ld5 가 채우는 것):
1. **wheel kinematics 부재** — bump steer, roll steer, camber-from-roll 등 suspension geometry 효과 표현 불가.
2. **anti-dive / anti-squat 의 정확한 model 부재** — scalar factor 로 근사 (`(1 − anti)` term).
3. **bushing compliance** — corner spring 의 효과만, bushing 의 6-DOF compliance 없음.
4. **suspension topology 의 차이** — MacPherson / double-wishbone / multi-link 의 차이가 표현 안 됨.
5. **K&C (Kinematics & Compliance) chart 부재** — chassis 설계자의 표준 도구.

## 13.2 Multibody 의 데이터 모형

VDSim 의 `core/include/vdsim/multibody.hpp` (M0 stub):

```cpp
namespace vdsim::mb {

struct RigidBody {
    std::string id;
    double      mass;
    Mat3        inertia_body;      // diagonal in PoC
    Vec3        cg_local;          // CG offset in body frame
    Vec3        position_world;    // (filled by solver)
    Quat        orientation;
    Vec3        velocity_world;
    Vec3        omega_body;
};

enum class JointType { Ball, Revolute, Cylindrical, Prismatic, Universal, Rigid };

struct Joint {
    std::string id;
    JointType   type;
    std::string body_a_id;
    std::string body_b_id;
    Vec3        position_in_a;     // joint location in body A frame
    Vec3        position_in_b;
    Vec3        axis_in_a;         // for revolute / cylindrical / prismatic
};

struct Bushing {
    // 6-DOF linear stiffness + damping in bushing-local axes
    Vec3 k_translation;            // [N/m]
    Vec3 k_rotation;               // [N·m/rad]
    Vec3 c_translation;            // [N·s/m]
    Vec3 c_rotation;
};

struct Hardpoint {
    std::string name;              // e.g. "lca_inner_front"
    Vec3        position;          // body-frame coordinate
    std::string body_id;
};

struct SuspensionTopology {
    TopologyKind                       kind;
    std::vector<RigidBody>             bodies;
    std::vector<Joint>                 joints;
    std::vector<Bushing>               bushings;
    std::map<std::string, Hardpoint>   hardpoints;
    // K&C outputs
    double toe_deg, camber_deg, caster_deg, kingpin_incl_deg,
           scrub_radius_mm, mech_trail_mm;
};

class IMultibodySolver {
public:
    virtual WheelPose forward_kinematics(const SuspensionTopology&,
                                          double travel_z, double steer_rad) const = 0;
    virtual void quasi_static_compliance(SuspensionTopology&,
                                          const WheelLoad&) const = 0;
    virtual void step_dynamics(SuspensionTopology&, double dt) const = 0;
};
```

![Four suspension topologies](figures/13_suspension_types.png)

## 13.3 6 가지 standard suspension topology

`configs/suspensions/`:
| Topology | 차종 / Use |
|---|---|
| MacPherson strut | sedan front |
| Double wishbone (SLA) | sports / race front |
| Multi-link 5-link | sedan rear |
| Trailing arm | compact rear |
| Beam axle (rigid) | truck / 4WD rear |
| Twist beam | FWD compact rear |
| (De Dion 등 후속) | rare |

각 topology 가 YAML 로 hardpoint + joint + bushing 명세.

### MacPherson 예 (sedan FL)

bodies: chassis (reference), lower_control_arm (mass 8.5, inertia diag), strut (12.0), knuckle (6.5), tie_rod (1.8).

hardpoints:

- lca_inner_front
- lca_inner_rear
- lca_outer_ball
- strut_top_mount
- strut_to_knuckle
- tie_rod_inner
- tie_rod_outer
- wheel_center

joints:

- lca_inner_front: revolute (axis = body-x) between chassis and LCA.
- lca_inner_rear: revolute (axis = body-x) between chassis and LCA.
- lca_outer_ball: ball joint between LCA and knuckle.
- strut_top_mount: ball joint between chassis and strut top.
- strut_to_knuckle: prismatic (axis = strut local z) between strut and knuckle.
- tie_rod_inner: ball joint between chassis and tie rod inner.
- tie_rod_outer: ball joint between tie rod and knuckle.

이게 표준 MacPherson kinematic skeleton.

## 13.4 Forward Kinematics — M1 의 목표

### Problem definition

given:

- `SuspensionTopology` (모든 hardpoint, joint).
- `travel_z` — suspension vertical travel input.
- `steer_rad` — steering wheel angle (steering rack 위치).

output:

- `WheelPose`: wheel center 의 world 위치 + orientation + toe, camber, caster.

### MacPherson 의 simplified FK

```
1. strut_top_mount 위치는 chassis frame 에 fixed.
2. strut_to_knuckle 의 prismatic axis 는 strut 의 local z.
3. lca outer ball 의 위치는 (lca_inner pivot 의 회전축) 주변 원 안.
4. tie_rod outer 는 (tie_rod_inner) 주변 원 안.
5. knuckle 의 6-DOF 자세가 (lca_outer, strut_to_knuckle, tie_rod_outer) 세 위치로 결정.
```

이 시스템 = 3 거리 조건 (constraint) + 3 회전 DOF (knuckle) = 6 unknowns.

해법: Newton-Raphson iteration 또는 closed-form (geometry 활용).

```
function FK(travel_z, steer_rad):
    1. lca 의 outer ball 위치 = (LCA pivot 주변 원의 angle 으로 표현)
       knuckle position 후보 = lca_outer
    2. tie_rod 의 outer 위치 = (tie_rod_inner + steer_rack_offset)
       constraint: distance from knuckle origin = tie_rod 길이
    3. strut_to_knuckle = (strut_top_mount + strut_extension * strut_axis)
       strut_extension 은 travel_z 에 대응
    4. Newton-Raphson 으로 (LCA angle, strut_extension, knuckle yaw) 해결
    5. knuckle pose → wheel_center 의 world 위치 + orientation
    6. toe, camber, caster = wheel pose 의 Euler 분해
```

closed-form 또는 12-line analytical 가능. M1 의 1차 구현.

### K&C chart (M5)

travel_z 를 ±100mm sweep, steer 를 ±0.5 rad sweep:

- toe(travel) — bump steer
- camber(travel) — camber gain
- caster(steer) — caster trail change
- track_change(travel) — scrub
- roll_center(travel) — roll center height

이게 chassis 설계자의 표준 chart. Adams Car / VI 가 GUI 로 자동 plot.

VDSim Phase 2: forward kinematics 후 sweep → matplotlib chart 자동 생성.

## 13.5 Quasi-Static Compliance — M3 의 목표

### Problem

given:

- topology + travel (FK 결과).
- WheelLoad (Fx, Fy, Fz at wheel center).

output:

- bushing 의 deflection (compliance).
- updated wheel pose (compliance steer / camber).

### Linear bushing model

```
F_bushing  =  − K · q_local  −  C · q_dot_local
```

`K, C` 가 6×6 diagonal (translation 3 + rotation 3 of bushing-local axes).

quasi-static 가정 시 `q_dot = 0`, force balance:
```
K · q_local  =  F_external_local
q_local  =  K^{-1} · F_external_local
```

이게 bushing 각각에 대해 풀리고, 전체 system 의 deflection 으로 wheel pose 가 약간 회전 / 평행이동.

real-world 의 compliance steer (Fy 가 toe 변화 만드는 정도) 가 본격 모델링됨. 본 PoC 의 Ld1-Ld3 는 무시.

## 13.6 Full Multibody Dynamics — M4 의 목표

### Featherstone's algorithm

`Rigid Body Dynamics Algorithms` (2008) 의 표준.
constraint 가 있는 multibody system 의 EoM 을 풀기 위한 효율적 알고리즘 (O(n) for kinematic tree, O(n³) for general).

### Augmented Lagrangian / Maxwell-stress

constraint 를 violation penalty 로 처리. 정확하지만 stiff.

### 본 PoC 의 선택

M4 의 implementation 은 deferred. M5 (K&C chart) 가 chassis 설계 측 deliverable 의 80 % 를 cover 하므로 우선순위.

## 13.7 Adams Car / VI-grade 와의 cross-validation — M7

동일 차종의 hardpoint 데이터 (Adams .adm template) 를 import 해 VDSim 의 K&C 와 비교:

| 항목 | metric | target |
|---|---|---|
| Toe gain | mm/° at ±50 mm travel | ±5 % vs Adams |
| Camber gain | °/° | ±5 % |
| Roll center | height [mm] at static | ±10 mm |
| Caster trail | mm | ±5 % |
| Scrub radius | mm | ±3 mm |

이게 본격 K&C cross-validation. 만족 시 chassis 설계자가 신뢰 가능.

## 13.8 Adams .adm import 의 schema

Adams Car template `.tpl` 또는 `.adm` 파일의 hardpoint 정의:
```
HARDPOINTS:
  lca_front: 0.0, 0.30, -0.05
  lca_rear:  ...
  ...
```

parser:

- regex 또는 yacc 으로 hardpoint coord 추출.
- joint topology (revolute axes, ball joints) 도 별도 키워드.
- bushing parameter (Adams 의 bushing curve) 는 linear 부분만 import (nonlinear curve 는 polynomial fit 옵션).

본 PoC 의 `python/tir_to_yaml.py` 의 패턴이 적용 가능.

## 13.9 Roadmap (M1-M7)

| Stage | 상태 | deliverable |
|---|---|---|
| M0 | ✅ done | header + topology YAML stub |
| M1 | ✅ **done** | MacPherson FK + native solver (Chapter 14.4) |
| M2 | ✅ **done** | Double wishbone + trailing arm + 5-link FK (Chapter 14.3/5/6) |
| M3 | ⏳ Phase 2 | Quasi-static compliance (bushing) — 사용자 보류 |
| M4 | ⏳ Phase 2 | Full DAE (Featherstone) — M5 가 deliverable 의 대부분 cover |
| M5 | 🟡 부분 | sweep CSV + plot 있음; 표준 K&C chart 형식은 미완 |
| M6 | ✅ **done** | Adams CSV import (`import_hardpoints.py`, Chapter 14.9) |
| M7 | ⏳ Phase 2 | Adams vs VDSim cross-validation (실측 데이터 대기) |

M1-M2, M6 가 이번 PoC 확장에서 완료. M3 (compliance) 가 Ld5 의 핵심으로 남음.
FK 가 lookup + native 두 backend 로 구현됨 (Chapter 14.7).

## 13.10 차별화 요약

VDSim Phase 2 의 unique claim:
> "Single C++ + Python ABI 안에서 Ld1-Ld3 (autonomy 도메인의 simplified) 부터 Ld4-Ld5 (chassis 도메인의 multibody) 까지의 사다리 전체 cover. control 사다리 Lc1-Lc8 와 m × n 통합."

Adams Car 는 Ld4-Ld5 영역만, control 측은 Simulink 외부. VDSim 의 통합이 unique.

## 13.11 한계

| 항목 | 현재 상태 |
|---|---|
| FK solver impl | ✅ DW/MP/TA/5-link (lookup + C++ native), Chapter 14 |
| Hardpoint YAML 의 값 | indicative (실측 fit 필요) |
| Bushing compliance | ❌ 강체 joint 만 (Ld5, Phase 2 — 사용자 보류) |
| Tire 통합 | ✅ camber/toe → Ld2 → Pacejka (Chapter 14.8) |
| Full DAE dynamics | ❌ FK + quasi-static 만 (M4 Phase 2) |
| K&C 표준 chart | 🟡 sweep CSV + plot; Adams 형식은 미완 |

Ld4 kinematics 는 구현 완료. Ld5 compliance 가 Phase 2 의 다음 큰 항목.

## 13.12 참고

- Featherstone, R., *Rigid Body Dynamics Algorithms*, Springer, 2008 (multibody 표준).
- Genta, G., *Motor Vehicle Dynamics*, World Scientific, 2014 — §8 (multibody chassis).
- Reimpell, J., Stoll, H., Betzler, J., *The Automotive Chassis: Engineering Principles*, Butterworth-Heinemann, 2001 — suspension geometry / K&C.
- Adams Car User Manual (MSC Hexagon) — `.adm` schema (commercial documentation).
- 본 PoC 의 `core/include/vdsim/multibody.hpp` + `configs/suspensions/*.yaml`.
