# Task 02 — D10 핵심 데이터 구조 명세

| Field | Value |
|---|---|
| Task ID | D10 |
| Type | Design |
| Date | 2026-05-28 |
| Commit | `1e15c5e` (헤더 확정) |
| Status | completed |

## 1. 목적

`State`, `ControlInput`, `ContactPoint`, `VehicleParams`, `TireParams`, `SolverParams` 의 메모리 레이아웃과 멤버 명세 확정. 이게 모호하면 동역학 / tire 모델 / contact provider / CARLA plugin 모두 ABI 가 흔들린다.

## 2. 구현 방법

### State (모든 사다리 cover)

L1/L2 에서는 L3 영역 (susp_*) 이 0. 분리 struct 대신 하나의 fat struct.

| 멤버 | 단위 | 사용 |
|---|---|---|
| `position` Vec3 | m | world ENU |
| `orientation` Quat | — | body→world |
| `velocity` Vec3 | m/s | body |
| `angular_velocity` Vec3 | rad/s | body (p, q, r) |
| `wheel_spin[4]` | rad/s | L2+ |
| `susp_compression[4]` | m | L3+ |
| `susp_velocity[4]` | m/s | L3+ |

accessor: `yaw()`, `yaw_rate()`, `speed_xy()`, `beta()`.

### ControlInput = `std::variant<CmdL1, ..., CmdL8>`

| 결정 | 채택 | 근거 |
|---|---|---|
| 표현 | `std::variant` | C++17 표준, type-safe, std::visit 로 dispatch |
| Polymorphic? | 거부 | heap alloc + virtual dispatch → RT 부담 |
| Unified struct + enum? | 거부 | 미사용 필드 혼란, type safety 잃음 |
| steer 단위 | 모든 L 에서 wheel angle [rad] | CARLA `[-1,1]` 은 plugin 측에서 변환 (steering_ratio×max_steer) |

### ContactPoint

| 멤버 | 의미 |
|---|---|
| `position` | world [m] |
| `normal` | unit (default UnitZ) |
| `surface_id` | PhysicalMaterial lookup id |
| `is_valid` | false 시 wheel off ground |
| `mu_long`, `mu_lat` | Pacejka mu 스케일 (각각 long/lat) |
| `penetration` | suspension nominal 대비 [m] |

### TireParams (MF96 simple)

| 그룹 | 멤버 | default | 비고 |
|---|---|---|---|
| Longitudinal MF | B_long, C_long, D_long, E_long | 10, 1.65, 1.0, 0.97 | F=D sin(C atan(B·s − E(...))) |
| Lateral MF | B_lat, C_lat, D_lat, E_lat | 8, 1.30, 1.0, −1.0 | 동일 |
| Friction | mu_nominal, Fz_nominal | 1.0, 4000 N | 기준 mu / Fz |
| Linear | cornering_stiffness | 80000 N/rad | 검증 / linear tire 용 |

### VehicleParams 핵심 default (mid-sedan)

| 그룹 | 값 |
|---|---|
| mass / mass_sprung | 1500 / 1350 kg |
| inertia diag (sprung) | (500, 2000, 2500) kg·m² |
| L / a / b | 2.7 / 1.2 / 1.5 m |
| Tw_f / Tw_r | 1.55 / 1.55 m |
| h_cg | 0.55 m |
| wheel R | 0.32 m |
| spring k (FL..RR) | 30000 N/m × 4 |
| damper c | 3000 N·s/m × 4 |
| roll stiff f / r | 30000 / 25000 N·m/rad |
| drive | RWD, max T 300 N·m, max brake 2000 N·m |
| steer ratio / max wheel | 15 / 0.5 rad |
| aero Cd × A | 0.30 × 2.2 m² |

### SolverParams

| 멤버 | default | 근거 |
|---|---|---|
| integrator | RK4 | 14-DOF 까지 안정 |
| max_substep_dt | 1 ms | L3 stiffness 견딤 |
| max_substeps | 10 | CARLA tick 0~20 ms 범위 cover |

## 3. 검증 방법 (근거)

명세 그 자체는 헤더 작성 단계의 smoke test 로 확인. 본격 검증은:
- 모든 헤더 include + default 인스턴스 생성 성공
- ControlInput 의 variant 가 L4 hold 후 std::get 으로 회복 가능
- VehicleParams default 가 `cg_to_front + cg_to_rear == wheelbase` 등 self-consistent

## 4. 검증 결과

`tests/unit/test_headers_compile.cpp` 6 test, 6/6 pass.

| Test | 확인 항목 | 결과 |
|---|---|---|
| Compile.TypesInstantiate | Vec3 / Quat / NUM_WHEELS | pass |
| Compile.StateDefaultsZero | State 모든 멤버 0 | pass |
| Compile.ContactDefaults | normal = UnitZ, mu_long = 1.0 | pass |
| Compile.ControlVariantHoldsL4 | variant holds_alternative<CmdL4> | pass |
| Compile.ParamsDefaultsPositive | mass>0, L=a+b, RK4 default | pass |
| Compile.EulerStructDefaults | roll/pitch/yaw = 0 | pass |

## 5. 판단

- 결과: **pass**
- 근거: 6/6 smoke test 통과, 후속 task 들 (tire 모델 / bicycle / contact provider) 이 본 구조를 사용해 모두 빌드 + 31 + 6 = 37 tests pass.
- Follow-up:
  - `VehicleParams::from_yaml` / `to_yaml` 직렬화는 task 12 에서 구현.
  - `TireParams::from_tir` (AVL .tir 임포트) 는 Phase 2.
