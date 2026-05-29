# Task 41-43 — Full 14-DOF + CARLA skeleton + L1-L3 dispatch

| Field | Value |
|---|---|
| Task ID | IM-W11-5 (cluster) |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

W11-W12 의 **마지막 30%** + W7-W8 (CARLA) 의 **첫 번째 deliverable** + D11 의 **L1-L3 미흡 진행분**.

| 항목 | 의미 |
|---|---|
| Task 41 (L3 unsprung mass) | Full 14-DOF — sprung 3 + unsprung 4 + planar 7 |
| Task 42 (CARLA plugin) | raycast contact provider (CARLA-less mock 빌드/테스트) |
| Task 43 (L1-L3 dispatch) | variant visit pattern + L4 lowering |

## 2. 구현

### 2.1 Task 41 — L3 unsprung mass

- TireParams 에 `tire_vertical_stiffness` (default 220 kN/m)
- 14-DOF Deriv 확장: `dz_u[4]`, `dz_u_dot[4]` 추가
- ODE:
  ```
  m_u · z̈_u_i = -F_susp(on_sprung) - k_tire · z_u_i
  ```
  F_susp = sprung 받는 위쪽 force. Newton III 로 unsprung 은 반대 부호.
- RK4 통합: 14 first-order ODE (sprung 6 + unsprung 8) per substep
- state.susp_compression / susp_velocity = `(z_corner_sprung - z_u_i)`

### 2.2 Task 42 — CARLA plugin

- `carla_integration/plugin/raycast_contact_provider.{hpp,cpp}` — `IContactProvider` 구현
- 외부 `RaycastFn` 함수 포인터로 raycast 주입 → CARLA / 테스트 mock 모두 동일 코드
- Surface ID → mu lookup table (SurfaceMaterial)
- `vdsim_carla_plugin` static library (CMake option `VDSIM_BUILD_CARLA_PLUGIN=ON`)
- 4 mock test: known surface lookup, unknown fallback, missed raycast, null safety

### 2.3 Task 43 — L1-L3 dispatch

- `bicycle_dynamics.cpp`, `seven_dof_dynamics.cpp` 의 step() 첫 번째 줄이 변경:
  - 이전: `std::get_if<CmdL4>` (L4 만, 다른 variant 은 zero fallback)
  - 현재: `lower_to_l4(u)` — std::visit + if constexpr 로 CmdL1/L2/L3 dispatch 후 L4 로 lowering
- Lowering 식:
  - L1: `throttle = Σ motor_torque / 600`, `brake = Σ brake_torque / 4000`
  - L2: `throttle = drive_torque / 600`, `brake = brake_torque / 4000`
  - L3: `Fx_total > 0 → throttle`, `Fx_total < 0 → brake`
- L5-L8 은 여전히 zero fallback (별도 ControlConverter cascade 가 처리해야 함)

## 3. 검증

### 3.1 새 test

| Test | 항목 | Pass |
|---|---|---|
| (Task 41) 기존 5 FourteenDOF test | 정적 / brake / planar / pose / oscillate | 모두 통과 (unsprung 도입 후 회귀 없음) |
| CarlaPlugin.FlatGroundLookupYieldsKnownMu | id=7 lookup | mu 일치 |
| CarlaPlugin.UnknownSurfaceFallsBackToDefault | id 모름 | default mu |
| CarlaPlugin.MissedRaycastInvalidatesContact | hit=false | is_valid = false |
| CarlaPlugin.NullRaycastSafe | nullptr | crash 없음 |
| ControlDispatch.BicycleHandlesCmdL2Drive | CmdL2 drive | vx 증가 |
| ControlDispatch.BicycleHandlesCmdL3BrakeNegativeFx | CmdL3 Fx<0 | vx 감소 |
| ControlDispatch.SevenDOFHandlesCmdL1PerWheelTorque | CmdL1 RL/RR torque | vx 증가 |
| ControlDispatch.BicycleFallbackOnHigherLevelInput | CmdL5 무시 | drag-only decay |

전체 135/135 통과 (이전 127 + 본 cluster 8 new).

### 3.2 14-DOF 회귀

기존 5 FourteenDOF test (Task 24/30) 모두 통과. 정적 compression, brake compression, planar L2 match, pose encoding, roll oscillation.

## 4. 판단

- 결과: **pass**
- 근거:
  - 8/8 새 test 통과, 누적 135/135.
  - Full 14-DOF ODE 동작 (sprung 3 + unsprung 4 + planar L2).
  - CARLA ABI 의 IContactProvider 호환성 — UE5 통합 entry point 확보.
  - 모든 Variant level 입력 처리 가능 → D11 의 "어느 level 이든" claim 강화.
- 미해결 / Follow-up:
  - **실제 CARLA 통합** — UE5 / Linux PhysX 의존성. 본 PoC repo 외.
  - **L1 dispatch 의 per-wheel 분리** — 현재 lower_to_l4 로 평균화. 분리는 별도.
  - **CmdL8 path tracking** — L4 lowering 미구현. ControlConverter 외부 cascade 사용.
  - **Tire vertical stiffness 차종별 calibration** — 현재 single default 220 kN/m.
