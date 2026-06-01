# Task 05 — D11 계층적 제어 API L1~L8 명세

| Field | Value |
|---|---|
| Task ID | D11 |
| Type | Design |
| Date | 2026-05-28 |
| Commit | `1e15c5e` |
| Status | completed (L1~L4 명세 + L4 impl, L5~L8 명세 reserve) |

## 1. 목적

VDSim 의 핵심 차별화 — "어느 추상 레이어의 제어 알고리즘이든 동일한 차량 응답으로 평가" — 를 ABI 로 구현. CarSim / CarMaker / VI-grade 가 throttle/brake/steer 또는 그 단순 확장만 받는 데 비해, VDSim 은 모터 토크부터 path 까지 8 계층을 통합.

## 2. 구현 방법

### 8 계층 명세

| Level | 입력 | 단위 | Dynamics 직접 처리 |
|---|---|---|---|
| L1 | motor_torque[4], brake_torque[4], steer | N·m, N·m, rad | yes |
| L2 | drive_torque, brake_torque, steer | N·m, N·m, rad | yes |
| L3 | Fx_total, steer | N, rad | yes |
| **L4** | throttle, brake, steer | [0,1], [0,1], rad | **yes (PoC default)** |
| L5 | ax_target, steer | m/s², rad | Converter |
| L6 | v_target, steer | m/s, rad | Converter |
| L7 | v_target, kappa | m/s, 1/m | Converter |
| L8 | path[N], lookahead | — | Converter |

### 핵심 결정

| 결정점 | 채택 | 근거 |
|---|---|---|
| C++ 타입 | `std::variant<CmdL1, ..., CmdL8>` | type-safe, std::visit dispatch |
| Steer 단위 | **모두 wheel angle [rad]** (L4 도) | 일관성. CARLA `[-1,1]` 은 plugin 경계에서 변환 |
| Dyn ↔ Ctrl 분리 | L1~L4 dyn 직접 / L5~L8 Converter | controller logic (PID, Pure Pursuit) 분리 |
| PoC 구현 범위 | L4 (다른 L 은 zero command fall back) | 일정 현실성 |

### 변환 chain (downward, controller layer 책임)

```
L8 ──Pure Pursuit/MPC──▶ L7
L7 ──δ = atan(L·κ)──────▶ L6
L6 ──velocity PID──────▶ L5
L5 ──engine map inv───▶ L4
L4 ──pedal → torque───▶ L3
L3 ──identity──────────▶ L2
L2 ──drive split──────▶ L1
```

각 화살표가 controller (PID, Pure Pursuit, engine map ...). Phase 2 에 `vdsim::control::ControlConverter` 모듈로 구현.

### Dyn 의 step() 안에서의 dispatch

`core/src/bicycle_dynamics.cpp:62`:
```cpp
const CmdL4* cmd_ptr = std::get_if<CmdL4>(&u);
const CmdL4& cmd = cmd_ptr ? *cmd_ptr : zero;  // L4 외 입력은 zero 로 fall back
```

L1~L3 직접 처리는 향후 visit pattern 으로 확장. PoC 는 L4 만.

## 3. 검증 방법 (근거)

PoC 범위에서는 L4 가 정상 동작 검증으로 충분. L1~L3 direct visit / L5~L8 Converter 는 Phase 2.

검증 항목:
- `ControlInput = CmdL4{}` 할당 후 `std::holds_alternative<CmdL4>` 통과
- bicycle step() 이 L4 throttle / brake / steer 에 응답
- L4 외 variant 전달 시 zero command 로 fall back (no throw)

## 4. 검증 결과

| Test | 항목 | 결과 |
|---|---|---|
| `Compile.ControlVariantHoldsL4` | variant 가 CmdL4 hold | pass |
| `BicycleSteadyState.LeftTurnYawRateMatches...` | L4 steer 응답 | pass (자세한 수치는 task 11) |
| `BicycleSteadyState.ZeroSteerStraightLine` | L4 zero command 동작 | pass |

| Coverage | 비율 |
|---|---|
| 명세된 levels | 8/8 |
| Struct 정의 (header) | 8/8 |
| Dyn 에서 visit 구현 | 1/8 (L4) |
| Converter 구현 | 0/8 |

## 5. 판단

- 결과: **partial** (PoC 범위 pass, 전체 8 levels 의 1/8 만 구현)
- 근거: L4 가 bicycle + analytical 검증 통과로 fundamental API 동작 입증. 나머지는 명세된 struct + factory 의존성 reserve.
- Follow-up:
  - L1~L3 direct visit dispatch (W7-W8 CARLA 통합 시).
  - `vdsim::control::ControlConverter` 의 L5/L6 (PID-based velocity tracking) — Phase 2.
  - L8 path tracking (Pure Pursuit / MPC) — Phase 2.
  - 차별화 메시지 ("어느 layer 든") 의 marketing claim 은 Phase 2 까지 보류.
