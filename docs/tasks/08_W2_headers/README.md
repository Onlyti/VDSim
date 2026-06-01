# Task 08 — IM-W2 인터페이스 헤더 7종 작성

| Field | Value |
|---|---|
| Task ID | IM-W2 / Task #9 |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | `1e15c5e` |
| Status | completed |

## 1. 목적

D7~D11 의 설계 결정을 코드로 변환. 헤더만 작성 (impl 은 후속 task). 빌드 가능 + 스모크 테스트 통과까지가 본 task 의 통과 기준.

## 2. 구현 방법

### 7 헤더 (`core/include/vdsim/`)

| 파일 | LOC | 의존 헤더 | Task 매핑 |
|---|---|---|---|
| `types.hpp` | 21 | `<Eigen/Core>`, `<Eigen/Geometry>` | D7 (좌표계 type alias) |
| `coordinate.hpp` | 41 | `types.hpp` | D7 |
| `state.hpp` | 49 | `coordinate.hpp`, `types.hpp` | D10 |
| `contact.hpp` | 20 | `types.hpp` | D10 |
| `control.hpp` | 71 | `types.hpp` | D11 |
| `params.hpp` | 75 | `types.hpp` | D10 |
| `interfaces.hpp` | 92 | 모든 위 헤더 + `<memory>` | D9 |

### 의존 그래프

```
types.hpp ─┬─> coordinate.hpp ─> state.hpp ─┐
          ├─> contact.hpp ─────────────────┼─> interfaces.hpp
          ├─> control.hpp ─────────────────┤
          └─> params.hpp ──────────────────┘
```

### Build 변경

`tests/unit/CMakeLists.txt` 에 `test_headers_compile.cpp` 추가. 6 smoke test:

| Test | 검증 |
|---|---|
| TypesInstantiate | Vec3 / Quat::Identity / NUM_WHEELS = 4 |
| StateDefaultsZero | state.position.norm() = 0, velocity.norm() = 0 |
| ContactDefaults | is_valid = false, normal.z() = 1.0 |
| ControlVariantHoldsL4 | std::holds_alternative<CmdL4> |
| ParamsDefaultsPositive | mass > 0, L = a+b, integrator = RK4 |
| EulerStructDefaults | roll/pitch/yaw = 0 |

## 3. 검증 방법 (근거)

헤더 검증은:
1. 빌드 가능성 (compile-only)
2. Default constructor 가 valid object 생성
3. Aggregate type self-consistency (`L == a + b` 등)

## 4. 검증 결과

```
[48/48] Built (incremental: 2/48 changed: test_headers_compile.cpp.o + vdsim_unit_tests)
100% tests passed, 0 tests failed out of 8
Total Test time (real) = 0.01 sec
```

| 빌드 항목 | 결과 |
|---|---|
| 7 헤더 모두 include + 빌드 | pass |
| 컴파일 경고 (-Wall -Wextra -Wpedantic) | 0 |
| Smoke test 6 개 | 6/6 pass |
| 전체 test (version 2 + smoke 6) | 8/8 pass |

### LOC 통계

| 그룹 | 줄 수 |
|---|---|
| Header decl total | 369 |
| Test scaffold | 80 |
| **Total commit** | **~450 lines** |

## 5. 판단

- 결과: **pass**
- 근거: 8/8 test 통과. 헤더 의존 그래프 cycle 없음. 후속 task 09, 10, 11 의 impl 이 본 헤더 위에 빌드 성공.
- Follow-up:
  - `params.hpp` 의 `from_yaml` / `to_yaml` impl: task 12.
  - `interfaces.hpp` 의 factory 들 impl: task 10 (tire), task 11 (bicycle).
