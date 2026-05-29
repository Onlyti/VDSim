# Task 09 — coordinate.cpp 구현 + 단위 테스트

| Field | Value |
|---|---|
| Task ID | W2 coordinate impl |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | `56cc48a` |
| Status | completed |

## 1. 목적

D7 좌표계 명세를 코드로 변환 + 단위 테스트. 좌표 변환 버그는 CarSim / Chrono CARLA 통합 초기 단골 (CARLA 통합 가이드 §10.1) — 후속 task 들이 안심하고 사용할 수 있게 11 개 단위 테스트로 cover.

## 2. 구현 방법

### `coordinate.cpp` 함수별 구현 노트

| 함수 | 방법 | 비고 |
|---|---|---|
| `quat_from_euler` | `AngleAxisd(yaw,Z) * AngleAxisd(pitch,Y) * AngleAxisd(roll,X)` | ZYX intrinsic, Eigen 표준 |
| `euler_from_quat` | 직접 q 성분 → roll/pitch/yaw | Eigen `eulerAngles(2,1,0)` 의 범위 surprise 회피 |
| `yaw_from_quat` | `atan2(2(qw qz + qx qy), 1 − 2(qy² + qz²))` | gimbal-safe |
| `ue::*_position` | 1/100 + Y flip | UE5 cm,LH ↔ VDSim m,RH |
| `ue::*_rotation` | quat (w,x,y,z) ↔ (w,−x,y,−z) | Y axis 반사 = LH↔RH conversion |
| `ue::*_velocity` | position 과 동일 transform | linear, scale 1/100 + Y flip |

### Quaternion Y-flip 유도

UE LH 에서 axis-angle `(ax, ay, az, θ)` → VDSim RH 로 변환:
- Y flip: ay → −ay
- LH→RH: θ → −θ (회전 방향 반대)

Quaternion 으로:
```
q_LH = (cos(θ/2), ax·sin(θ/2), ay·sin(θ/2), az·sin(θ/2))
q_RH = (cos(−θ/2), ax·sin(−θ/2), (−ay)·sin(−θ/2), az·sin(−θ/2))
     = (cos(θ/2), −ax·sin(θ/2), ay·sin(θ/2), −az·sin(θ/2))
     = (qw_LH, −qx_LH, qy_LH, −qz_LH)
```

→ involution (`f(f(x)) = x`), 즉 from_ue 와 to_ue 가 같은 식.

### 외부 frame 흐름도

```
ADMA/GPS (ENU, m, RH) ────identity──── VDSim world (ENU, m, RH)
                                              │
                          coordinate.hpp       │
                                              │
                          ┌──────── body→world ────┐
                          ▼                        │
                    VDSim body                     │
                  (ISO 8855, m, RH)                │
                          │                        │
   Y flip + ×100 ─────────┤                        │
                          ▼                        │
                    UE5 body/world                 │
                  (LH, cm, Z up, Y right)          │
                                                   │
   Y flip ────────── MORAI Unity ──────────────────┘
                  (LH, m, Z up, Y right)
```

## 3. 검증 방법 (근거)

검증 4 카테고리:
1. **Roundtrip** — `f⁻¹(f(x)) ≈ x` (수치 안정성 + 부호 일관)
2. **Sign / direction** — yaw +π/2 이 body X → world +Y 매핑 (좌측 = Y+ in ISO)
3. **UE 부호 반전** — VDSim yaw +π/2 (CCW, RH) ↔ UE yaw −π/2 (CW, LH)
4. **Identity preservation** — `Quat::Identity()` → `Quat::Identity()` 유지

Tolerance: 1e-9 (RK4 누적 오차보다 10⁶ 이상 정밀)

## 4. 검증 결과

`tests/unit/test_coordinate.cpp` 11 tests, **11/11 pass**.

| Test | 검증 | 결과 |
|---|---|---|
| QuatFromEulerIdentity | (0,0,0) → Quat(1,0,0,0) | pass (≤ 1e-9) |
| PureYawQuat | yaw=π/4 → qw=cos(π/8), qz=sin(π/8) | pass |
| EulerRoundtripSmallAngles | (0.05, 0.03, 0.7) → quat → euler | pass (≤ 1e-12) |
| EulerRoundtripNegativeYaw | yaw=−π/3 | pass |
| YawCcwPositive | yaw +π/2: (1,0,0)_body → (0,1,0)_world | pass |
| PositionForward | (1,2,3)_VDSim → (100,−200,300)_UE | pass |
| PositionRoundtrip | VDSim → UE → VDSim identity | pass |
| VelocityRoundtrip | 동일 (linear) | pass |
| RotationRoundtrip | quat (0.1, 0.2, 0.3) → UE → VDSim | pass (\|dot\| = 1) |
| YawSignFlip | VDSim yaw +π/2 → UE qz 부호 반전 | pass |
| IdentityIsIdentity | I → I | pass |

### 수치 예 (대표)

| Input (VDSim) | UE | Inverse | Error |
|---|---|---|---|
| pos (1.0, 2.0, 3.0) m | (100.0, −200.0, 300.0) cm | (1.0, 2.0, 3.0) m | 0 |
| pos (1.234, −5.678, 9.0) m | (123.4, 567.8, 900.0) cm | (1.234, −5.678, 9.0) m | < 1e-15 |
| Euler (0.05, 0.03, 0.7) rad | — | (0.05, 0.03, 0.7) rad | < 1e-13 |
| vy +π/2 (좌회전) | UE qz=−sin(π/4) | yaw +π/2 | 부호 반전 정확 |

### 빌드 / 테스트 통계

| 항목 | 값 |
|---|---|
| LOC (impl) | 78 |
| LOC (test) | 119 |
| Total ctest | 19/19 pass (위 11 + 8 이전) |
| Build 시간 (incremental) | < 5 s |

## 5. 판단

- 결과: **pass**
- 근거: 11/11 단위 테스트 모두 tolerance (1e-9 ~ 1e-12) 이내. UE 변환의 involution 성질, yaw 부호, roundtrip 모두 검증. 후속 task 11 의 bicycle 가 본 함수들 (특히 `yaw_from_quat`, `quat_from_euler`) 사용해 analytical agreement 통과.
- Follow-up:
  - MORAI Unity 변환은 헤더에 reserve 했으나 impl 미작성. 한국 시장 영업 task 시 추가.
  - NED 변환 (aerospace) — Phase 2.
