# Task 01 — D7 좌표계/단위 표준 확정

| Field | Value |
|---|---|
| Task ID | D7 |
| Type | Design |
| Date | 2026-05-28 |
| Commit | `d267221` (skeleton) / `56cc48a` (impl) |
| Status | completed |

## 1. 목적

VDSim core 와 외부 시뮬레이터 (CARLA / UE / MORAI Unity / CarMaker / ADMA) 사이에서 좌표계와 단위가 일관되지 않으면 sign 버그가 동역학 결과를 통째로 망친다. CarSim / Chrono CARLA 통합 사례에서 좌표 변환은 단골 초기 버그였다 (CARLA 통합 가이드 §10.1). 본 task 는 core 내부의 canonical convention 과 외부 frame 매핑을 PoC 초기에 lock 한다.

## 2. 구현 방법

### Canonical (내부)

| 항목 | 값 | 근거 |
|---|---|---|
| 단위 | SI (m, kg, s, rad, N, N·m) | 학계 표준, CarMaker / ADMA 호환 |
| Body frame | ISO 8855 RH — X 전방, Y 좌측, Z 상방 | CarMaker / ADMA 와 동일 |
| World frame | ENU RH — X 동, Y 북, Z 상 | GPS / ADMA / KITTI 호환 |
| Quaternion | body → world (Eigen 기본) | ROS tf2 / Probabilistic Robotics 일치 |
| Euler 순서 | ZYX intrinsic (yaw → pitch → roll) | ISO 8855 / SAE J670 |
| Wheel index | FL=0, FR=1, RL=2, RR=3 | 사용자 lab convention (CLAUDE.md global) |

### 외부 frame 매핑

| Frame | 단위 | Hand | X | Y | Z | 변환 |
|---|---|---|---|---|---|---|
| VDSim world / body | m | RH | fwd / east | left / north | up | canonical |
| UE 5 | cm | LH | fwd | right | up | ×100, Y flip |
| MORAI Unity | m | LH | fwd | right | up | Y flip |
| ENU / ADMA | m | RH | east | north | up | identity |
| CarMaker | m | RH | fwd | left | up | identity |

### API (선언)

`core/include/vdsim/coordinate.hpp`:
```cpp
struct Euler { double roll, pitch, yaw; };
Quat   quat_from_euler(const Euler&);
Euler  euler_from_quat(const Quat&);
double yaw_from_quat(const Quat&);
namespace ue {
    Vec3 from_ue_position(const Vec3&);    // cm,LH -> m,RH
    Vec3 to_ue_position(const Vec3&);
    Quat from_ue_rotation(const Quat&);
    Quat to_ue_rotation(const Quat&);
    Vec3 from_ue_velocity(const Vec3&);
    Vec3 to_ue_velocity(const Vec3&);
}
```

## 3. 검증 방법 (근거)

좌표계 명세는 그 자체로 코드 검증할 수 없으므로 후속 task 09 (coordinate.cpp 구현)의 단위 테스트가 명세를 만족함을 확인하는 형태.

검증 기준:
- 위치 / 속도 / 회전 모두 VDSim ↔ UE roundtrip 시 identity (1e-9 이내)
- pure yaw +π/2 (좌회전, RH) → UE 에서 −π/2 (CW, LH)
- 좌회전 yaw +π/2 적용 시 body X(1,0,0) → world (0,1,0) (left)
- Euler ↔ Quat roundtrip (gimbal 회피 case)

## 4. 검증 결과

후속 task 09 의 단위 테스트 11/11 통과. 자세한 결과는 [task 09 보고서](../09_W2_coordinate_impl/README.md) §4 참조.

명세 자체의 self-consistency 검증:

| 변환 | 검증 | 결과 |
|---|---|---|
| VDSim Y left, UE Y right | (1,2,3)m → (100,−200,300)cm | ✓ |
| yaw +π/2 RH ↔ yaw −π/2 LH | quat z 부호 반전 | ✓ |
| ENU world ↔ CarMaker world | 둘 다 RH, 단위 m, identity | ✓ |
| ISO 8855 body ↔ CarMaker body | 둘 다 X fwd, Y left, Z up | ✓ |

## 5. 판단

- 결과: **pass**
- 근거: 명세가 자체 일관 + 모든 외부 frame 매핑 1대1 가역. 후속 구현 task 09 의 11/11 단위 테스트 통과 (commit `56cc48a`).
- Follow-up:
  - NED (aerospace) 변환은 현재 미포함. ADMA 일부 모드에서 NED 사용 가능 — Phase 2 에 검토.
  - Wheel index 가 일부 한국 OEM 자료에서 다른 순서를 쓰는 경우 어댑터 필요.
