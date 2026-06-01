# VDSim — `feat/poc-w5-to-95pct` 브랜치 요약

이 페이지는 이 브랜치에서 새로 추가된 작업 (T22 ~ T30) 의 한눈 정리.

## 누적 task ladder

```
T22  CARLA Python bridge          ─┐
T23  Jump 14-DOF (Python proto)    │
T24  3D turning jump               │  사용 사례 / 시연 / 통합
T25  Tire upgrades (load+relax)    │
T26  Camber path (Ld3→Ld2)         │
                                   │
T27  Ld4 Stage A-D : DW            ─┤
T28  Ld4 Stage E1   : MacPherson    │  하드포인트 기반 multibody framework
T29  Ld4 Stage E2   : trailing arm  │
T30  Ld4 Stage E3   : 5-link rear  ─┘
```

## Ld4 (hardpoint kinematics) 의 핵심 - C++ side 가 type-agnostic

```
        ╔══════════════════════════════════════════════╗
        ║  ░░ Offline (type-별 솔버) ░░░░░░░░░░░░░░░░  ║
        ╠══════════════════════════════════════════════╣
        ║   hardpoint YAML (type-specific schema)       ║
        ║                ↓                              ║
        ║   tools/kinematics/*_3d_solver.py             ║
        ║   (DW / MP / TA / 5-link 각 type)             ║
        ║                ↓                              ║
        ║   sweep CSV (universal format)                ║
        ║   travel × steer × camber × toe × track × ... ║
        ╚══════════════════════════════════════════════╝
                         ↓
        ╔══════════════════════════════════════════════╗
        ║  ░░ Runtime (universal lookup) ░░░░░░░░░░░░  ║
        ╠══════════════════════════════════════════════╣
        ║   create_lookup_kinematics(csv)               ║
        ║                ↓                              ║
        ║   attach_{front,rear}_kinematics(L3)          ║
        ║                ↓                              ║
        ║   L3.step() 가 매 substep:                    ║
        ║     - wheel_travel 계산                       ║
        ║     - lookup → camber + toe                   ║
        ║     - set_camber_per_wheel / set_toe_         ║
        ║                ↓                              ║
        ║   L2 의 PacejkaMF96 input.gamma + steer        ║
        ╚══════════════════════════════════════════════╝
```

이게 사용자가 원했던 "Adams Car 같은" workflow 의 골격.

## 4 종 suspension 의 diagnostic 결과

```
DW    front sports : ±80mm sweep all pass, 0.094°/mm camber gain
MP    front sedan  : OK, 0.019°/mm camber, strut tilt 19.4°
TA    rear  sedan  : OK, 0.022°/mm camber, semi-trailing 14° + anti-dive 5.5°
5-link rear sports : OK, 0.019°/mm camber, 0.026°/mm toe (sports-tight)
```

전부 wheel-z 정확도 < 1 μm, monotone 부드러운 곡선.

## 타이어 계층 (T25 + T26)

| 단계 | 추가된 기능 |
|---|---|
| Pacejka MF96 (베이스) | F = D · sin(C·atan(...)), friction ellipse combined |
| Load sensitivity | μ_eff(Fz) — 단위 하중당 grip 감소 |
| Relaxation length (transient α) | σ_y / |v| · α̇_dyn = α_geom − α_dyn |
| Camber thrust + camber Mz | Pacejka γ input 으로 Fy 및 Mz contribution |
| Ld2 / Ld3 에 모두 적용 | `set_camber_per_wheel`, `set_toe_per_wheel` interface |

## 점프 시뮬 계층 (T23 + T24)

| | 14-DOF 차이 | Pacejka |
|---|---|---|
| T23 | + world z, gravity, ramp ground, airborne 분기, droop/bump stop | linear cornering only |
| T24 | + planar (yaw, roll), 3D 시각화 | + 4-wheel Pacejka, transient α, load sensitivity |

## CARLA bridge 계층 (T22)

- 사용 model: Ld2 또는 Ld3 (L3 이면 hardpoint kinematics 가능)
- 실차 wheel 모델은 비활성 (`set_simulate_physics(False)`), VDSim 이 ego dynamics 단독 권한
- raycast 로 ground 4점 wheel contact 조회
- frame conversion: ISO 8855 ↔ UE4 LH (y, yaw 부호 반전)
- New: yaml 에 `load_sensitivity`, `relaxation_length_lat` 켜면 자동 활성
- New: BridgeConfig 에 `kinematics_{front,rear}_csv` 옵션 → L3 의 lookup attach

## Sign / 정합성 회귀

| 테스트 | 검증 항목 |
|---|---|
| `RollFeedsPerWheelCamberWithCorrectSign` | Ld3 의 phi → tire γ 부호 일관성 (left/right 대칭) |
| `KinematicsToeProducesBumpSteer` | lookup toe → Ld2 wheel steer 가산 + 슬립각 발생 |
| `AttachedKinematicsOverridesCamberPerRoll` | lookup 가 attach 되면 phenomenological 값 무시 |
| `WheelTravelSignConventionBumpIsPositive` | bump (chassis dive) = positive travel input |
| `LeftRightCamberSymmetry` | 같은 lookup, 좌/우 휠 mirror symmetry |

## 산출물 (브랜치 단위 신규)

### C++
- `core/include/vdsim/suspension.hpp` — ISuspensionKinematics
- `core/src/suspension_lookup.cpp` — bilinear lookup
- `core/src/pacejka_mf96.cpp` — load_sensitivity, camber Mz
- `core/src/seven_dof_dynamics.cpp` — transient α, set_{camber,toe}_per_wheel
- `core/src/bicycle_dynamics.cpp` — transient α (Ld1)
- `core/src/fourteen_dof_dynamics.cpp` — kinematics 사용, attach helpers
- 신규 tests: 9 (5 unit + 4 integration)
- **160/160 tests passing** (브랜치 시작 시 144)

### Python tools
- `tools/kinematics/double_wishbone.py` — 2D analyzer
- `tools/kinematics/dw_3d_solver.py` — DW 3D (true wheel-z Newton)
- `tools/kinematics/mp_3d_solver.py` — MacPherson (cylindrical joint)
- `tools/kinematics/ta_3d_solver.py` — trailing arm
- `tools/kinematics/fivelink_3d_solver.py` — 5-link general multilink
- `tools/kinematics/diagnose.py` — 통합 진단 도구
- `tools/kinematics/import_hardpoints.py` — Adams-style CSV → VDSim YAML

### Configs / docs
- `configs/suspensions/{dw,mp,ta,5link}*.yaml` — 4 sample geometries
- `configs/vehicles/sports.yaml` — roll_center_height_*, pitch_center_height 추가
- `configs/tires/default_pacejka.yaml` — load_sensitivity, relaxation_length_*

### Reports
- `docs/tasks/T22_carla_bridge/REPORT.md`
- `docs/tasks/T23_jump_14dof/REPORT.md`
- `docs/tasks/T27_ld4_dw/REPORT.md` (Stage A + 3D 결과)
- `docs/tasks/T28_ld4_mp/REPORT.md` (cylindrical 수정 포함)
- `docs/tasks/T29_ld4_ta/REPORT.md`
- `docs/tasks/T30_ld4_5link/REPORT.md`

## 의미 / 다음 방향

- **자율주행 도메인 통합**: CARLA 와 hardpoint kinematics 결합. 다른 simulator 들이 hardpoint design 을 지원 안 함 → 차별점.
- **확장 가능한 multibody framework**: 새 suspension type 은 offline solver + YAML schema 만 추가하면 됨. C++ runtime 무수정.
- **Open-core fit**: Adams Car 의 proprietary workflow 와 호환되는 hardpoint CSV → VDSim YAML 변환 가능 (`import_hardpoints.py`).

남은 후보:
1. **Ld5 compliance (부싱)** — 사용자 보류
2. **C++ side suspension type-dispatched native solver** — 현재 lookup 만 있지만, 향후 in-place solver 도 가능
3. **MBSym / SymPy 코드 생성**: 하드포인트 → symbolic Jacobian / constraint equations 자동 생성
4. **GUI editor**: hardpoint 시각 편집 + 라이브 sweep
