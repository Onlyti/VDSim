# T28 — Ld4 Stage E: MacPherson strut analyzer

## 목적

Ld4 의 두 번째 suspension type 추가. DW 와 schema / solver 패턴은 같지만
strut 의 **kinematic 모델이 본질적으로 다름** — strut tube 가 knuckle 에 강체
결합 + chassis 측에 spherical top mount + 내부 telescoping(spring 압축).

Stage E 의 핵심 검증:
1. YAML schema 가 type-dispatch 로 깨끗하게 분리됨 (`type: macpherson`)
2. C++ side 의 `ISuspensionKinematics` 인터페이스는 변경 없이 동작 — sweep CSV format 만 일치하면 됨
3. 다른 suspension type 도 같은 lookup 경로로 Ld3 에 attach 가능

## 구조적 차이 vs DW

| 항목 | Double wishbone | MacPherson |
|---|---|---|
| Upper anchor | UCA (chassis_front-rear axis) — UCA rotates about axis | Strut tube (강체 on knuckle) + chassis 측 spherical top mount |
| Upper-knuckle 운동 | UCA axis 주위 1-D arc | tube axis 가 chassis ST 통과해야 (cylindrical joint coaxial 제약) |
| Strut 길이 | (없음, 두 arm 의 rigid bar) | **가변** (spring telescoping) |
| Knuckle 자유도 닫힘 | UCA + tie rod = 0 free DOF | tube-axis cylindrical 제약 (2 scalar) + tie rod (1 scalar) = 0 free DOF |
| Solve | Sequential: LCA θ → UCA θ → trilaterate TK | Single LM (3 cross-product residuals + 1 tie rod, 1 redundant) |

## 첫 시도의 결함과 수정

초기 구현은 strut top 을 단일 ball-joint sphere 로 모델 (`|SK − ST| = L_strut`, 1 scalar). 이는 strut 의 길이만 고정하고 방향은 자유롭게 두는 model — 실차의 *cylindrical joint*(축 고정, 길이 가변) 와 정반대. 결과적으로 knuckle 의 1 rotational DOF 가 미닫혀 numerical 발산 / regularization hack 필요.

**올바른 제약**: tube 가 chassis 측 shaft 와 coaxial 이어야 하므로, *strut compression 길이는 자유* 이지만 *tube axis 의 line 은 chassis ST 를 통과* 해야 함.

Tube 축 방향은 body frame 에 고정 (`tube_axis_body = (ST_static − SK_static).normalized`). 회전 후:
```
SK_world  - ST_chassis  ∥  R @ tube_axis_body
   ⟺   (SK_world - ST_chassis) × (R @ tube_axis_body) = 0   (3 scalar, 1 redundant)
```

## 구현 (`tools/kinematics/mp_3d_solver.py`)

```python
def residuals(axis_angle_v):
    R = axis_angle_to_R(v)
    SK = LK + R @ off_SK
    TK = LK + R @ off_TK
    tube_axis_world = R @ tube_axis_body
    cross = np.cross(SK - ST_chassis, tube_axis_world)   # 3 scalars
    r_tie = |TK - TR_inner| - L_tr                       # 1 scalar
    return [cross[0], cross[1], cross[2], r_tie]
# 4 residuals, 3 R DOF, 1 dependency in cross — net 3 indep × 3 DOF = 0 free
```

Regularization 도 continuation 도 불필요 — 단발 LM 으로 unique solution 수렴.

## 검증

### 정적 (travel=0, steer=0)

| 항목 | 결과 |
|---|---|
| camber | 0.000° |
| toe | 0.000° |
| track_change | 0.000° |
| caster | 0.000° (hardpoint 의 ST/SK 둘 다 x=−0.04 라 strut 가 y-z 평면) |

### 17×5 (travel × steer) sweep

- `sweep_3d.csv` — 85 points, all valid
- `sweep_3d.png` — 4 panel plot
- 곡선 부드럽고 monotone, **단발 LM** 으로 수렴 (continuation 불필요)

| 메트릭 | 값 (수정 후) |
|---|---|
| Camber gain | **0.034 °/mm** — 실차 sedan MacPherson typical (0.02–0.06 °/mm) ✓ |
| Bump steer (toe gain) | ~0.02 °/mm — kinematic bump steer 의 정상치 |
| Track narrowing | ~0.15 mm/mm — 일반적 |
| Strut compression | ≈ 1:1 with wheel travel (LCA 가 거의 수평일 때 예상) |
| Caster (이 sample) | 0° (ST/SK 모두 x=−0.04 → 측면 정렬) |

### C++ lookup 호환

- Same CSV schema as DW (`wheel_travel, steer_rack_dy, camber, toe, track_change, caster, valid`)
- `vdsim.create_lookup_kinematics(csv)` 가 그대로 로드
- `vdsim.attach_front_kinematics(dyn, csv)` 가 Ld3 에 attach OK

## 의미

C++ side 는 **suspension type 에 종속되지 않음** — offline solver 만 type별로
구현하면 됨. 향후 5-link, trailing arm, twist beam 등도 같은 패턴으로 추가.
이게 사용자가 원하는 "Adams Car 같은" workflow 의 기본 구조:

```
YAML hardpoints (type별 schema)
    ↓ offline solver (type별)
sweep CSV (universal format)
    ↓ create_lookup_kinematics (C++)
ISuspensionKinematics (type-agnostic runtime)
    ↓ attach to Ld3
실시간 시뮬레이션
```

## 다음 단계

1. 5-link rear analyzer (가장 복잡, 5개 link 의 5개 length constraints)
2. Trailing arm rear (2-3 hardpoints 만 — 가장 단순한 multilink)
3. Twist beam rear (axle-coupled, special handling)
4. Adams-compatible YAML import (Adams Car 의 hardpoint 파일 포맷)
5. 본격 multibody compliance (Ld5) — 부싱 stiffness, force-dependent deflection
