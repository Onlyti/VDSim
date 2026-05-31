# T28 — Ld4 Stage E: MacPherson strut analyzer

## 목적

Ld4 의 두 번째 suspension type 추가. DW 와 schema / solver 패턴은 같지만
**strut top 이 단일 ball joint (UCA 처럼 axis 없음)** 이라 구조적으로 다름.

Stage E 의 핵심 검증:
1. YAML schema 가 type-dispatch 로 깨끗하게 분리됨 (`type: macpherson`)
2. C++ side 의 `ISuspensionKinematics` 인터페이스는 변경 없이 동작 — sweep CSV format 만 일치하면 됨
3. 다른 suspension type 도 같은 lookup 경로로 Ld3 에 attach 가능

## 구조적 차이 vs DW

| 항목 | Double wishbone | MacPherson |
|---|---|---|
| Upper anchor | UCA (axis: chassis_front-rear) → UCA rotates about axis | Strut top (single ball joint) → 자유 회전 |
| Upper-knuckle 운동 | Circle around UCA axis | Sphere around strut top |
| 모르는 DOF | 0 (모두 결정) | 1 (knuckle 의 kingpin axis 회전 — under-determined) |
| Solve | Sequential: LCA θ → UCA θ → trilaterate TK | least_squares with regularization + continuation |

## 구현 (`tools/kinematics/mp_3d_solver.py`)

```python
def residuals(axis_angle_vec):
    R = axis_angle_to_R(v)
    SK = LK + R @ off_SK
    TK = LK + R @ off_TK
    return [
        |SK - ST| - L_strut,            # hard constraint 1
        |TK - TR_inner| - L_tr,         # hard constraint 2
        0.005 * |v|,                    # regularization (prefer small rotation)
    ]

# scipy.optimize.least_squares (Levenberg-Marquardt)
# Continuation: each (travel, steer) uses nearest already-solved point as x0
```

Regularization weight `0.005` 가 핵심: 너무 작으면 spurious far-flip basin
으로 수렴, 너무 크면 constraint 무시. 0.005 + continuation = monotone 부드러운
sweep.

## 검증

### 정적 (travel=0, steer=0)

| 항목 | 결과 |
|---|---|
| camber | 0.000° |
| toe | 0.000° |
| track_change | 0.000° |
| caster | 0.000° (hardpoint 의 ST/SK 둘 다 x=−0.04 라 strut 가 y-z 평면) |

### 11×5 (travel × steer) sweep

- `docs/tasks/T28_ld4_mp/run01/sweep_3d.csv` — 55 points, all valid
- `docs/tasks/T28_ld4_mp/run01/sweep_3d.png` — 4 panel plot
- 곡선은 부드럽고 monotone — solver 안정성 검증 OK
- Magnitude 는 sample hardpoint 가 IC 를 wheel 에 가까이 두어 큼
  (sample 의 IC y ≈ wheel y − 0.02 m → camber gain ~1°/mm)
- 실제 sedan 의 IC 는 wheel y − 0.6 m 정도 inboard → 0.05°/mm

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
