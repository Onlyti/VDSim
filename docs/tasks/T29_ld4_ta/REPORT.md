# T29 — Ld4 trailing arm rear suspension

## 목적

Ld4 framework 의 **3번째 suspension type** 추가. 기계적으로 가장 단순한
multilink (single rigid arm rotating about one chassis axis).
DW/MP 와 동일 universal CSV schema 로 C++ ISuspensionKinematics 가 변경 없이
로드 가능 — framework 의 type-agnostic runtime 을 추가 검증.

## 모델

- Body = `[arm + knuckle + wheel]` 1개 강체
- Joint = chassis revolute axis (1 DOF)
- Steering 없음 (rear)
- Net 1 DOF (wheel travel)

```
M = 6·(N − 1) − Σc = 6·1 − 5 = 1 DOF (= wheel travel)
```

## Hardpoints (`configs/suspensions/ta_rear_sedan.yaml`)

```yaml
type: trailing_arm
side: left
wheel:
  center: [-1.35, 0.78, 0.305]
  static_radius: 0.305
arm_pivot:
  chassis_inboard:  [-0.80, 0.42, 0.22]
  chassis_outboard: [-0.75, 0.62, 0.20]
```

`arm_pivot` 두 점의 차이가 axis 방향. 차이가 +y 방향 정확히면 pure trailing
(camber/toe gain 0). y 성분 외에 x 또는 z 성분 있으면 semi-trailing.

Sample 의 axis = `[0.241, 0.966, -0.097]` →
- semi-trailing tilt z (toe direction): +14°
- anti-dive tilt x (pitch direction): -5.5°

## 검증 (diagnostic 출력)

```
Solver success rate     : 17 / 17
Max wheel-z error       : 0.077 μm
Camber monotone in travel: True
Camber gain (avg)       : -0.0224 °/mm
Toe gain (avg)          : 0.0090 °/mm
Track gain (avg)        : 0.0524 mm/mm
```

- ±80mm sweep 전체 통과
- camber/toe 부드러운 monotone
- sub-μm 정확도

## 한계 / 다음 단계

- 단일 강체 arm 가정 — 실차에선 부싱 (compliance) 가 있어 약간의 lateral
  compliance 존재. Phase 2 (Ld5) 에서 추가 가능
- 반-trailing 의 anti-dive geometry 가 anti-squat 으로 작용 — 가속 시 squat 가
  감소되는 효과. 현재는 kinematic 만 다루고 dynamics 의 anti-squat 효과는
  L3 의 anti-squat_rear 파라미터에 phenomenological 로 들어감
