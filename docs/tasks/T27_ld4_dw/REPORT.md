# T27 — Ld4 Stage A: Double-wishbone hardpoint analyzer

## 목적

VDSim 의 최종 목표는 **하드포인트 + 설계변수 기반 다물체 동역학**(Adams Car
등가). Ld3 까지는 `camber_per_roll`, `roll_center_height_front/rear` 같은
phenomenological scalar 로 처리했지만, 본 task 부터는 **suspension 의
기하학적 정의**(하드포인트) 로부터 이 값들을 *유도* 한다.

Stage A 의 범위:
- YAML 하드포인트 포맷 정의 (DW front 부터)
- 측면(y-z) 2D 운동학 분석기 — IC, RC, camber gain, track change 출력
- Wheel-travel sweep 의 closed-form 2-bar linkage 풀이 (camber, track 곡선)
- 선형 (IC method) vs 비선형 (closed-form) cross-check

Stage B 이후 (별도 task):
- 3D 운동학 nonlinear solver (steering 포함)
- C++ `ISuspensionKinematics` interface + lookup table runtime
- Ld3 에 wire — `camber_per_roll` deprecated, hardpoint-derived 값 사용

## 구현

### 하드포인트 YAML (`configs/suspensions/dw_front_sports.yaml`)

Body frame ISO 8855 RH, 모든 거리 [m].  Left wheel only — right wheel mirror.

```yaml
type: double_wishbone
side: left
wheel:
  center:        [0.00, 0.79, 0.330]
  static_radius: 0.330
lca:
  chassis_front: [+0.15, 0.30, 0.08]
  chassis_rear:  [-0.18, 0.30, 0.08]
  knuckle:       [-0.02, 0.75, 0.15]
uca:
  chassis_front: [+0.08, 0.45, 0.32]
  chassis_rear:  [-0.22, 0.45, 0.32]
  knuckle:       [-0.01, 0.70, 0.55]
tie_rod:
  rack:    [-0.30, 0.35, 0.18]
  knuckle: [-0.30, 0.74, 0.21]
spring_damper:
  chassis: [-0.05, 0.45, 0.50]
  lca:     [-0.05, 0.55, 0.13]
```

### 분석기 (`tools/kinematics/double_wishbone.py`)

| 메서드 | 출력 | 식 |
|---|---|---|
| `instant_center()` | LCA-line ∩ UCA-line in y-z | 2D 선 교점 |
| `roll_center_height()` | (tire contact → IC) ∩ (y=0) | linear extrapolation |
| `camber_gain()` | dγ/dz at static, 선형 근사 | −sign / (y_wheel − y_IC) |
| `track_change_gain()` | dy_wheel/dz | −dz_IC / dy_IC |
| `sweep(travel, n)` | 비선형 wheel-travel 응답 | LCA θ_l 직접 적분 + UCA 의 closed-form 2-원 교점 + knuckle rigid attach |

`sweep()` 의 closed-form:
1. Wheel z 목표 → LCA-knuckle z 역계산 → `asin` 으로 θ_l 결정
2. UCA-knuckle 위치 = (UCA-chassis 중심 반경 L_uca) ∩ (LCA-knuckle 중심 반경 L_knuckle) — 두 해 중 정적 위치에 가까운 것 선택
3. Knuckle (UCA-LCA 연결) 각도 변화로 wheel center / camber 갱신

## 검증

### Sports trim 입력 → 산출 (정적)

| 메트릭 | 값 |
|---|---|
| Instantaneous center (y, z) | (0.167, 0.059) m — wheel inboard, slightly above ground |
| Roll center height | **0.075 m** (지면 위 7.5 cm) — sports car typical 5-15 cm |
| Camber gain (선형) | **−0.0919 °/mm** — 10 mm bump → 0.92° 음각 (top-of-wheel 내측, mild recovery) |
| Camber gain (비선형, z=0 근방 FD) | **−0.0982 °/mm** — 7% 차이 (2차 항 포함) |
| Track change | **−0.434 mm/mm** — bump 시 track narrows |

### Cross-check

선형 IC method vs 비선형 2-bar solver 의 z=0 근방 기울기가 7% 이내로 일치 →
구현 정합성 OK. ±80 mm 범위에서 camber 곡선이 비선형 (단조 증가-감소 휘어짐).
이게 Ld3 의 `camber_per_roll` 스칼라가 캡처 못 하는 부분 — Stage C 의
lookup table 이 표현해야 할 본질.

## 산출물

| File | 용도 |
|---|---|
| `configs/suspensions/dw_front_sports.yaml` | 하드포인트 입력 |
| `tools/kinematics/double_wishbone.py` | 분석기 + sweep + plot |
| `docs/tasks/T27_ld4_dw/run01/side_view.png` | 측면도 (LCA/UCA/IC/RC) |
| `docs/tasks/T27_ld4_dw/run01/travel_sweep.png` | camber/track vs travel |
| `docs/tasks/T27_ld4_dw/run01/travel_sweep.csv` | lookup table (Stage C 입력) |
| `docs/tasks/T27_ld4_dw/run01/summary.yaml` | 정적 산출치 |

## 한계 (Stage A 의 범위)

1. **2D only** — y-z plane. Caster (x 방향 tilt), kingpin axis 의 3D 변화 무시
2. **Steering 무시** — tie rod 동작 안함, toe angle 안 나옴
3. **Compliance 없음** — 부싱이 강체 (Stage A 정의)
4. **Ride only** — heave / single-wheel bump 만. roll 동작 시 양쪽 wheel 의 의존성 별도 처리 필요
5. **Static (kingpin angle ≈ 90°)** 가정 — 실차의 kingpin inclination 영향 일부 누락

## 다음 단계

| | Stage | 범위 |
|---|---|---|
| B | 3D nonlinear solver — steering, caster, kingpin inclination, toe gain 포함 | tie rod 모델, scipy.optimize 또는 직접 Newton |
| C | C++ `ISuspensionKinematics` interface, lookup table runtime (cubic spline) | `core/include/vdsim/suspension.hpp`, `core/src/suspension_lookup.cpp` |
| D | Ld3 wiring — `camber_per_roll` deprecate, per-wheel suspension travel 로 인용 lookup | `fourteen_dof_dynamics.cpp` 의 camber 계산 경로 교체 |
| E | Compliance: 부싱 stiffness 추가 → wheel pose 의 force-dependent perturbation | Ld5-MultibodyCompliant |
| F | MacPherson, 5-link, multi-link 추가 (`type` 디스패치) | 별도 yaml schema |
