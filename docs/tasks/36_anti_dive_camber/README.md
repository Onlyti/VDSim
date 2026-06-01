# Task 36-37 — Anti-dive geometry + Camber-from-roll + MF96 camber thrust

| Field | Value |
|---|---|
| Task ID | IM-W11-3 (cluster) |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

L3 의 sprung dynamics (Task 30) 위에 두 가지 사실적 보완 추가:

1. **Anti-dive / anti-squat** — 실차의 suspension geometry 가 brake 시 pitch (nose-dive) 와 accel 시 squat 을 부분 흡수.
2. **Camber thrust** — wheel camber 가 lateral force 에 추가 기여. Roll-induced camber 의 효과는 본 task scope (실제 wheel kinematics 통합) 까지 안 가고 TireParams + MF96 확장만.

## 2. 구현 방법

### 2.1 코드 변경

| 위치 | 변경 |
|---|---|
| `core/include/vdsim/params.hpp` | VehicleParams: `anti_dive_front`, `anti_squat_rear`, `camber_per_roll`. TireParams: `camber_stiffness` |
| `core/src/params.cpp` | YAML I/O 4 새 필드 (default 0, backward-compat) |
| `core/include/vdsim/interfaces.hpp` | `ITireModel::Input` 에 `gamma` (camber) 추가 |
| `core/src/pacejka_mf96.cpp` | Fy_camber = -C_gamma · γ · Fz · μ_lat 추가 |
| `core/src/seven_dof_dynamics.cpp` | tire input 에 `in.gamma = 0` 명시 (L2 는 camber 미반영) |
| `core/src/fourteen_dof_dynamics.cpp` | pitch derivative 에 `(1 - anti)` 보정 |
| `tests/unit/test_tire_models.cpp` | 2 새 test (camber 효과 / default 0) |
| `tests/integration/test_fourteen_dof.cpp` | 1 새 test (anti-dive 가 pitch 감소) |

### 2.2 식

**Anti-dive (pitch derivative)**:
```
fraction = (ax < 0) ? anti_dive_front : anti_squat_rear   (clamped [0,1])
M_pitch_inertial = m_s · ax · h_cg · (1 - fraction)
```

`anti_dive_front = 0`: 기존 동작 (모든 pitch moment 전달).
`anti_dive_front = 1`: brake 시 pitch = 0 (full anti-dive).

**Camber thrust**:
```
Fy_camber = -C_gamma · γ · Fz · μ_lat
Fy_total  = Fy_pacejka_lat + Fy_camber
```

C_gamma 는 TireParams.camber_stiffness, default 0 (no effect). 부호: `γ > 0` (wheel top leans toward +y) → 차량을 +y 방향으로 추가 force. ISO 8855 RH 에서 `+y` 좌측 → wheel-frame Fy < 0 (camber thrust 와 sign convention 일치 필요), 사실은 textbook varies; default = 0 으로 보수적.

### 2.3 한계

- **L2 의 camber 입력 = 0** — wheel kinematics 통합 미반영. L3 + roll-aware seven_dof 후속.
- **`camber_per_roll`** 정의됐지만 사용처 없음 — 추후 L3 → camber 통합 시 활용.
- **Anti-dive 의 anti-lift (rear under accel)** 별도 모드 미반영.
- **Camber stiffness 의 Fz 의존** 무시 — linear approximation.
- **Camber from L3 roll** 통합 — Task 30 의 phi 값을 wheel-level gamma 로 매핑 필요. 본 task 미반영.

## 3. 검증

### 3.1 3 새 test

| Test | 항목 | Pass 기준 |
|---|---|---|
| PacejkaCombinedFixture.CamberAddsLateralForce | γ=0.05, C_gamma=1.5, Fz=4000 | Fy ≈ -1.5·0.05·4000 = -300 N |
| PacejkaCombinedFixture.CamberZeroByDefault | C_gamma=0, γ=0.10 | Fy 변화 없음 |
| FourteenDOF.AntiDiveReducesPitchUnderBrake | anti_dive 0 vs 0.5 | \|pitch_anti\| < \|pitch_base\| |

전체 127/127 통과 (이전 124 + 본 cluster 3 new).

### 3.2 Backward compat

- Camber default 0 → 기존 tire test 9개 + L1/L2 tests 전부 동일.
- Anti-dive default 0 → L3 pitch derivative 결과 동일.

## 4. 판단

- 결과: **pass** (anti-dive geometry + tire camber API + MF96 camber thrust)
- 근거:
  - 3/3 새 test 통과, 누적 127/127.
  - Anti-dive 가 정성적 pitch 감소 효과 측정.
  - Camber thrust 가 linear region 에서 정량 일치.
- 미해결 / Follow-up:
  - **Unsprung mass 4 DOF (Task 35)** — full 14-DOF 의 마지막 4 DOF. **Deferred** (3 시간 budget 외).
  - **Camber-from-roll wiring** — L3 의 roll φ → per-wheel γ_i. 다음 cycle.
  - **Camber-Mz coupling** — Mz 의 camber term 통합.
