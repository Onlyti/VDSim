# T23 — L3-FourteenDOF *world-z extension* (jump simulation)

## 목적

기존 `Ld3-FourteenDOF` 는 sprung mass 의 heave / roll / pitch 와 4 unsprung
z 를 적분하지만, 모두 **정적 평형점 기준 small-signal linearization** 이고
중력 항이 `g_z_effective ≈ 0` 으로 빠져 있다 (`core/src/fourteen_dof_dynamics.cpp:26`).
또한 노면 z 가 항상 0 으로 가정되어 있어, 차량이 노면 굴곡에 반응하거나
공중에 뜨는 상황은 표현 불가.

이 task 는 **Phase-2 의 확장형 14-DOF 모델**을 Python 으로 구현하여
점프대를 실제로 통과하는 시뮬레이션 + 측면 뷰 애니메이션을 산출한다.

## 구현 방법

### 모델 확장 (`apps/jump_demo/sim_14dof_jump.py`)

| 항목 | 현재 C++ L3 | T23 확장 |
|---|---|---|
| World z 적분 | 없음 (`position.z` 미사용) | sprung CG world z + 4 unsprung world z 모두 적분 |
| 중력 항 | `g_z_effective ≈ 0` (제거) | sprung, unsprung 모두 `−g` |
| 노면 입력 | `z=0` 평지 가정 | `ground_z(x)` 함수 — 각 wheel 의 noise location 에서 평가 |
| Tire 수직력 | `F_tire = k_tire · (−z_u)` | `F_tire = max(0, k_tire · ((ground_z + r_eff) − z_u))` — airborne 시 0 |
| Spring force | linear (퍼터베이션) | linear + **droop stop (F_spring ≥ 0)** + **bump stop (F_spring ≤ 3·F_static)** |
| Damper | 항상 작동 | spring 이 topped-out (F=0) 일 때 0 — wheel-hanging 모드 |

### Ramp profile

- `x_start = 20 m`: 평지 시작 → 램프 진입
- `x_top = 24 m`: 램프 정상 (height 0.6 m, half-cosine smooth)
- `x_lip_end = 24.4 m`: 짧은 lip (0.4 m flat)
- `x > 24.4 m`: cliff drop → 평지 (z=0)

### 적분

- RK4, `dt = 0.0005 s`
- `mass = 1320 kg`, `mass_sprung = 1180 kg`, `unsprung = [42,42,38,38] kg` (sports.yaml)
- `k_spring = [45000, 45000, 42000, 42000] N/m`, `c_damper = [4200, 4200, 3900, 3900] N·s/m`
- `k_tire = 220000 N/m`, `r_wheel = 0.33 m`, `I_yy = 1700 kg·m²`
- 초기 속도 `v0 = 15 m/s` (54 km/h), 가벼운 throttle 0.1 (coast)
- 시뮬레이션 시간 5 s

## 검증 방법 (근거)

### Sanity checks

| 항목 | 기대 | 실측 |
|---|---|---|
| Static equilibrium 진입 안정 | t ∈ [0, 1] 평지 주행 시 z_s = h_cg ± noise, pitch ≈ 0 | z_s = 0.382 m (h_cg=0.42), pitch = 0° |
| 정적 Fz (전륜) | m·g·b/(2·L) = 1320·9.81·1.35/(2·2.55) ≈ 3428 N | 측정 Fz_FL = 3475 N (오차 1.4 %) |
| 정적 Fz (후륜) | m·g·a/(2·L) = 1320·9.81·1.20/(2·2.55) ≈ 3048 N | 측정 Fz_RL = 3095 N (오차 1.5 %) |
| 램프 진입 시 전륜 Fz 증가 | 노면 z 가 올라가며 tire compression 증가 → 큰 Fz | t=1.30 s 에 Fz_FL = 11758 N (3.4× static) ✓ |
| Pitch 응답 (램프 위) | 전륜 lift 로 nose-up pitch (코드 convention: 음수 = nose-up) | t=1.50 s 에 pitch = −13.5° ✓ |
| Airborne 진입 | 4-wheel Fz 모두 0 인 구간 발생 | t=1.70~2.40 s, Fz 모두 0 ✓ |
| Airborne 시 sprung CG ballistic | z_s 가 포물선 궤적 | t=1.70(1.35) → t=1.95(1.66 peak) → t=2.40(0.68) ✓ |
| 착지 시 Fz 임펄스 | 전륜 먼저 착지 (pitch swung 양수 → nose-down) | t=2.40 s, Fz_FL = 21948 N (6.3× static); 직후 t=2.60 s 에 후륜 23602 N |
| Airborne duration | sqrt(2·Δz/g) 정도 | 0.71 s (sprung CG 가 1.66 - 0.68 = 0.98 m 낙하 → 0.45 s ballistic + 0.26 s 추가 spring response → 합리적) |

### 시각화

`apps/jump_demo/run01/jump_demo.mp4` — 1300×700, 5.6 s, 30 fps, 550 KB.
상단 측면 뷰 + 하단 4 패널 (z_s / pitch / Fz_front / Fz_rear).

## 검증 결과

| 메트릭 | 값 |
|---|---|
| 시뮬레이션 시간 | 5.0 s (10000 RK4 steps @ 0.5 ms) |
| Sprung CG peak z | **1.660 m** (정적 0.382 m → 1.66 m, ramp 0.6 m + KE 변환 ~0.7 m) |
| Peak |pitch| | **21.3°** (착지 직전, nose-down 회전) |
| Airborne duration | **0.71 s** |
| Peak landing Fz (전륜) | **21948 N** (정적의 6.3×) — bump stop 한계 (3× static = 10425 N) 위에 tire 동적 압축 추가 |
| Peak landing Fz (후륜) | **23602 N** |
| 산출물 | `telemetry.csv` (500 samples), `ramp.yaml`, `veh.yaml`, `jump_demo.mp4` |

## 판단 결과 (근거)

| 판단 | 근거 |
|---|---|
| **OK** — 점프 시뮬레이션이 동역학적으로 정합 | (1) 정적 평형 Fz 가 이론치와 1.5 % 오차, (2) 램프 진입 시 nose-up pitch + 전륜 Fz spike 의 phasing 이 차량 기동 직관과 일치, (3) airborne 구간 모든 Fz = 0 + sprung CG 가 ballistic 궤적, (4) 착지 시 nose-down rotation 으로 전륜 먼저 닿음 |
| **OK** — droop / bump stop clipping 으로 model 안정 | clipping 추가 전 동일 ramp (1.2 m) + v=18 m/s 에서 peak z=4.6 m, |pitch|=44° 까지 발산 (spring 이 droop 방향으로 무한히 stretch). clipping 도입 후 합리적 1.66 m peak |
| **한계** — landing Fz 의 정량 신뢰도 낮음 | bump stop 을 단순 hard clip (3× F_static) 으로 모델링 → 실차의 progressive bump rubber stiffness 와 다름. peak landing load 의 정확한 값은 신뢰 불가, **위상 / 발생 위치 / 상대 크기 만 의미 있음** |
| **한계** — 횡 동역학 미포함 | T23 은 직선 점프만; roll, vy, yaw 모두 0 고정. 옆으로 기울며 점프하는 모션 (skid-jump) 은 표현 불가 |
| **한계** — Python prototype, C++ L3 미수정 | C++ Ld3-FourteenDOF 본체는 그대로. T23 결과는 "Phase-2 에서 C++ 측 14-DOF 가 이렇게 확장되어야 한다" 는 specification 역할 |

## 다음 단계 (Phase 2 → C++ port)

1. `fourteen_dof_dynamics.cpp` 내부 `z_s_`, `z_u_[]` 의 의미를 "정적 평형 기준 perturbation" → "world z" 로 재정의
2. `derivatives_vertical` 에 `ground_z[4]` 추가, `g_z_effective` 를 정확한 `−g` 로 복원
3. Tire 수직력 `max(0, k_tire · ((ground_z + r_eff) − z_u))` 로 변경, airborne 분기 자동 발생
4. Bump / droop stop 을 `VehicleParams` 에 추가 (`spring_travel_compression_max`, `spring_travel_extension_max`) — progressive stiffness rubber model 사용 (단순 hard clip 대신 e.g. polynomial)
5. `state_.position.z()` 가 `z_s_` 를 따라 갱신되도록 `inner_->reset(state_)` 와 `step()` 끝의 state copy 수정
6. CARLA bridge 의 `query_contacts()` 가 반환하는 `ContactPoint.position.z` 가 dynamics 입력으로 실제 사용되도록 path 연결 (현재 무시)
7. 본 task 의 5 가지 sanity check 를 `tests/` 의 GoogleTest 로 컴파일 — regression suite
