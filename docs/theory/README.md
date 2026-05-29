# VDSim — Theoretical Reference

본 문서는 VDSim 구현의 **이론적 배경** 을 강의 노트 / 교과서 수준으로 정리한 자료다.
PhD 수준 독자가 읽고 — 식의 출처, 가정, 한계, 코드 매핑까지 모두 — 한 번 더 누군가에게 설명할 수 있는 깊이를 목표로 한다.

## 읽는 순서

| # | 챕터 | 길이 | 선수 지식 |
|---|---|---|---|
| 01 | [좌표계 / 부호 약속](01_frames_and_conventions.md) | 짧음 | 선형대수 |
| 02 | [강체 동역학 (Newton-Euler)](02_rigid_body_dynamics.md) | 중 | 01 |
| 03 | [타이어 모델 — Pacejka MF96](03_tire_pacejka_mf96.md) | 길음 | 01, 02 |
| 04 | [Ld1-Bicycle (5 DOF)](04_ld1_bicycle.md) | 중 | 02, 03 |
| 05 | [Ld2-SevenDOF (per-tire + Ackerman + diff)](05_ld2_seven_dof.md) | 중 | 04 |
| 06 | [Ld3-FourteenDOF (sprung 3 + unsprung 4)](06_ld3_fourteen_dof.md) | 길음 | 02, 04 |
| 07 | [Control ladder Lc1-Lc8 + variant dispatch](07_control_ladder.md) | 짧음 | — |
| 08 | [Lc5-AxTarget / Lc6-VTarget PID](08_pid_controllers.md) | 중 | 기초 제어 |
| 09 | [Lc7-PurePursuit / Lc8-Waypoint](09_pure_pursuit_path.md) | 중 | 기하 |
| 10 | [Driver model (latency + noise)](10_driver_model.md) | 짧음 | 09 |
| 11 | [수치 적분 (RK4, substepping, 1-step lag)](11_numerical_integration.md) | 중 | ODE 기초 |
| 12 | [소프트웨어 아키텍처 (C++ ABI, pybind11, CARLA)](12_software_architecture.md) | 중 | C++17, 모듈러 |
| 13 | [Multibody outlook (Ld4-Kinematic / Ld5-Compliant)](13_multibody_outlook.md) | 길음 | 02, 06 |

권고: 01 → 02 → 03 → 04 순으로 시작. 그 이후는 관심 영역에 따라.

## 표기 / 약자 (전체에 공통)

### 좌표 / 부호
- **ISO 8855 RH (right-handed)** — body frame: x forward, y leftward, z up.
- **World frame** — ENU (East-North-Up) RH.
- **Quaternion** — body → world (Eigen convention, `Quat::Identity()` = no rotation).
- **Euler** — ZYX intrinsic (yaw → pitch → roll).
- **Wheel index**: FL = 0, FR = 1, RL = 2, RR = 3.

### 부호 — 자주 헷갈리는 점
- ISO 8855 RH 에서 좌선회 (yaw rate r > 0) 시 alpha > 0 → Fy < 0 (restoring).
  *Rajamani 등 SAE Y-right 책의 `delta - atan(...)` 형식과 부호가 다르다.*
- Roll φ > 0 — 차체의 right side 가 아래로 내려가는 방향 (SAE 호환).
- Pitch θ > 0 — nose-up. brake 시 nose-dive → θ < 0.

### Greek / 변수 약속 (수식 안에서)
- `m`, `m_s`, `m_u` — total mass, sprung mass, unsprung mass per corner.
- `Izz`, `I_xx`, `I_yy` — sprung body inertia diagonal.
- `L`, `a`, `b` — wheelbase, CG-to-front-axle, CG-to-rear-axle. `L = a + b`.
- `Tw_f`, `Tw_r` — track width front / rear.
- `h_cg` — CG height above ground.
- `R` — wheel radius (nominal, kinematic).
- `vx`, `vy`, `r` — body-frame velocity x/y + yaw rate.
- `delta` (δ) — front-wheel steer angle [rad].
- `alpha` (α) — slip angle [rad]. `kappa` (κ) — slip ratio.
- `Fz`, `Fx`, `Fy`, `Mz` — tire normal / longitudinal / lateral force, aligning moment.
- `mu` (μ) — friction coefficient. `mu_long`, `mu_lat` per-axis scaling.

### 약자 (전체 문서에서 expand 하지 않음)
- **TUR** — Hyundai 산학 (Fz/μ/Cα UKF 추정).
- **NHCalib** — jerk-aware non-holonomic calibration paper.
- **SMPC** — Stochastic Model Predictive Control.
- **HPIPM** — High-Performance Interior Point Method (MPC QP solver).
- **UKF** — Unscented Kalman Filter.
- **FSK** — Formula SAE Korea.
- **MF96** — Pacejka Magic Formula 1996 simple form.
- **ERG** — CarMaker result file.

## 핵심 참고 문헌 (이 시리즈가 reference 하는 책 / paper)

1. **Genta, G.** *Motor Vehicle Dynamics: Modeling and Simulation*, World Scientific, 2014. 차량 동역학 표준.
2. **Rajamani, R.** *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. 단, **SAE Y-right convention** 임에 주의 (alpha 부호 반대).
3. **Pacejka, H.B.** *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012. MF96 / MF2002 정본.
4. **Milliken, W.F. & Milliken, D.L.** *Race Car Vehicle Dynamics*, SAE International, 1995. weight transfer / Ackerman / diff 직관.
5. **Reimpell, J., Stoll, H., Betzler, J.** *The Automotive Chassis: Engineering Principles*, Butterworth-Heinemann, 2001. suspension geometry / K&C.
6. **Featherstone, R.** *Rigid Body Dynamics Algorithms*, Springer, 2008. multibody (Ld4-Ld5).
7. **Thrun, S., Burgard, W., Fox, D.** *Probabilistic Robotics*, MIT Press, 2005. R / Q noise convention (TUR / NHCalib 측 reference).

## VDSim 코드 매핑 (이 문서에서 자주 등장)

| 파일 | 역할 |
|---|---|
| `core/include/vdsim/types.hpp` | Eigen typedef + wheel index constants |
| `core/include/vdsim/state.hpp` | State struct (모든 사다리 cover) |
| `core/include/vdsim/params.hpp` | VehicleParams / TireParams / SolverParams |
| `core/include/vdsim/interfaces.hpp` | IVehicleDynamics + ITireModel + IContactProvider |
| `core/include/vdsim/control.hpp` | ControlInput = variant<CmdL1..L8> |
| `core/include/vdsim/control_converter.hpp` | LongAxController / LongVxController / PurePursuit / DriverModel |
| `core/include/vdsim/multibody.hpp` | Ld4-Ld5 의 hardpoint / joint / bushing struct (M0 stub) |
| `core/src/pacejka_mf96.cpp` | MF96 + combined slip + Mz + camber |
| `core/src/bicycle_dynamics.cpp` | Ld1 본체 |
| `core/src/seven_dof_dynamics.cpp` | Ld2 본체 |
| `core/src/fourteen_dof_dynamics.cpp` | Ld3 본체 |
| `core/src/control_converter.cpp` | Lc5/Lc6/Lc7/Driver |

## 문서 진척

| 상태 | 챕터 |
|---|---|
| **draft 1차 완성** | 01, 02, 03, 04, 05, 06, 07 |
| 작성 예정 | 08, 09, 10, 11, 12, 13 |

이 README + 01-07 챕터 정독 시 VDSim 의 **simulator-side full stack** 의 이론적 배경 90% 가 cover 된다.
나머지 (제어 디테일, 적분기, 아키텍처, multibody) 는 후속 챕터.
