# VDSim — Theoretical Reference

이 시리즈는 차량 동역학 / 제어 / 수치 적분 / 소프트웨어 아키텍처를
**강의 노트 수준의 textbook** 으로 정리한 reference 다. 각 chapter 는
self-contained — 모델의 동기, 가정, 유도, 검증, 한계를 본문에서 한 번
완결한다. VDSim 의 구현은 별도 *implementation note* 박스로 분리되어,
독자는 두 가지 깊이 중 본인 목적에 맞는 trail 만 따라갈 수 있다.

대상 독자:

- 차량 동역학 / 자율주행 제어를 처음 / 두 번째 배우는 PhD/MS 수준 학습자
- VDSim 의 API / 채택 / 한계를 파악하려는 사용자 (산업 / 연구)

---

## 두 가지 trail — 본문과 구현 노트

각 chapter 는 다음 두 layer 로 구성된다.

### A. 본문 — 교과서 (모델 / 식 / 검증)

차량 동역학 일반 reference 로 읽힌다. VDSim 의 특정 채택과 무관한, 모델
자체의 derivation 과 limitation 을 다룬다.

표준 sequence:

```
0. Learning objectives        (이 chapter 를 마치면 ... 할 수 있다)
0. Prerequisites              (이전 chapter / 외부 reference)
1. 동기                       (왜 이 모델이 필요한가)
2. 가정                       (명시적 list)
3. 유도                       (식 + 직관 + 단위 + 부호)
4. 검증                       (analytical + numerical)
5. 한계                       (어떤 현상을 다루지 못하는가)
6. 다음 chapter 와의 연결
7. 참고문헌
8. Self-check (3-5 문제)
```

### B. VDSim 구현 노트 — *implementation note* 박스

VDSim 의 구체 채택, 코드 경로, 검증 Task 번호, 사용 패턴 (C++ / Python /
YAML). 본문 의존 없이 skim 가능하도록 단독 박스로 분리한다. 박스 양식:

```
> **[VDSim impl] § X.Y**
>
> - 코드: core/src/bicycle_dynamics.cpp:120-127
> - 검증: Task 17 (RK4 substep + 1-step lag)
> - 사용: 아래 minimal example 참조.
```

학습 목적이면 본문만 — VDSim 코드를 보지 않아도 self-contained. VDSim 사용자는
본문을 가볍게 훑고 박스만 추적해도 API / 채택 / 검증 통과 여부 파악 가능.

---

## Reading paths — 학습자의 목적별

전체 16 chapter 의 순차 독해 대신, 목적에 따라 4 가지 path 중 하나로
시작하기를 권장한다. 각 path 는 self-contained.

### Path A — 차량 동역학 textbook (입문 sequence)

`01 → 02 → 03 → 04 → 05 → 06`

좌표계 / 부호 → 강체 Newton-Euler → 타이어 Pacejka → (선택) **19 LuGre** →
모델 사다리 (Ld1 bicycle → Ld2 7-DOF → Ld3 14-DOF). 차량 동역학 수업의 표준
chapter 순.

### Path B — 자율주행 제어 (control sequence)

`07 → 08 → 09 → 10`

variant dispatch (control ladder) → PI + feed-forward → pure pursuit →
driver model (latency / noise). Path A 의 04 이후에 읽으면 dynamics 와
control 의 cascade 가 명확.

### Path C — 수치 적분 / 소프트웨어 (engineering sequence)

`11 → 12`

RK4 + substep + 1-step lag → C++17 ABI / pybind11 / CARLA bridge. 실시간
loop / language binding / ABI 안정성에 관심 있는 reader.

### Path D — Multibody / 산업 표준 (advanced sequence)

`13 → 14 → 15 → 16`

lumped 14-DOF 의 한계 → hardpoint kinematics → ISO 표준 maneuver / DOE
→ FMI 2.0 co-simulation. Path A 완독 후 권장.

---

## Chapter index — 한 줄 objective

| # | chapter | length | prereq | learning objective |
|---|---|---|---|---|
| 01 | [Frames & Conventions](01_frames_and_conventions.md) | 짧음 | 선형대수 | ISO 8855 RH 좌표계와 alpha/kappa 부호 약속을 정확히 정의 |
| 02 | [Rigid-Body Dynamics](02_rigid_body_dynamics.md) | 중 | 01 | Body-frame Newton-Euler EoM 을 inertial / body 의 차이와 함께 유도 |
| 03 | [Tire — Pacejka MF96](03_tire_pacejka_mf96.md) | 길음 | 01, 02 | Magic Formula 의 의미와 friction-ellipse rescale 의 가정 |
| 19 | [Tire — LuGre / brush-dynamic](19_lugre_dynamic_tire.md) | 중 | 01, 03, 11 | Bristle state $z$, MF96 as $g()$, presliding vs sliding; VDSim opt-in |
| 25 | [Tire Contact & Interface](25_tire_contact_interface.md) | 중 | 01, 03, 19, 21 | Effective rolling radius $R_e(F_z)$, camber contact migration → $M_x$, inverted kinematics-in/wrench-out interface |
| 04 | [Ld1 — Bicycle (5 DOF)](04_ld1_bicycle.md) | 중 | 02, 03 | Single-track 가정 하의 5 DOF EoM 과 understeer gradient 의 해석해 |
| 05 | [Ld2 — Seven-DOF](05_ld2_seven_dof.md) | 중 | 04 | Per-tire + lateral weight transfer + Ackerman + differential 분기 |
| 06 | [Ld3 — Fourteen-DOF](06_ld3_fourteen_dof.md) | 길음 | 02, 04 | Sprung 3 + unsprung 4 의 lumped vertical 동역학 |
| 07 | [Control Ladder Lc1-Lc8](07_control_ladder.md) | 짧음 | — | std::variant 기반 control input 의 m × n 사다리 매트릭스 |
| 08 | [PI + Feed-Forward Cascade](08_pid_controllers.md) | 중 | 기초 제어 | Lc5 AxTarget / Lc6 VTarget 의 PI + FF + anti-windup |
| 09 | [Pure Pursuit / Waypoint](09_pure_pursuit_path.md) | 중 | 기하 | Geometric path tracking 의 유도, lookahead scheduling |
| 10 | [Driver Model](10_driver_model.md) | 짧음 | 09 | Reaction delay + Gaussian noise 의 cascade 통합 |
| 11 | [Numerical Integration](11_numerical_integration.md) | 중 | ODE 기초 | RK4 + substep + 1-step lag 의 안정성과 오차 |
| 12 | [Software Architecture](12_software_architecture.md) | 중 | C++17 | Factory + pure virtual ABI, std::variant, pybind11, CARLA plugin |
| 13 | [Multibody Outlook (Ld4-Ld5)](13_multibody_outlook.md) | 길음 | 02, 06 | Lumped 의 한계와 hardpoint kinematics / quasi-static compliance 의 동기 |
| 14 | [Hardpoint Kinematics](14_hardpoint_kinematics.md) | 중 | 13 | 6 가지 standard suspension topology 의 forward kinematics |
| 15 | [Validation & DOE](15_validation_and_doe.md) | 짧음 | 04, 05 | ISO 7401 / 4138 / 3888-2 표준 maneuver 와 parameter sweep |
| 16 | [FMI 2.0 Integration](16_fmi_integration.md) | 짧음 | 12 | FMU export / import + co-simulation 의 양방향 통합 |
| 17 | [Actuator Dynamics & Sensing](17_actuator_and_sensing.md) | 중 | 11, 07 | FOPDT·rate·sat·dead-zone, steering servo(PD+inertia+LuGre) vs lag, brake mu(T), sensor delay |
| 18 | [Runtime Kernel & Co-simulation](18_runtime_and_cosim.md) | 중 | 17, 12 | SimSession kernel·run modes, RealTimeRunner watchdog, UDP wire protocol, GUI data port (JSON vs binary) |

---

## Notation — 시리즈 공통

본문 식 안에서는 다음 약속을 expand 없이 사용한다.

### 좌표계와 부호

- **ISO 8855 RH** — body frame: x forward, y leftward, z up.
- **World frame** — ENU (East-North-Up) RH.
- **Quaternion** — body → world. Eigen convention 으로 `Quat::Identity()` = no rotation.
- **Euler angles** — ZYX intrinsic (yaw → pitch → roll).
- **Wheel index** — FL = 0, FR = 1, RL = 2, RR = 3.

### 자주 헷갈리는 부호

| 항목 | VDSim (ISO 8855) | 비교 |
|---|---|---|
| 좌선회 시 yaw rate `r` | r > 0 | — |
| 좌선회 시 front slip angle `alpha` | alpha > 0 | — |
| 좌선회 시 front lateral force `Fy` | Fy < 0 (restoring) | Rajamani (SAE Y-right) 는 부호 반대 |
| Roll `φ` 양의 방향 | 차체 right side 가 아래 | SAE 호환 |
| Pitch `θ` 양의 방향 | nose-up | brake → nose-dive → θ < 0 |

### Greek / 변수

- **질량 / 관성** — `m`, `m_s`, `m_u` (total / sprung / unsprung per corner), `Izz`, `Ixx`, `Iyy` (sprung body inertia diagonal).
- **기하** — `L`, `a`, `b` (wheelbase, CG-to-front-axle, CG-to-rear-axle. `L = a + b`), `Tw_f`, `Tw_r` (track width), `h_cg`, `R` (nominal kinematic wheel radius).
- **운동** — `vx`, `vy`, `r` (body-frame velocity x/y + yaw rate), `delta` (front steer angle, rad).
- **타이어** — `alpha` (slip angle, rad), `kappa` (slip ratio), `Fz`, `Fx`, `Fy`, `Mz`, `mu` (friction). `mu_long`, `mu_lat` per-axis scaling.

### 약자 (본문에서 expand 하지 않음)

- **MF96** — Pacejka Magic Formula 1996 simple form.
- **DOF** — degree of freedom.
- **EoM** — equation of motion.
- **ABI** — application binary interface.
- **K&C** — kinematics and compliance (suspension chart).

---

## 핵심 참고 문헌

각 chapter 의 §참고 절은 이 list 의 책 / paper 를 우선 인용한다.

1. **Genta, G.** *Motor Vehicle Dynamics: Modeling and Simulation*, World Scientific, 2014. 차량 동역학 표준.
2. **Rajamani, R.** *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. SAE Y-right convention (본 시리즈와 alpha 부호 반대).
3. **Pacejka, H.B.** *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012. MF96 / MF2002 의 정본.
4. **Milliken, W.F. & Milliken, D.L.** *Race Car Vehicle Dynamics*, SAE International, 1995. weight transfer / Ackerman / differential 의 직관.
5. **Reimpell, J., Stoll, H., Betzler, J.** *The Automotive Chassis: Engineering Principles*, Butterworth-Heinemann, 2001. suspension geometry / K&C.
6. **Featherstone, R.** *Rigid Body Dynamics Algorithms*, Springer, 2008. multibody (Ld4-Ld5).
7. **Thrun, S., Burgard, W., Fox, D.** *Probabilistic Robotics*, MIT Press, 2005. noise convention (R = process noise, Q = observation noise).

---

## VDSim 코드 매핑

본문 implementation note 박스가 자주 가리키는 파일.

| 파일 | 역할 |
|---|---|
| `core/include/vdsim/types.hpp` | Eigen typedef + wheel index constants |
| `core/include/vdsim/state.hpp` | State struct (모든 사다리 cover) |
| `core/include/vdsim/params.hpp` | VehicleParams / TireParams / SolverParams |
| `core/include/vdsim/interfaces.hpp` | IVehicleDynamics + ITireModel + IContactProvider |
| `core/include/vdsim/control.hpp` | ControlInput = variant<CmdL1..L8> |
| `core/include/vdsim/control_converter.hpp` | LongAxController / LongVxController / PurePursuit / DriverModel |
| `core/include/vdsim/multibody.hpp` | Ld4-Ld5 의 hardpoint / joint / bushing struct |
| `core/src/pacejka_mf96.cpp` | MF96 + combined slip + Mz + camber |
| `core/src/bicycle_dynamics.cpp` | Ld1 본체 |
| `core/src/seven_dof_dynamics.cpp` | Ld2 본체 |
| `core/src/fourteen_dof_dynamics.cpp` | Ld3 본체 |
| `core/src/control_converter.cpp` | Lc5 / Lc6 / Lc7 / DriverModel |

---

## 시리즈 상태

| chapter | 본문 (교과서) | 구현 노트 박스 | self-check |
|---|---|---|---|
| 01 – 16 | draft 1차 완성 | 본문에 inline 형태로 분산 (분리 작업 진행 중) | 미구비 |

본문 ↔ 구현 노트 박스 분리, self-check 문제, math appendix (변환 식 derivation),
worked examples, Adams Car 실측 fit case study 가 follow-up.
