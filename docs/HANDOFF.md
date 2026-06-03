# VDSim 핸드오프 — research-infra arc + road-surface (#131)

작성: 2026-06-03. 다음 세션은 이 문서로 인계받아 **slice 1 (slope-gravity EoM)** 부터 시작.

## 1. 목표
VDSim 을 실제 연구(TUR Fz/μ/Cα UKF, NHCalib, slip-tolerant SMPC, FSK)에서 쓸 수
있게 "연구 인프라"(센서·로깅·선형화·Monte Carlo·검증·maneuver·estimator)와
road-surface 를 깔고, 구현 물리를 전부 매뉴얼화한다. dynamics 충실도는 이미 충분.

## 2. 현재 상태
- main 최신 push: `ba15244`. 작업트리 clean (logs/ 는 gitignore).
- 빌드: `cmake --build build -j`. tests 125 unit + 57 integration **전부 통과**.
- GUI: `python3 gui/server.py --port <p>` (kill 은 classifier 가 막아 exit 144 뜨지만
  실제로는 동작 — `ps`로 확인). 떠있는 인스턴스 정리는 사용자가 `!kill`.
- 매뉴얼: theory ch01–18 (배포 mike "main": onlyti.github.io/VDSim/main/theory/...).

## 3. 완료 (이번 arc)
research-infra (task #124–#130) — 전부 완료·push:
- #126 `vdsim.linearize(...)` → 운전점 이산 A(6×6)·B(6×2). state [X,Y,ψ,vx,vy,r].
- #125 GUI 로깅 → CSV + TUM(evo). `/api/log/start|stop|status` + download.
- #124 `SensorModel`(core): IMU/gyro/wheel/steer/GNSS noise+bias+RW. SimSession→
  SimOutput.sensors. GUI Plant "Sensors" + measured 로깅. disabled=identity.
- #130 `examples/estimator_in_loop.py`: estimator testbench (RMSE + NEES). 사용자
  UKF 는 `reset(x0)/step(meas,cmd,dt)->(x_hat,P)` 로 끼움. 레퍼런스 EKF 동봉.
- #127 `examples/monte_carlo.py`: 불확실성 샘플 → percentile band + violation rate
  (chance-constraint). 데모 P(viol)=13.3%.
- #128 `examples/validate_against_log.py`: log replay → NRMSE. CSV universal +
  ERG/ADMA/rosbag stub. self-test(exact~0, +10%mass→NRMSE급증).
- #129 `examples/maneuvers.py`: ISO 7401/4138/3888-2.
- 매뉴얼: ch17(actuator+sensing), ch18(runtime+co-sim), Lk(§4.14), full-MF(§3.16),
  ch05 quasi-static-roll 노트. 설정 GUI 각 섹션에 "?" → 해당 doc 링크.
- 부수: 초기위치 설정(init_x/y/yaw/v), throttle dead-zone, LuGre servo-mode grey-out,
  per-vehicle data port, co-sim 구동·시각화, plant param live-link/applicability.
- #131 **slice 0 (split-μ)**: `create_split_mu_ground`, make_sim_session(mu_right,
  mu_boundary_y), GUI Simulation "Road/surface"(uniform μ + split). 검증됨.

## 4. 미완 — 다음 할 일 (#131 slice 1: slope-gravity, = #123)
**즉시 착수 대상.** banked/graded road 의 중력 성분을 EoM 에 넣어 CG-shift/경사 거동.

설계:
- 현재 `ContactPoint.normal` 은 있으나 **dynamics 가 무시**(flat: Fz_static=m·g·b/(2L),
  중력 투영 없음). FlatGround/SplitMuGround 는 normal=+z 고정.
- 할 일:
  1. **banked/graded contact provider** — `create_inclined_ground(z, grade, bank,
     mu)` (또는 더 일반 surface): per-wheel z(평면) + tilted normal 반환.
  2. **EoM 중력-투영 항** — bicycle/seven_dof/fourteen_dof 의 body-frame force
     balance 에 `Fx += -m·g·sin(grade)`, `Fy += +m·g·sin(bank)`, 수직하중
     `Fz *= cos(slope)` 추가. grade/bank 는 4-contact normal 평균을 body frame 으로
     회전해 추출(normal 은 WORLD frame).
  3. L3 vertical: 경사가 static compression/normal load 바꿈.
  4. GUI: grade/bank 입력 + road tilt 렌더(#123 render).

순서 권장: provider(1) → seven_dof EoM(2, L2 먼저) → 검증 → bicycle/L3 → 렌더.

## 5. 주의 · 함정
- **flat-road 거동 보존**: normal≈+z 면 slope 항 skip(gate)해서 182 test 가 안 깨지게.
  안 그러면 재baseline 필요(물리정합 우선이면 OK지만 먼저 gate 로 무영향 확인).
- **부호**: ISO 8855(+x fwd,+y left,+z up). 오르막 grade→감속(Fx<0). normal 은
  world→body 회전 후 분해.
- **per-wheel μ 는 이미 동작**(split-μ 가 증명). slope 는 normal 을 EoM 이 써야 비로소 동작.
- Euler 적분기는 stiff 동역학에서 발산 — 기본 RK4 유지(integrator 셀렉터에서 Euler 고르면 깨짐).
- pkill/kill 은 classifier 가 exit 144 내지만 실제 실행됨. 체인 명령은 `;` 로 끊겨도 별도 실행.
- TUR 실측 tire 데이터는 대외비 — generic 지칭만, .tir 값·세부 기록/커밋 금지.

## 6. 관련 경로
- 코어 동역학: `core/src/{bicycle,seven_dof,fourteen_dof}_dynamics.cpp`
- contact: `core/src/contact_providers.cpp`, `core/include/vdsim/interfaces.hpp`(ContactPoint)
- 센서/액추에이터: `core/{include/vdsim,src}/{sensors,actuator}*`
- 바인딩: `python/bindings.cpp` (make_sim_session, linearize, create_*_ground)
- GUI: `gui/server.py`(Runner·endpoints), `gui/index.html`(buildPlant/buildSim/buildDataPort)
- 연구 예제: `examples/{estimator_in_loop,monte_carlo,validate_against_log,maneuvers}.py`
- 매뉴얼: `docs/theory/*.md` (ch05/06/17/18 + index `docs/theory/README.md`, `mkdocs.yml`)
- task 보드: #117/#118/#123 (= #131 잔여 슬라이스), #121/#122(GUI 후속)
