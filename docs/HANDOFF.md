# VDSim 핸드오프 — v0.1.0 공개 완료 + v0.2 멀티차량 착수

작성: 2026-06-05. v0.1.0 published. v0.2: 서브시스템 골격(#159-162)+deadtime+ARB 재설계
완료, **WS2 컴포넌트 워크샵 진행 중**. 멀티차량(#157/#158)은 v0.2.0 최후단(사용자 결정).

## WS2 컴포넌트 워크샵 상태 (현재 작업)
설계 spec: `docs/design/V0.2_WORKSHOPS.md`. 새 Workshops 창, UI 새로 설계(사용자 결정).
- **DONE(검증)**: WS2-0 deadtime config bridge(`aa4fca7`, full-sim 검증) · 백엔드 필드 노출
  (`f8e318a`, VEHICLE_FIELDS 에 deadtime/final_drive).
- **DONE(시각검증 대기)**: 새 Workshops 창 UI(`e7d47d1`, gui/index.html) — toolbar 'Workshops'
  → 드래그 창, 좌측 Suspension/Brake/Steering/Drivetrain 셀렉터, 우측 /api/vehicle 라이브 편집
  +곡선. **serve/markup/필드/괄호balance 는 OK 확인, 브라우저 렌더링은 미확인** → 유저가
  `python3 gui/server.py --port 8096`(PYTHONPATH=build/python:cosim) 열어 Workshops 창·기존
  GUI(3D/Plant) 정상 확인 필요. 깨지면 e7d47d1 수정/revert.
- **남음**: Tire 탭(.tir import 엔드포인트 + 곡선; `python/tir_to_yaml.py parse_tir`+FIELD_MAP →
  set_params("tire") — 단 파싱키가 TIRE_FIELDS 화이트리스트에 있어야 함, 매핑 커버리지 확인 필요).
  Engine 탭(stub, 모델 v0.3). 샘플 라이브러리는 **기존 `configs/{tires,suspensions}/`** 사용
  (새 components/ 트리 불필요 — V0.2_WORKSHOPS 의 configs/components 결정은 이걸로 정정).
  보안: tire 샘플은 generic 합성만, 실측 현대 .tir 값 커밋 금지.

다음(WS2 후): WS4 parts → WS3 scene UI → 멀티차량 런타임 #157/#158(최후단).

## 1. 목표
v0.2 "composable vehicle": 컴포넌트 모델 + 워크샵 + 씬 UI 로 차량 조립, 멀티차량
시나리오 구동. 지금 단계는 그 토대인 **멀티차량 shared-world 런타임**.
전체 그림: `docs/design/V0.2_PLAN.md`.

## 2. 현재 상태
- main 최신 push: `5684c92`. 작업트리 clean.
- 빌드: `cmake --build build -j`. **187/187 ctest 통과**.
- v0.1.0 공개됨: origin/main + 태그 `v0.1.0` → `4c77d7f` (force-move 완료). 단,
  **GitHub repo visibility=public 토글 + Topics 는 사용자가 UI 에서** 해야 함(미완).
- 라이브 GUI: `python3 gui/server.py --port 8095` (PYTHONPATH=build/python:cosim).
  현재 8095 에 v4 로 떠 있을 수 있음(pid 4136549). 재기동 시 orphan plant 가 7401/7402
  점유하면 그 특정 pid 만 kill 후 재기동 (아래 함정 참고).

## 3. 완료 (이번 arc)
저속 동역학 (v0.1.0 블로커, L1/L2/L3 + 187/187):
- kinematic-dynamic blend(<3 m/s, lateral 을 derivative 에서 kinematic 으로 cross-fade)
  → 스핀/휘청/드리프트/정지진동 제거. `e3e0009`.
- viscous brake-hold 댐퍼(brake>throttle, v_x_body 기준) → 경사 hold creep cm/s. ring 없음.
- lateral force 를 dynamics 에서 λ-fade → ay→0, 정지 시 Fz 안정(떨림 해결). `6aec66c`.
- 보고 force 도 저속 λ-fade(display) → 화살표 chatter 제거. `96321a3`.
- 설계 근거: `docs/design/LOW_SPEED_HANDLING.md`.
v0.2 착수:
- 로드맵/설계 문서 4종: `V0.2_PLAN`, `V0.2_MULTIVEHICLE`, `V0.2_DRIVETRAIN`, `V0.2_TIRE_LUGRE`.
- 증분 1 — **VDS1 v4 `vehicle_id`** (`cosim_protocol.hpp` + `protocol.py`): 헤더 pad→
  vehicle_id, 크기 불변(CMD76/STATE436), version 3→4. C++↔py 교차검증 완료. `5684c92`.

## 4. 미완 (다음 할 일)
- **#157 멀티차량 world 런타임** (`cosim/realtime_server.cpp`): N SimSession 을 공유환경
  ·공유 clock 으로 step, scenario YAML 로 차량목록(vehicle/tire yaml·level·spawn·id),
  CMD 를 vehicle_id 로 demux(per-vehicle ZOH), tick 당 N STATE 송신. flat CLI = N=1 단축.
  설계 그대로: `docs/design/V0.2_MULTIVEHICLE.md` "Runtime"/"Implementation increments".
- **#158 GUI 멀티차량**: CosimBridge 가 scenario 로 plant 1개 spawn, STATE 를 vehicle_id 로
  demux→`self.ports[vid]`, 제어는 선택 vehicle_id 로 CMD, 렌더 N대. GUI snapshot 에
  vehicle_id 포워딩(현재 빠짐). 데이터층은 이미 vid 키잉(`live_vid`, `self.ports`).
- 충돌(V2V): **future TODO** (이번 범위 아님). contact-coupling pass 훅 위치만 런타임에
  남겨둠. `V0.2_MULTIVEHICLE.md` "Collision" 참고.
- **서브시스템 모듈 (설계 lock 완료, `V0.2_SUBSYSTEMS.md` 결정 1-7)** — #159/#160 완료,
  #161(dynamics 연결) 대기:
  brake(pedal→4륜 토크) / steering(handwheel→roadwheel·rack, unity 변형 포함) /
  drivetrain(throttle→4륜 토크, skeleton=flat) / suspension(코너별) / anti-roll bar
  (축별 별도 모듈) / 각 모듈 deadtime. 모두 `SubsystemContext`(full state) 받음, C++,
  default==현재거동(187 유지). 엔진 torque-RPM 곡선+관성(Issue2)은 **v0.3 이관**.
- v0.2 이후 모듈: LuGre tire(저속 blend 대체, `V0.2_TIRE_LUGRE.md`).

## 구현 순서 (사용자 결정: 3 부터) + Cursor 위임 워크플로
서브시스템 골격 (#159-161). 멀티차량(#157/#158)은 그 뒤로 미룸.
**Cursor 위임 동작 확인됨**: 기계적 작업은 `cursor-agent -p --force --trust --model auto
--output-format text "<지시>"` 로 위임 → Claude 가 리뷰+빌드+ctest 검증 후 커밋(push 금지).
`.cursor/rules/vdsim.mdc`(규약·가드레일 미러), `docs/design/CURSOR_USAGE.md`(라우팅) 적용됨.

- #159 **DONE** (커밋 f8305b9, Cursor 위임→Claude 검증): `core/include/vdsim/
  subsystems.hpp`(인터페이스+SubsystemContext+DriverCmd) + `delay_line.hpp`. 187 green.
- #160 **DONE** — default 모듈 (`core/include/vdsim/default_subsystems.hpp` +
  `core/src/default_subsystems.cpp`): ProportionalBrake, BasicDrivetrain,
  RatioSteering, UnitySteering, LinearSuspension, LinearARB + factory helpers.
  아직 dynamics 미연결(#161). 187 green.
- #161 **DONE** — 코어가 default 모듈 경유, default==현재거동, 187 green:
  - L2 drive/brake/steer 모듈 경유 (5a952f7, +동적 Fz EBD). L3 는 inner L2 위임이라 자동 수혜.
  - L3 per-corner suspension 모듈 경유 (8c11354). L1 은 single-track 이라 면제(인라인 유지).
  - **ARB DONE (#162, b60e139)**: L3 ARB 를 per-wheel force(LinearARB)로 재설계 — F_susp 에
    더해 heave/pitch/roll+unsprung 에 작용. pure-roll 은 −K_arb·φ 와 등가, 비대칭 입력도 반응.
    기본 sedan arb=0 라 187 불변; nonzero 시 roll 단조 억제 확인(susp FL-FR: 0→−0.083, 100k→−0.022).
  - **남은 연기 항목**(비차단):
    (b) ~~deadtime 미활성~~ **DONE** — `begin_step(ctx,dt)` 가 제어스텝당 1회 delay advance,
    `wheel_torque`/`apply` 는 캐시 `*_eff_` 사용(derivatives dt=0, RK4 4회 재진행 없음).
    (c) brake EBD 정적fallback(total<=1)만 잔존 — 정상 동작 시 동적 Fz 사용(일치).
- 그 다음: #157 멀티차량 런타임 → #158 GUI 레이싱.
- v0.3 이관: 엔진 torque-RPM 곡선 + 관성(Issue 2), LuGre tire.
설계 spec: `docs/design/V0.2_SUBSYSTEMS.md` (결정 1-7, 인터페이스, per-level 표).

## 5. 주의 · 함정
- **VDS1 v4**: plant·GUI 둘 다 v4 여야 통신됨(version gate). protocol.py 수정 후 GUI
  서버 **재기동**해야 새 버전 반영(import 시점 로드). plant 는 재빌드.
- **orphan plant**: 죽은 GUI 서버가 vdsim_realtime 를 7401/7402 에 남기면 새 GUI 가 plant
  를 못 띄움. 해결=그 **특정 pid** 만 `kill <pid>` (광범위 `pkill -f vdsim_realtime` 는
  classifier 가 막음 — 타 사용자 프로세스 영향 우려). 무관 점유자: `mike serve`(8090,
  mkdocs), `ailab_dashboard.py`(8091) — 건드리지 말 것.
- 저속 수정은 **λ=1(>3 m/s)에서 완전 무영향** 설계라 ISO/187 불변. 재튜닝 시 이 불변식 유지.
- **재baseline**: drivetrain/tire 모듈은 throttle 응답·ISO 수치를 바꾸므로 마일스톤당 1회
  검증 패스로 `docs/VALIDATION.md` 갱신.
- TUR 실측 tire 데이터 대외비 — generic 지칭만, .tir 값/세부 커밋 금지.
- kill/체인 명령에서 exit 144 떠도 특정 pid kill 은 실제 실행됨. `ps`로 확인.

## 6. 관련 경로
- 프로토콜: `cosim/cosim_protocol.hpp` + `cosim/protocol.py` (v4, vehicle_id)
- 런타임: `cosim/realtime_server.cpp` (단일차량 → #157 에서 N차량 world)
- GUI: `gui/server.py` (CosimBridge spawn/subscribe, Runner self.ports[vid]), `gui/index.html`
- 코어 동역학: `core/src/{bicycle,seven_dof,fourteen_dof}_dynamics.cpp` (저속 blend 적용됨)
- v0.2 설계: `docs/design/V0.2_{PLAN,MULTIVEHICLE,DRIVETRAIN,TIRE_LUGRE}.md`,
  `docs/design/LOW_SPEED_HANDLING.md`
- task 보드: #157(멀티차량 런타임), #158(GUI 멀티차량), #155(데모 GIF), #138/#140(B/CARLA 논의)
