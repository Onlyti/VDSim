# VDSim 핸드오프 — v0.1.0 공개 완료 + v0.2 멀티차량 착수

작성: 2026-06-05. v0.1.0 published. v0.2 설계 lock 완료(멀티차량+서브시스템).
다음 = **서브시스템 모듈 골격 (step 3, 아래 구현순서)**. 멀티차량 런타임(#157/#158)은
설계만 끝났고 그 뒤로 미룸 (사용자 결정).

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
- **서브시스템 모듈 (설계 lock 완료, `V0.2_SUBSYSTEMS.md` 결정 1-7)** — 구현 대기:
  brake(pedal→4륜 토크) / steering(handwheel→roadwheel·rack, unity 변형 포함) /
  drivetrain(throttle→4륜 토크, skeleton=flat) / suspension(코너별) / anti-roll bar
  (축별 별도 모듈) / 각 모듈 deadtime. 모두 `SubsystemContext`(full state) 받음, C++,
  default==현재거동(187 유지). 엔진 torque-RPM 곡선+관성(Issue2)은 **v0.3 이관**.
- v0.2 이후 모듈: LuGre tire(저속 blend 대체, `V0.2_TIRE_LUGRE.md`).

## 구현 순서 (사용자 결정: 3 부터)
**다음 = 서브시스템 골격** (#159-161). 멀티차량(#157/#158)은 그 뒤로 미룸.
- #159 인터페이스 + `SubsystemContext`(full state) + `DriverCmd`(handwheel/throttle/
  brake/gear/handbrake) + `DelayLine`(deadtime 헬퍼). `core/include/vdsim/` 에.
- #160 default 모듈 (전부 현재거동 재현): brake(비례+bias/EBD) / steering(ratio +
  unity) / drivetrain(flat torque+diff) / suspension(선형, 코너별) / anti-roll bar
  (선형, 축별). 각 모듈 deadtime 파라미터(default 0).
- #161 dynamics 코어를 default 모듈 경유로 리팩터 — **default==현재거동, 187 green
  유지**가 합격 기준. L0/L1=roadwheel, L2+=rack. suspension/ARB 는 step 내부 콜백.
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
