# VDSim 핸드오프

작성: 2026-06-05. v0.1.0 공개 완료. v0.2 진행 중 — 서브시스템 모듈 완료, **GUI 전면
재설계(app.html) step 1 완료(유저 시각검토 대기)**. 멀티차량은 v0.2.0 최후단.

## 1. 목표
v0.2 "composable vehicle": 차량 = chassis + parts(모듈), 컴포넌트 워크샵 + 씬 UI 로
조립·구동. 현재 집중 = **GUI 전면 재설계(좌 사이드바=SIM/차량트리/미니맵, 중앙 3D,
우 텔레메트리, [차량 edit]→큰 모달=모듈 워크샵)**. 멀티차량 런타임은 맨 마지막.
전체 그림: `docs/design/V0.2_PLAN.md`.

## 2. 현재 상태
- origin/main = `1cbea58`. **로컬 미push 9커밋**(서브시스템 #161c~ + GUI 재설계). push 는
  사용자 명시요청 시에만.
- 빌드 `cmake --build build -j`; **ctest 187/187**.
- v0.1.0 공개됨(태그 → `4c77d7f`). GitHub public 토글+Topics 는 사용자 UI 작업(미완).
- **Cursor 위임 워크플로 가동**: 기계적/GUI 작업은 `~/bin/cursor_delegate.sh "<지시>"
  <type> [ws]`(composer-2.5 고정 + usage 로깅) → Claude 가 리뷰+빌드+ctest 검증 후 커밋.
  규약은 `.cursor/rules/vdsim.mdc`, 라우팅은 `docs/design/CURSOR_USAGE.md`.

## 3. 완료
저속 동역학(v0.1.0 블로커, L1/L2/L3): kinematic blend + brake-hold + lateral λ-fade +
force display fade. `docs/design/LOW_SPEED_HANDLING.md`.
서브시스템(WS1, 187 green):
- #159 인터페이스+SubsystemContext+DriverCmd+DelayLine (`subsystems.hpp`,`delay_line.hpp`)
- #160 default 모듈 (`default_subsystems.{hpp,cpp}`): Proportional Brake / Basic Drivetrain /
  Ratio·Unity Steering / Linear Suspension / Linear ARB — 현재거동 정확 복제.
- #161 코어 배선: L2 seven_dof drive/brake/steer 모듈 경유(+동적 Fz EBD); L3 는 inner L2
  위임으로 자동 + per-corner suspension 모듈 경유; L1 면제(single-track).
- #162 ARB per-wheel force 재설계(F_susp 합산, pure-roll 등가).
- deadtime 활성화(begin_step 제어스텝당 1회) + **config bridge**(VehicleParams 에
  brake/drive/steer_deadtime_s, YAML/bindings/VEHICLE_FIELDS 노출). full-sim 검증됨.
GUI 재설계 step 1: **`gui/app.html`**(`/app` 서빙) — 3-컬럼(좌 SIM컨트롤+차량·센서트리+
미니맵 / 중앙 3D 포팅 / 우 텔레메트리 pose·vel·accel·wheel_spin·slip·Fz·Ft·rack). [edit]→
모듈 모달(Susp/Brake/Steering/Drivetrain/Tire/Sensors 탭, /api/vehicle|tire|sensors 라이브).
wheel_spin 을 cosim bridge+SSE snapshot 에 추가(휠스피드 표시). **index.html 무손상**.

## 4. 미완 (다음 할 일)
- **app.html 시각검토 + 반복**(유저): `PYTHONPATH=build/python:cosim python3 gui/server.py
  --port 8096` → `http://localhost:8096/app`. 레이아웃/비율/모달/텔레메트리 항목 피드백 →
  반영. (유저가 "시행착오 필요" 명시.)
- GUI 재설계 나머지(`docs/design/V0.2_GUI_REDESIGN.md` Sequence): 모듈 모달 완성도, Tire
  .tir import+샘플(`python/tir_to_yaml.py`+`from_tir`), SIM save/load, infra/ground 센서
  그룹, 멀티차량 트리(#157 연동 시 채워짐).
- 승인되면 **app.html 을 기본('/')으로 교체**하고 기존 index.html 의 독립 Workshops 창
  (`e7d47d1`)은 제거 정리(app.html 의 모듈 모달로 대체됨).
- **멀티차량 런타임 #157/#158 = v0.2.0 최후단**(사용자 결정). VDS1 v4 vehicle_id 완료,
  설계 `docs/design/V0.2_MULTIVEHICLE.md`(no runtime spawn, multi-client racing, subscriber).
- v0.3 이관: 엔진 torque-RPM 곡선+관성(Issue2, `V0.2_DRIVETRAIN.md`), LuGre tire
  (`V0.2_TIRE_LUGRE.md`). brake EBD 정적 fallback(정상시 동적 Fz, 무영향).

## 5. 주의 · 함정
- **GUI 시각검증 불가**(에이전트 환경에 브라우저/JS엔진 없음) → serve/markup/괄호balance 까지만
  확인, 렌더링·동작은 유저가 /app 으로 확인. UI 변경은 항상 이 한계 명시.
- **index.html 무손상 유지**: app.html 은 별도 페이지(회귀 0). 승인 전 index.html 건드리지 말 것.
- **187 green 불변식**: default 모듈==현재거동. 서브시스템 변경 시 ctest 187 유지(재baseline 금지,
  단 ARB/엔진처럼 의도된 거동변경은 예외 — 그땐 spot-check+문서화).
- **orphan plant**: 죽은 GUI 가 vdsim_realtime 를 7401/7402 에 남기면 새 GUI 가 plant 못 띄움 →
  그 **특정 pid** 만 `kill <pid>`(광범위 pkill 금지=classifier 차단). 무관 점유: `mike serve`
  (8090 mkdocs), `ailab_dashboard.py`(8091). kill 체인에서 exit 144 떠도 실제 실행됨(`ps` 확인).
- estimation noise = **Simon `Q=process / R=measurement`**(Thrun 아님; SLAM 맥락만 Thrun).
- TUR 실측 tire 데이터 대외비 — generic 합성만, .tir 값/세부 커밋 금지.
- VDS1 v4: plant·GUI 둘 다 v4(version gate). protocol.py 수정 후 GUI **재기동** 필요.
- git: push/force/tag 이동은 **명시 요청 시에만**.

## 6. 관련 경로
- GUI: `gui/app.html`(신규, /app), `gui/server.py`(/app route + snapshot/bridge wheel_spin +
  VEHICLE_FIELDS), `gui/index.html`(기존, 무손상).
- 코어 서브시스템: `core/include/vdsim/{subsystems,default_subsystems,delay_line}.hpp`,
  `core/src/default_subsystems.cpp`, `core/src/{seven_dof,fourteen_dof,bicycle}_dynamics.cpp`,
  `core/include/vdsim/params.hpp` + `core/src/params.cpp`, `python/bindings.cpp`.
- 프로토콜: `cosim/cosim_protocol.hpp` + `cosim/protocol.py`(v4 vehicle_id).
- 설계 docs: `docs/design/V0.2_{PLAN,GUI_REDESIGN,WORKSHOPS,SUBSYSTEMS,MULTIVEHICLE,
  DRIVETRAIN,TIRE_LUGRE}.md`, `LOW_SPEED_HANDLING.md`.
- Cursor: `docs/design/CURSOR_USAGE.md`, `.cursor/rules/vdsim.mdc`, `~/bin/cursor_delegate.sh`.
- task 보드: #157/#158(멀티차량, 최후단), #155(데모 GIF), #138/#140(B/CARLA 논의).
