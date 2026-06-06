# VDSim 핸드오프

작성: 2026-06-05. v0.1.0 공개 완료. v0.2 진행 중 — 서브시스템 모듈 완료, **GUI 재설계
app.html 승인·`/` 기본 라우트 완료(2026-06-06)**. 멀티차량 런타임 코드 완료.

## 1. 목표
v0.2 "composable vehicle": 차량 = chassis + parts(모듈), 컴포넌트 워크샵 + 씬 UI 로
조립·구동. **`gui/app.html` = 기본 GUI (`/`)**. 레거시 뷰어는 `/legacy`.
전체 그림: `docs/design/V0.2_PLAN.md`.

## 2. 현재 상태
- origin/main = `1cbea58`. **로컬 미push 커밋 다수**(서브시스템 + GUI 재설계 + plant/fleet
  fixes). push 는 사용자 명시요청 시에만.
- 빌드 `cmake --build build -j`; **ctest 187/187**.
- v0.1.0 공개됨(태그 → `4c77d7f`). GitHub public 토글+Topics 는 사용자 UI 작업(미완).

## 3. 완료
저속 동역학, WS1 서브시스템 (#159-162 + deadtime), 멀티차량 #157/#158, GUI `app.html`
(3-컬럼, setup, fleet, telemetry, edit 모달, help `?`, track edit, manual 키).
**`/` → app.html**; `/legacy` → index.html (Workshops 제거, New GUI 링크).
plant: orphan cleanup, STATE subscriber fix (`touch_sub(seed)`), fleet ghost prune,
`_renumber_fleet`, setup snapshot 즉시 갱신.
설계: `docs/design/V0.4_PLAN.md` (스턴트 v0.4.0 단일 릴리스).

## 4. 미완 (다음 할 일)
- **WS4** vehicle assembly (chassis + parts YAML).
- **WS3** 잔여 — data-comms 패널, 3-tab 설계 대비 갭 (app에 setup/sim 일부 있음).
- **WS2** — app 모달 vs index Workshops parity (Tire import·곡선 등 index에만 있던 것 이전).
- v0.3.0: drivetrain 관성+torque-RPM, LuGre tire (`V0.2_DRIVETRAIN.md`, `V0.2_TIRE_LUGRE.md`).
- v0.4.0: 점프~루프 (`V0.4_PLAN.md` M1–M6).
- 멀티차량 future: V2V, dynamic spawn, mesh LOD.
- git push / Topics / 데모 GIF (#155).

## 5. 주의 · 함정
- GUI 시각검증은 유저 담당; 에이전트는 markup/API/ctest 만.
- **187 green 불변식** on default modules.
- orphan plant: `kill <pid>` for stale `vdsim_realtime` on 7401/7402 only.
- estimation noise = Simon Q/R. TUR tire confidential.
- VDS1 v4; GUI 재기동 after protocol change.
- git: push/force/tag — 명시 요청 시에만.

## 6. 관련 경로
- GUI: `gui/app.html` (`/`), `gui/index.html` (`/legacy`), `gui/server.py`.
- 설계: `docs/design/V0.2_*.md`, `V0.4_PLAN.md`.
- Cursor: `docs/design/CURSOR_USAGE.md`, `.cursor/rules/vdsim.mdc`.
