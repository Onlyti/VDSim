# VDSim — agent 진입점

본인 창업 open-core vehicle-dynamics 시뮬레이터. 정확·검증된 차량동역학 + hardpoint 설계검증 + FMI 2.0. 외부 시각화/센서는 위임(CARLA 등).

## 작업 전 읽기
1. README.md (영) / README.ko.md — layout·positioning
2. docs/ + mkdocs (https://onlyti.github.io/VDSim/) — 이론·리포트

## 구조
- core/ — libvdsim_core (C++17): Ld1 Bicycle/Ld2 7DOF/Ld3 14DOF + Pacejka MF96 + Ld4 hardpoint. ISO 8855 RH
- python/ — pybind11 (vdsim) · tools/kinematics/ — offline hardpoint solver + Adams importer + GUI
- apps/ examples/ tests/ fmi_export/ carla_integration/
- 3갈래 런타임(docs/design/RUNTIME_ARCH.md): gui/(시각화 웹) · cosim/(제어·신호 UDP, canonical VDS1) · python/vdsim_lab+tools/vdsim_batch(sync API/배치) · builder/(저작)

## 빌드
- CMake (CMakeLists.txt). docs = mkdocs (github.io 배포)
- 공통: ~/.claude/CLAUDE.md (wheel FL=0, ISO 8855)

## 거버넌스 — demand-gate (2026-06-25)
신규 fidelity/물리 기능 PR은 **VDSim-PO** demand-gate 승인 필요 — 그것을 당기는 named puller(고객 피드백·세그먼트 도입 실패 사례)가 명시될 때만 착수. puller 없는 fidelity = Frozen. 로드맵 SoT·tier·우선순위는 PO 소유(`VDSimMarketing/ROLE.md`, `04_roadmap_revision_proposal.md`). thesis 연구인프라(linearize/monte_carlo/estimator-in-loop/plant)는 thesis puller 존재 → gate 면제.
