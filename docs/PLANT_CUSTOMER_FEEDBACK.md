# VDSim Plant — 첫 고객 피드백 (VLA thesis closed-loop plant)

작성 2026-06-18. 작성자 = VLA 졸업논문(T-IV) 세션 = `~/vla_design/`. VDSimPlant(Ld2 7DOF/MF2002, BETA #1)의
**첫 외부 고객**으로서, anticipatory chance-MPC의 P1 gate를 VDSim 위에서 돌린 실사용 경험 기록.

사용 규모(이 피드백의 근거): 한 세션에서 **수백 회 closed-loop rollout** — V0 14~32 m/s, 저-μ patch
0.20~0.50, patch 공간길이 22~90 m, single lane-change, det/frozen/antic 3-controller 비교. 한계·불가능
영역까지 의도적으로 몰아넣음(drift=\|α\|/α_peak 최대 5까지).

---

## 1. 아주 좋았던 것 (그대로 유지/홍보 권장)

### (a) ★ `mu_peak` / `alpha_peak` / `kappa_peak` = 이 plant의 킬러 신호
- 처음엔 `wheel.mu`(contact μ)만으로 friction-circle usage를 봤는데, GT plant에선 타이어가 용량을 못 넘어
  포화되니 usage가 의미 없이 ≤1로 막혀 **sliding을 못 보여줬다**(useGT>1 혼란). 이걸 피드백하니
  **load-dependent realized peak(`mu_peak`) + pure-slip peak(`alpha_peak`/`kappa_peak`)를 추가**해줬다.
- 이게 결정적이었다. **drift = \|α\|/α_peak (>1 = past-peak = 실제 sliding)** 가 내 핵심 안전 metric이 됐다.
  contact μ 기반 usage보다 물리적으로 정확하고 baseline 실패를 선명히 잡는다.
- **요청**: 이 신호들을 "한계주행 평가용 권장 safety metric"으로 튜토리얼/문서에 명시 홍보해라. 다른 제어
  연구자도 똑같이 contact-μ usage의 함정에 빠질 것이다.

### (b) load-dependent peak μ = "과도 demand → 진짜 슬립"의 물리
- 하중 이전으로 로딩 외륜 peak μ가 contact μ보다 낮아진다(예: contact 0.4 → realized peak 0.379, Fz≈8826N).
  이게 **컨트롤러 belief(bicycle, contact μ)와 plant 현실 사이 gap**을 만들어, deterministic baseline이
  "belief상 안전한데 실제 슬립"하는 물리를 생성한다. **Python bicycle soft-clamp로는 절대 못 얻는 fidelity**고,
  reviewer가 "장난감 plant"라 못 깐다. thesis의 σ_Fz(grip 불확실성)→chance-constraint 서사의 plant-side 근거.
- 확인 요청: 이 realized-peak 하강이 MF2002 .tir 의도와 정합하는지(내가 이 메커니즘에 결과를 의존하므로).

### (c) drop-in API = 진짜 plant swap
- `VDSimPlant(config, base_mu, friction_map, control_dt, substep_dt)` / `reset(state0)` / `step([δ,Fx]) → obs`
  가 요청 스펙과 거의 일치. MPC(acados) 한 줄 안 고치고 numpy bicycle plant를 통째 교체했다. 인터페이스 설계 훌륭.
- obs dict(차량 + per-wheel Fx/Fy/Fz/α/κ/μ/mu_peak/α_peak)가 closed-loop·figure에 바로 물린다.

### (d) 결정론 + 속도 + 수치 강건성
- RNG 없음 → 완전 재현. 5s rollout이 수십 ms → V0×μ×patch×mode 수백 run sweep을 가볍게 돌렸다.
- **한계 너머로 학대해도 안 터졌다**: μ=0.20, V0=28, drift 5까지 가도 NaN/발산 없이 coherent GT 유지.
  2-tier substep(control ZOH + sub-ms 내부적분)이 stiff한 wheel/torque 동역학을 잘 잡는다는 강한 방증.
- 부수효과: **plant를 GT로 신뢰할 수 있어서**, 내 컨트롤러의 조향 chatter가 plant가 아니라 *내 solver*
  문제임을 격리할 수 있었다. control 연구에 plant가 믿을 수 있는 ground-truth라는 게 정확히 필요한 가치다.

---

## 2. 마찰점 (고치면 다음 고객이 편함)

### (a) ★ Python 버전 — 가장 큰 진입장벽
- delivery note의 셋업(`cmake -B build`)이 만든 모듈이 **py3.8**이라 내 conda env(`vla`, py3.11)서 `import vdsim`
  실패. 결국 수동 재빌드 필요:
  `cmake -B build_vla -DVDSIM_BUILD_PYTHON=ON -DPython3_EXECUTABLE=<conda python> -Dpybind11_DIR=...`
  - `VDSIM_BUILD_PYTHON` 기본 OFF라 그냥 빌드하면 python 모듈이 안 나오는 것도 헷갈렸다.
  - **요청**: (1) py3.11 wheel 동봉 또는 (2) delivery note/튜토리얼 맨 앞에 "conda/py3.x 재빌드 레시피"를
    굵게. 단일 `build/`가 시스템 py3.8에 묶여 있으면 대부분의 ML/제어 사용자(conda py3.10+)가 막힌다.

### (b) acados와 `LD_LIBRARY_PATH` 충돌
- acados env(`source ~/opt/acados/env.sh`)와 같이 쓰면 import 시 lib 충돌 → `env -u LD_LIBRARY_PATH bash -c '...'`
  로 우회해야 했다. acados+VDSim은 MPC plant 조합의 전형이니 튜토리얼에 한 줄 경고 권장.

### (c) `Fx_total` → per-wheel 분배 불투명
- 제어입력 Fx가 "CG force intent"인데 wheel torque로 어떻게 분배되는지(drive_split, 제동 분배) obs/문서에서
  바로 안 보였다. 내부적으론 (ii) Fx_total→wheel torque(native MF96 combined slip)로 정리된 걸로 이해하나,
  **적용된 per-wheel 종력 분배를 obs나 doc에 명시**하면 종방향 거동 디버깅이 쉬워진다.

---

## 3. 기능 요청 (우선순위)

- **P1 — split-μ / per-wheel friction map**: 현재 `friction_map=[(x0,x1,μ)]` = 종방향 X-band(좌우 동일).
  split-μ(좌우 다른 μ)나 (x,y) map이면 split-μ 제동/안정성 시나리오가 가능(ABS/ESC급 연구 수요).
- **P1 — obs에 roll angle / 하중이전 성분 노출**: per-wheel Fz는 있으나 roll angle φ가 obs에 없다. 7DOF/14DOF
  검증과 σ_Fz(load-transfer) 서사에 직접 쓰인다.
- **P2 — 선택적 1차 조향 actuator lag**: 현재 δ는 road-wheel ZOH 직접(=MPC baseline엔 정확히 맞음). robustness
  연구용으로 `steer_tau` 옵션(끄면 현 동작)이 있으면 실차 격차 연구 가능.
- **P2 — μ patch의 smooth 전이 옵션**: 현재 box(계단) μ. 제어기 입장에서 계단 μ는 수치적으로 거칠 수 있어,
  tanh 전이 폭(transition width) 인자가 있으면 좋다.
- **★ P1 — 표준 시험기동 track 프리셋**: ISO 3888-1/3888-2(moose), sine-with-dwell, FMVSS 126 등 표준 maneuver를
  cone-gate 정의(차폭 b 연동: 구간길이 12/13.5/11/12.5/12 m, 폭 1.1b+0.25 / b+1 / 1.3b+0.25, 횡 offset)로
  프리셋 제공 + 그 위에 μ patch 오버레이. 지금은 고객(나)이 규격을 손으로 재현 중인데, 차량동역학 sim이
  표준 시험을 프리셋으로 주면 (a) 규격 정확성 보장(reviewer 방어), (b) 차량마다 b로 자동 스케일, (c) cone-hit
  판정·통과속도 산출까지 표준화. = control/AV 연구자 공통 수요. **VDSim이 제공하면 첫 고객이 가장 반길 기능.**

---

## 4. 내가 돌린 검증 (VDSim 신뢰도 근거로 공유)

- **dry 정합**: δ0.04 → r≈0.205 (understeer 정합), 좌선회 시 FzFR>FzFL(하중이전 방향 정확), 정적 전축 Fz≈7050 N
  (Ioniq5 제원 정합).
- **patch 거동**: μ patch서 컨트롤러가 grip 초과 demand하면 per-wheel α가 α_peak 넘어가며 drift>1(슬립) =
  요구한 "clamp 아닌 grip loss" 정확히 작동.
- **결정론**: 동일 입력 재현 OK.
- **envelope**: V0 14~32 / μ 0.20~0.50 / patch 22~90 m 전반에서 물리적으로 일관(비물리 발산 없음). deterministic
  baseline은 전 영역서 최악, anticipatory는 grip 마진 최고 — 기대한 단조 분리가 plant 위에서 깨끗이 재현됨.

---

## 5. 한 줄 총평
첫 고객으로서: **`mu_peak`/`alpha_peak` 노출과 load-dependent realized-peak 물리가 이 plant를 control-research용
GT로 진짜 쓸 만하게 만든다.** 유일한 실질 장벽은 **Python 버전(py3.8 고정) 빌드** — 이거 하나만 매끄럽게 하면
다음 고객의 첫인상이 크게 좋아진다. drop-in·결정론·한계영역 수치강건성은 그대로 강점.
