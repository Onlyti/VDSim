# Handoff — tire inverted interface (done) + L5 6-DOF (phase A done, B/C pending)

## 목표
타이어 추상화를 "kinematics-in / wrench-out" 으로 역전해 새 타이어/transient/camber를 dynamics
편집 없이 한 클래스로 추가. 더해서 L5(free_3d)를 L4의 full 6-DOF multibody sibling 으로 재정초.

## 현재 상태 (전부 origin/main, 364/364 ctest green, 트리 clean)
- 타이어 inverted 인터페이스(`evaluate` / `advance_bristle` / `advance_relaxation`)가 `ITireModel`
  base 에 1회 정의(`core/src/tire_model.cpp`), force law 는 virtual `compute()` dispatch. base 가
  `params_` 소유(`on_initialize()` hook). pacejka / linear / MF2002 전 백엔드 적용.
- 소비: **L1 bicycle / L2 seven_dof / L3 14-DOF(seven_dof wrap) / L5 free_3d 전부 전환 완료.**
  L1 은 per-tire individual contact(축당 0.5·Fz evaluate ×2), L5 는 contact-frame `v_long_k`
  (loop-denom floor 제거) + LuGre bristle 적분 버그 수정.
- 검증자료: `docs/evidence/tire/EVIDENCE.md`(Re/camber/phantom/inversion 등가) +
  `EVIDENCE_L1_L5.md`(L1/L5 phase-A 100/100 canary 결과요약). 재생성:
  `python3 tools/tire_inversion_evidence.py`.

## 완료
#209–#220, #221(phase A). 타이어 역전 + L1~L5 통일 + 검증.

## 미완 (다음 할 일) — L5 6-DOF phase B/C
설계 spec: `docs/design/L5_6DOF_MULTIBODY.md` (phase B 구현설계 포함, 코드 수준).
- **#222 B1 (먼저)** — free_3d 에 spatial sprung/unsprung strut 동역학 신규 추가. 현재 free_3d 는
  서스펜션이 없고 penalty-at-hub 로 차를 떠받침. per-corner unsprung mass + body-frame strut
  travel DOF `z_strut`(+rate) 추가, strut spring/damper 를 6-DOF Newton-Euler 에, tire-spring 을
  unsprung 에. 14-DOF `derivatives_vertical`(fourteen_dof_dynamics.cpp:345-424) 이 참고 모델
  (heave/roll/pitch+unsprung 를 평면 기준으로 함 → 6-DOF body-fixed strut 축으로 일반화).
- **#223 B2** — `z_strut` 를 `PrescribedCornerMotion.travel_z` 로 corner DAE
  (`create_hard_joint_dae_model`, `step_hard_joint_dae`) 에 넘겨 `WheelPose`(toe/camber) 받아
  tire `ContactInput.gamma`/toe 로. 14-DOF 배선(fourteen_dof_dynamics.cpp:205-278) 재사용.
- **#224 C** — 검증 evidence: ballistic jump 포물선/에너지, loop critical speed, flat 교차검증
  vs L2/L3, suspension travel vs L4. 통과 시 VALIDATION.md 의 L5 experimental 캐비엣 제거.

## 주의 / 함정
- **B2 는 B1 에 의존**: travel source 가 없으면 DAE 는 no-op. 반드시 B1 먼저.
- B1 은 차를 떠받치는 방식을 교체 → 모든 stunt/L5/terrain 테스트 rebaseline + loop 가 attitude
  sweep 내내 안정해야 함. **권장: opt-in flag 로 strut 경로 추가(default penalty 유지 → 기존
  baseline byte-stable), 신규 settle/ride-frequency 테스트로 격리 검증.** VDSim 의 reff/drivetrain-v2
  opt-in 패턴 따르기. 무검증 default 전환 금지(verification-first).
- Python 으로 L5 직접 구동 불가(contact `penetration`·loop/ramp ground 미바인딩) → 검증은 C++ ctest.
- 대외비: 현대 tire 실측값 커밋 금지. 영상 git 금지.

## 관련 경로
- core/src/{tire_model,free_3d_dynamics,seven_dof_dynamics,bicycle_dynamics,pacejka_mf96}.cpp
- core/include/vdsim/{interfaces,tire_contact,multibody}.hpp
- docs/design/{TIRE_INTERFACE_INVERSION,L5_6DOF_MULTIBODY}.md, docs/theory/25_tire_contact_interface.md
- docs/evidence/tire/, tools/tire_inversion_evidence.py
