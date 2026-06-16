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
#209–#220, #221(phase A), **#222(B1)**. 타이어 역전 + L1~L5 통일 + 검증 + L5 spatial strut.

## 미완 (다음 할 일) — L5 6-DOF phase B2/C
설계 spec: `docs/design/L5_6DOF_MULTIBODY.md` (phase B1 DONE 기록 포함).
- **#222 B1 — DONE** — free_3d 에 opt-in spatial strut 추가 (`SolverParams::l5_spatial_suspension`,
  default false → penalty 경로 byte-stable, 기존 364 ctest 불변). per-corner unsprung mass +
  travel DOF(State.susp_compression/velocity 재사용, body-up strut 축). tire-spring on unsprung
  via `pen = pen_rigid - comp·(ez·n)`; strut spring/damper(preload→comp=0 static, F_susp≥0 top-out,
  bump/droop clamp) 가 6-DOF body 에 strut 축으로, in-plane tire force 는 rigid. body 병진질량
  비등방: strut 축=m_sprung, in-plane=total (평면 핸들링 L2/L3 일치). 검증: `L5Strut.*` 5개 green —
  settle(Fz_sum=14808 vs 14710 N, comp≈0, z=0.533m), heave 1.33Hz vs quarter-car 1.41Hz analytic.
- **#223 B2 (다음)** — `comp`(susp_compression) 를 `PrescribedCornerMotion.travel_z` 로 corner DAE
  (`create_hard_joint_dae_model`, `step_hard_joint_dae`) 에 넘겨 `WheelPose`(toe/camber) 받아
  tire `ContactInput.gamma`/toe 로. 14-DOF 배선(fourteen_dof_dynamics.cpp:205-278) 재사용.
- **#224 C** — 검증 evidence: ballistic jump 포물선/에너지, loop critical speed, flat 교차검증
  vs L2/L3, suspension travel vs L4. 통과 시 VALIDATION.md 의 L5 experimental 캐비엣 제거.

## 주의 / 함정
- **B2 는 B1 에 의존**: travel source(=`comp`)는 이제 B1 strut DOF 에서 나옴. B2 는 그 `comp` 를
  corner DAE 에 배선만 하면 됨. B2 도 strut 경로(flag on)에서만 동작.
- B1 은 opt-in flag(`l5_spatial_suspension`) 뒤에 격리됨 → default penalty 경로는 byte-stable,
  기존 stunt/L5/terrain/loop 테스트 전부 불변 통과. strut 검증은 `L5Strut.*` 신규 테스트로 격리.
- **phase C(#224) 미완**: ballistic jump/loop critical speed/flat cross-model(L5 strut vs L2/L3)/
  suspension travel vs L4 evidence 필요. 통과 전까지 L5 = experimental 유지. B1 의 in-plane 비등방
  질량은 flat 핸들링을 L2/L3 에 맞추려는 것 → C 의 cross-model 로 정량 확인 必.
- Python 으로 L5 직접 구동 불가(contact `penetration`·loop/ramp ground 미바인딩) → 검증은 C++ ctest.
- 대외비: 현대 tire 실측값 커밋 금지. 영상 git 금지.

## 관련 경로
- core/src/{tire_model,free_3d_dynamics,seven_dof_dynamics,bicycle_dynamics,pacejka_mf96}.cpp
- core/include/vdsim/{interfaces,tire_contact,multibody}.hpp
- docs/design/{TIRE_INTERFACE_INVERSION,L5_6DOF_MULTIBODY}.md, docs/theory/25_tire_contact_interface.md
- docs/evidence/tire/, tools/tire_inversion_evidence.py
