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
#209–#220, #221(phase A), **#222(B1)**, **#223(B2)**, **#224(phase C)**. 타이어 역전 +
L1~L5 통일 + 검증 + L5 spatial strut + corner DAE toe/camber + 검증 게이트 통과.
L5 6-DOF spatial-strut 트랙 종료 — strut 경로는 validated, default penalty 경로는 stunt demo 유지.

## 검증 완료 — phase C (strut 경로)
evidence: `docs/evidence/l5/PHASE_C.md` (suite `L5StrutValidation.*`, 376/376 ctest green).
- C1 flat cross-model vs L2: yaw rate 1.9%, lateral g 3.6%.
- C2 ballistic jump: fit g 0.5% 오차, horizontal speed drift 0.5%.
- C3 loop critical speed emergent: 1.15 v_crit ~20 rad, 0.70 v_crit ~2.7 rad.
- C4 corner camber vs standalone L4 DAE: < 1.3e-4 rad 일치.
- (B1) quarter-car heave 1.33 vs 1.41 Hz analytic.

## 참고 — 설계 spec
`docs/design/L5_6DOF_MULTIBODY.md` (phase B1/B2/C DONE 기록 포함).
- **#222 B1 — DONE** — free_3d 에 opt-in spatial strut 추가 (`SolverParams::l5_spatial_suspension`,
  default false → penalty 경로 byte-stable, 기존 364 ctest 불변). per-corner unsprung mass +
  travel DOF(State.susp_compression/velocity 재사용, body-up strut 축). tire-spring on unsprung
  via `pen = pen_rigid - comp·(ez·n)`; strut spring/damper(preload→comp=0 static, F_susp≥0 top-out,
  bump/droop clamp) 가 6-DOF body 에 strut 축으로, in-plane tire force 는 rigid. body 병진질량
  비등방: strut 축=m_sprung, in-plane=total (평면 핸들링 L2/L3 일치). 검증: `L5Strut.*` 5개 green —
  settle(Fz_sum=14808 vs 14710 N, comp≈0, z=0.533m), heave 1.33Hz vs quarter-car 1.41Hz analytic.
- **#223 B2 — DONE** — corner DAE 배선. `free_3d_attach_multibody(dyn, front_axle, topo, enable)`
  (14-DOF attach 미러). axle 당 DAE model 1개를 두 corner 가 공유(corner 별 HardJointCornerState),
  outer step 당 1회 `PrescribedCornerMotion{travel_z=comp[i], rate, steer_rack}` + 직전 corner
  하중으로 advance → `WheelPose` toe/camber (L/R sign flip) 를 tire `ContactInput.gamma` 와
  wheel-heading(toe=bump-steer) 로. strut on + DAE attach 시에만 활성 → 미부착 시 B1 불변.
  검증: `L5StrutDae.*` 3개 green — attach/enable, settle 하중유지, steer 시 DAE on/off 핸들링 분기
  (vy 0.863 vs 0.926 m/s).
- **#224 C — DONE** — 위 "검증 완료" 참조. VALIDATION.md 의 L5(strut) experimental 캐비엣 제거됨.

## 주의 / 함정
- **B1/B2 둘 다 opt-in 격리**: B1=`SolverParams::l5_spatial_suspension`(default false),
  B2=`free_3d_attach_multibody`(미부착이 default). 둘 다 off 이면 penalty 경로 byte-stable →
  기존 stunt/L5/terrain/loop 테스트 전부 불변 통과. strut/DAE 검증은 `L5Strut*.*` 신규 테스트로 격리.
- B2 의 toe/camber 는 corner DAE 가 prescribed travel(=`comp`, B1 strut DOF)로부터 계산. DAE 는
  outer step 당 1회만 advance(substep/RK4 stage 마다 X) — 비용·안정성 고려, 14-DOF 와 동일.
- **ax/ay_body_est 규약**: free_3d 는 이제 body-frame specific force(중력 제거, =L2/L3 Fy/m 규약)를
  보고 — 정상 선회에서 0 이 되는 body-velocity 미분이 아님. C1 cross-model 이 이 규약으로 L2 와 일치.
  (state 불변, 보고값만 수정 → penalty state byte-stable.)
- Python 으로 L5 직접 구동 불가(contact `penetration`·loop/ramp ground 미바인딩) → 검증은 C++ ctest.
- 대외비: 현대 tire 실측값 커밋 금지. 영상 git 금지.

## 관련 경로
- core/src/{tire_model,free_3d_dynamics,seven_dof_dynamics,bicycle_dynamics,pacejka_mf96}.cpp
- core/include/vdsim/{interfaces,tire_contact,multibody}.hpp
- docs/design/{TIRE_INTERFACE_INVERSION,L5_6DOF_MULTIBODY}.md, docs/theory/25_tire_contact_interface.md
- docs/evidence/tire/, tools/tire_inversion_evidence.py
