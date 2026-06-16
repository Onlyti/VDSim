# Handoff — L5 dynamic MBD suspension (coupled solve): design done, impl WIP (roll bug)

Continuation of the L5 spatial-strut track (see `HANDOFF_TIRE_L5.md` for B1/B2/C, all
landed). This handoff covers the move from the B1 lumped-1-DOF strut to a true coupled
multibody solve (Option 2), which is DESIGNED + reviewed but whose first implementation
has an unresolved roll-coupling instability. Repo is reverted to green (376/376).

## 목표
B1 strut(코너당 vertical 1-DOF 점질량 + one-way pseudo-force)은 루프(360° 회전·고-g)에서
에너지 ~85kJ 비물리적 주입 — 모델이 아니라 **수치적으로 비일관**(원칙 위반: 모델은 근사 OK,
수치는 정확해야). 목표는 코너당 실제 링키지 travel 1-DOF를 6-DOF 차체와 **완전 coupled**로 정확히
푸는 것(`M u̇ = Q − b`, 에너지 구조적 보존). high-fidelity MBD.

## 현재 상태 (repo: main green, 376/376)
- **설계·유도 완료·검토됨**: `docs/design/L5_MBD_DYNAMIC_COUPLING.md` (커밋 `4c4bc26`). 가정 1~5
  reviewed (점질량 unsprung now / link tensor inertia later; 1-DOF travel; spin은 tire side;
  wheel-rate spring now / motion-ratio next; anti-dive는 contact-patch 기하로 emergent).
- **구현 = WIP, 미완**: coupled 10-DOF solve를 free_3d strut 경로에 구현했으나 **roll 커플링
  버그**로 평지 정상선회가 발산 → green 위해 free_3d revert. WIP 보존:
  `docs/design/L5_MBD_COUPLED_WIP.patch` (e2d467d/4c4bc26 base의 free_3d diff; `git apply`로 복원).

## 완료
- B1의 에너지 버그 정량 진단: 루프 +85kJ 주입, dt-무관(2e-4=2e-5), 질량비등방 아님(테스트),
  바퀴 flywheel 아님(~13kJ), normal/strut 경로 ~69kJ + 접선 ~15kJ.
- 원칙 합의: 모델 단순화 OK, **수치/EoM은 정확·에너지보존** 必.
- Lagrangian 유도(질량행렬 M 블록, bias b, 일반화력 Q, contact-patch tire = anti-dive emergent).
- coupled solve 구현(WIP): **자유낙하 깨끗**(jump ballistic g 정확 — B1 누수 해결), heave/settle
  정확, loop-critical/camber/slip-balance 통과. 루프 단조 +85kJ 주입 사라짐.
- 디버그/audit 하니스: `apps/jump_demo/strut_demo_dump.cpp` (모드: jump/loop/loopfail/flatsteer,
  19+컬럼 telemetry: pose, per-wheel comp/Fz, ax/ay, wheel spin) + python 에너지 budget.

## 미완 (다음 할 일)
1. **roll 커플링 버그 수정** (블로커). coupled solve에서 평지 0.3g 선회가 roll 발산(전복) →
   static 부호/항 버그. 격리 테스트 제안(아래 주의 참고).
2. 수정 후 **에너지 audit 테스트**: 루프(throttle=0)에서 E_total(=KE+회전KE+중력PE+타이어스프링PE
   +서스펜션스프링PE)이 댐퍼·slip 외 보존(현재 누수). 단위테스트로 자유-부유 회전체 momentum/energy
   보존 체크부터.
3. 전 테스트 재통과: settle/ride/cross-model/jump/loop-critical/camber/Stunt.* + FreeLoopCompletesLap
   pitch 임계(현 coupled에서 0.25 vs 0.28, rebaseline 필요 가능).
4. 모델 refinement(후속): link tensor inertia(`I_u` 3×3), wheel rot inertia의 brake/accel
   anti-dive 기여(caliper-torque-on-knuckle), spring **motion ratio**(다음 타겟), DAE travel-path
   기하(w', w'')로 anti-dive/toe/camber emergent(현 WIP는 10a=vertical travel만).

## 주의 / 함정
- **버그 국소화**: heave/free-flight 정확(settle·jump 통과) → 병진/travel은 OK, **회전(roll) 커플링
  전용** 버그. 10× 댐핑에도 발산 → anti-damping 아님, static 부호/항.
- **물리 통찰**: B1은 stiff 타이어 위 강체(직접 penalty Fz at hub)라 roll stiff·안정. coupled는
  차체가 **soft 서스펜션** 위 → roll restoring이 contact↔comp↔spring 경유 → jacking/부호
  양성피드백에 민감. 0.3g 전복은 비물리 = 명백한 구현 버그.
- **격리 테스트 제안**: (a) 자유-부유(no contact) 회전체로 momentum/energy 보존 단위테스트(M·b만);
  (b) tire torque lever r_cp(contact patch)→r_wc(wheel center)로 바꿔 jacking 모멘트 격리;
  (c) `Gamma_i = s_i·F_tire`의 rolled-êz projection(jacking) 항 on/off; (d) M_tr/M_rq off-diagonal
  on/off. flatsteer 모드(`strut_demo_dump flatsteer`)가 roll/z 발산을 바로 보여줌.
- **contact 동결**: contacts는 step당 1회 query(RK4 stage 내 body roll에 rigid penetration 미반응,
  comp만 갱신) — B1·penalty도 동일. roll restoring이 comp 경유라 더 민감할 수 있음.
- WIP patch 적용: `git apply docs/design/L5_MBD_COUPLED_WIP.patch` (free_3d만 변경). Eigen Dense
  필요(이미 include). 10×10 LDLT solve.
- 대외비/영상 규약 동일. 데모/audit 산출물(mp4/png/csv)은 git 금지(스크립트만 추적).

## 관련 경로
- 설계/유도: `docs/design/L5_MBD_DYNAMIC_COUPLING.md`
- WIP 구현 diff: `docs/design/L5_MBD_COUPLED_WIP.patch`
- 대상 코드: `core/src/free_3d_dynamics.cpp` (strut 경로 derivatives())
- DAE 운동학(travel→pose, w'/w''/I_theta): `core/src/multibody_hard_dae.cpp`, `core/include/vdsim/multibody.hpp`
- 디버그/audit 하니스: `apps/jump_demo/strut_demo_dump.cpp` (+ animate_strut.py, plot_telemetry.py)
- B1/B2/C 핸드오프: `docs/HANDOFF_TIRE_L5.md`; 검증: `docs/evidence/l5/PHASE_C.md`
- 테스트: `tests/integration/test_l5_strut_validation.cpp`, `test_l5_driving.cpp`, `test_stunt.cpp`
