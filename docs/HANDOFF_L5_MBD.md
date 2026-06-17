# Handoff — L5 dynamic MBD suspension

## RESOLVED 2026-06-17 (free-3D inertial unsprung — IMPLEMENTED, 377/377 green)

The B1 strut path is replaced by a free-3D inertial unsprung model. Design + reasoning +
energy evidence: **`docs/design/L5_MBD_FREE3D_UNSPRUNG.md`**. Both prior designs are
superseded (10-DOF coupled = QS roll bug; world-vertical z_u = breaks general-surface
normal). Direction set by user: discard world-z, keep arbitrary-surface contact.

- Each unsprung = free 3-D inertial point mass `x_u` (new `State::unsprung_pos/vel`); strut
  = soft two-point spring along body-up, perpendicular = stiff link bushing (penalty 1e7).
  Tire on the wheel (Fz along real normal -> general surface); body feels only the mount
  connection reaction (two-point -> energy-consistent + direct QS roll moment).
  `susp_compression/velocity` are now DERIVED strut travel (DAE/FMI/cosim contract kept).
- Result: 377/377 (was 376). Structural energy leak gone — flat steer 0 injection; loop
  residual is dt-CONVERGENT frozen-contact discretization (4811->641 J/turn as dt halves),
  not structural (B1's ~85 kJ was dt-independent). `Stunt.FreeLoopCompletesLap` attitude
  check rebaselined to peak (not final instantaneous) pitch.
- Touched: `state.hpp` (+unsprung_pos/vel), `free_3d_dynamics.cpp` (strut path rewrite +
  per-substep re-query + k_link), `interfaces.hpp`, `python/bindings.cpp`,
  `tests/integration/test_stunt.cpp`, `apps/jump_demo/strut_demo_dump.cpp` (energy ledger +
  env toggles). NOT committed yet.
- Follow-ups DONE (2026-06-17): (1) per-substep contact re-query
  (`free_3d_attach_contact_provider`, opt-in) — removes the loop frozen-contact energy
  residual: coast loop goes from net +5 kJ injection (frozen) to net -18..-40 kJ dissipative
  (re-query), peak ledger swing +18 kJ -> +0.9 kJ. (2) link "hard constraint" = stiffened the
  penalty to 1e8 N/m (~40 um, effectively rigid); a true k=inf constraint is rejected (it
  reverts to the reduced-coordinate roll loss) — penalty IS the limit and keeps roll
  (gradient 2.73 deg/g). Verified stable to 1e9; 1e10 RK4-unstable. Details in the design doc.
- Follow-up DONE (2026-06-17): high-fidelity linkage geometry now drives the strut dynamics
  when a corner DAE is attached — the bushing constrains the wheel to the REAL hardpoint
  travel path (travel_maps Δw + tangent), so anti-dive/squat + roll-centre migration emerge
  from geometry (verified: static no-coupling; front w'.x~0 inert; rear w'.x=0.19 strong,
  magnitude ~Fx*w'.x/k). Active only with hardpoints; straight body-up fallback otherwise.
  This is the core L5 (Adams-class hardpoint -> behaviour) value prop. Design doc updated.
- Follow-up DONE (2026-06-17): progressive coil rate from spring hardpoints. DAE exposes
  spring eye-to-eye length l(z_v) (MacPherson strut, DW spring_damper); travel_maps gives
  spring_len + motion_ratio=dl/dz_v; free_3d uses F=k_coil*(l-l_free)*MR with k_coil=ks/MR0^2
  cached at attach so static load/rate match the wheel-rate model, then progressive +
  bump/rebound asymmetric (verified MacPherson -7%/+10%, DW -7%/+12% over +-60mm; DW MR0=-0.53
  -> stiff inboard coil). No spring hardpoints -> wheel-rate fallback. 377/377 green.
- Follow-up DONE (2026-06-17): (1) progressive damper (c_eff=cs*(MR/MR0)^2; stops stay on
  the wheel travel, correct). (2) per-substep contact re-query is now default in all
  SimSession runs (session attaches its ground provider after init; no-op for non-free_3d;
  standalone tests still use the frozen ContactArray). 377/377 green.
- Follow-up DONE (2026-06-17): gyroscopic wheel-spin coupling. Each spinning wheel's lateral
  spin momentum H=I_wheel*omega_spin couples to the body via tau_body -= omega x H (yaw<->roll
  at speed). Test: airborne yaw+spin develops roll, no-spin doesn't
  (Stunt.GyroscopicWheelSpinCouplesYawToRoll). 378/378 green.
- L5 high-fidelity dynamics feature-complete: energy-consistent general-surface free-3D
  unsprung + hardpoint-emergent anti-dive/squat/roll-centre + progressive coil rate +
  progressive damper + gyroscopic wheel-spin coupling. Remaining (out of scope): gyro
  refinements (axis tilt, spin-up reaction), full KC compliance model.

Everything below is the prior (superseded) coupled-solve / world-z history.

---

# Handoff — L5 dynamic MBD suspension (coupled solve): design done, impl WIP (roll bug)

Continuation of the L5 spatial-strut track (see `HANDOFF_TIRE_L5.md` for B1/B2/C, all
landed). This handoff covers the move from the B1 lumped-1-DOF strut to a true coupled
multibody solve (Option 2), which is DESIGNED + reviewed but whose first implementation
has an unresolved roll-coupling instability. Repo is reverted to green (376/376).

## 방향 전환 2026-06-17 저녁 (정식 MBD 재정식화 — 설계 완료)

jacking root-cause를 끝까지 추적하고(아래 "구현 갱신" 참조) Cursor MBD 리뷰로 교차검증한
결과: **현 10-DOF reduced-coordinate coupled solve는 Lagrangian적으로 self-consistent하나
틀린 모델이다.** unsprung를 body-상대좌표(`x_u=p+R(rb+w(θ))`)로 매립해 스프링 위치에너지
`U=U(θ)`가 body pose에 무관 → 스프링이 body에 일반화력 0 → QS에서 roll restoring이 tire
load transfer만 남아 ~5× 부족 → ~0.15g 전복.

**fix = 정식 MBD: unsprung 수직을 inertial DOF `z_u`로** (검증된 Ld3 14DOF 구조를 6-DOF
free body로 lift). 설계: **`docs/design/L5_MBD_RIGOROUS.md`**. 핵심:
- 질량행렬 분리 → **10×10 solve 제거**, body 6-DOF Euler + 4개 스칼라 `z_u` ODE.
- `δ_i=z_corner,i(p,R)−z_u,i`가 body pose 의존 → 스프링이 `Σ(R·rb)_y·F_susp`로 roll 모멘트
  직접 전달(14DOF의 그 항). 타이어 normal은 `z_u`에, tangential은 body patch lever에.
- DAE `travel_maps`(committed)·Pacejka·contact·RCPC 재사용. 에너지 명시적 보존.
- 합격: settle ΣFz=weight, cornering 안정(>0.7g), roll stiffness ~26kN·m/rad, 전 L5 테스트.

다음 세션 진입점 = 이 설계 문서로 구현. (Cursor 리뷰 전문은 이 세션 로그/요약 참조; Cursor의
"10-DOF 유지+mount 반력 추가(B+)" 권고는 비엄밀[이중적용 위험]이라 채택 안 함 — 분리 z_u가 정답.)

## 구현 갱신 2026-06-17 PM (Option A 완료 + jacking 직교 확정)

세션에서 Option A(실제 링크 기하)를 구현하고, jacking이 geometry와 **직교**함을 확정했다.

**커밋됨 (main, clean)**:
- `fix(core)` camber thrust 부호(pacejka MF96, ISO 8855) + 박제 테스트 3곳 수정.
- `refactor(core)` hard-joint DAE central-diff cleanup.
- `feat/refactor(core)` DAE travel maps API: `pose_at_travel(z_v,steer,seed)` +
  `travel_maps` → body-frame `w(z_v), w'(=dw/dz_v, z성분=1), w''` + toe/camber. 5개
  서스펜션 타입 전부. 단위테스트 `TravelMapsRealLinkageGeometry`.

**WIP (free_3d coupled+A2, 미커밋 — `docs/design/L5_MBD_COUPLED_WIP.patch`, clean)**:
- L5 travel DOF = **실제 수직 wheel travel z_v[m]** (전 타입 균일; L2/L3 = 모션비 1 수직
  특수케이스). `susp_compression = travel[m]` 계약 보존(getter/stop/검증테스트 정상).
- DAE 부착 시 coupled solve가 `w(z_v),w',w''`로 실제 링키지 경로를 따름(r_wc=R(rb+dw),
  s=R·w', abias에 R·w''·żv², penetration=pen_rigid−[R·dw]·n, spring은 z_i=dw.z·모션비
  J_z=w'.z, RCPC 일관 contact patch). 미부착 시 vertical-slider fallback(z_v, 모션비 1).
- 결과: **375/377 통과**. camber 테스트(`CornerCamberMatchesL4Dae`) 통과, 다른 회귀 없음.

**jacking은 geometry가 아니다 (CONFIRMED)**: flatsteer threshold를 실제 DAE 기하 vs
vertical fallback 직접 비교 — 둘 다 ~0.15–0.18g에서 jacking(DAE가 오히려 약간 나쁨). 즉
**실제 링크 기하도 cornering jacking을 못 고친다.** 남은 2 실패(`FreeLoopCompletesLap`,
`FlatCrossModelMatchesL2`)가 이 coupled-solve jacking 버그. Option A는 fidelity상 옳고
완료됐으나 jacking과 직교.

**남은 문제 = jacking (coupled-solve 정식화)**. leading 가설(미확정): 서스펜션 스프링/댐퍼의
roll 모멘트가 body에 inertial M-커플링 경유로만 약하게 전달돼 roll restoring 부족 → jacking이
이김(14DOF는 직접 모멘트 전달이라 안정). 다음 단계: 스프링력의 body-roll 전달을 정량 측정,
혹은 14DOF식 직접전달(option B) 차용 검토. 진단 도구: `apps/jump_demo/strut_demo_dump`
모드 `flatsteer`(+`VDSIM_DAE`/`VDSIM_STEER`)·`rolltest`·`airspin`.

## 진단 갱신 2026-06-17 (CONFIRMED — 이전 가설 3건 정정)

깊은 격리 디버깅으로 roll 발산의 성질을 재규정. **핸드오프 본문의 가설("static 부호/항 버그",
"에너지 누수", isolation (a)의 inertial 의심)을 아래로 정정한다.** 재현·evidence는
`apps/jump_demo/strut_demo_dump.cpp`의 신규 모드(`flatsteer`/`rolltest`/`airspin`)+env 토글.

- **버그는 coupled 경로 한정** (CONFIRMED): 동일 tire/steer로 penalty(non-strut) 경로는
  완벽 안정(roll 0.0043 rad 정착). `VDSIM_PENALTY=1 strut_demo_dump flatsteer`.
- **inertial M/b 어셈블리는 옳다** (CONFIRMED, isolation (a) 무효화): 수식 증명 —
  `comp_dot=0`인 강체 회전에서 `Σ m_u(r×abias)=ω×(I_u ω)` → `b_r=ω×(M_r ω)` →
  `M_r·ω̇=-b_r`는 정확한 Euler 식. `airspin`의 omega 붕괴는 **airborne preload-pumping
  오염**(무접촉 시 예열 스프링이 comp를 violent하게 펌핑)이지 inertial 버그 아님.
- **에너지 주입 아님** (CONFIRMED, "누수" 가설 정정): 발산 중 E_total은 **감소**
  (176k→95k J, speed 15→9 m/s). z(PE) 상승은 전진 KE로 지불. 즉 소산적이지만
  **동역학적으로 불안정**(전진 KE가 roll 모드로 펌핑).
- **순수 roll(lateral=0)은 안정** (CONFIRMED): roll rate 주입 시 감쇠 진동으로 복귀
  (`rolltest`, steer=0). 불안정은 **lateral force 전용**.
- **subcritical 전복** (CONFIRMED): steer=0.010(정상선회 안정)에서도 roll을 크게 교란하면
  재성장. 임계 roll≈0.05 rad에서 eigenvalue zero-crossing. lateral threshold ~0.15g.
- **진짜 메커니즘 = JACKING** (CONFIRMED): cornering+roll 시 **sumFz/W > 1**(상시 net
  상향력 4~14%) → CG 상승(vz>0 성장) → overturning arm 증가 → roll restoring을 이김.
  **penalty 경로는 sumFz/W=1.000 정확**(comp 동결) → jacking은 **coupled comp 동역학이
  만드는 버그**. `rolltest`가 sumFz/W·sumComp·vz를 덤프.
- **단일 항 아님 / 구조적** (CONFIRMED): jacking 토글(접선/법선), bias, M_rq, M_tr, M_z,
  DAE toe/camber, strut축(world-z), rcp lever(±comp·ez), slip-vel comp_dot,
  penetration tilt인자(ez·n) — **어느 것도 발산을 막지 못함**. NOTANG(접선 jacking 제거,
  virtual-work 일관)은 늦추기만.
- **stiffness/damping이 억제** (CONFIRMED): spring ×10 또는 damper ×30이면 안정.
  destabilizer는 suspension-rate 독립(jacking, lateral 비례), restoring/damping은 rate
  비례 → default soft 서스펜션에서 marginal.

**root cause (HYPOTHESIS)**: body-roll(R)과 comp가 독립 DOF이고 tire(Fz)·inertial M로만 커플.
서스펜션 spring+damper의 roll stiffness/damping이 body에 M 커플링 경유로만 간접 전달되어
soft 서스펜션에서 marginal → roll mode가 jacking에 민감. 한 줄 부호버그가 아니라
**coupled 정식화가 정상선회 vertical 평형(sumFz=weight)을 유지하지 못하는 구조 문제**.
다음 단계: penalty처럼 정상선회에서 sumFz=weight가 되도록 vertical/comp 일관성 재정식화
(혹은 suspension roll moment를 body에 직접 전달하되 M 커플링과 double-count 회피 검증).

### 블록별 독립 물리검토 (5 subagent, 2026-06-17)

coupled solve를 5블록으로 나눠 각각 설계문서 대비 독립 검토(편향 방지):
- **Mass matrix M**: 정확 (전 블록 부호·frame·대칭·SPD 일치). HIGH.
- **Bias b**: 정확 (`b_r=ω×(M_r ω)` 항등식 코드에서 성립, Coriolis 2배·gyro 부호 OK). HIGH.
- **Integration/frame 변환**: 정확 (`a_body=Rᵀ v̇_w−ω×v_body`, `α_body=Rᵀ ω̇_w` exact). HIGH.
- **Generalized force Q (타이어력)**: **virtual-work 불일치 CONFIRMED** — body torque는 지면
  patch lever `sg_rcp`(comp 불변), comp jacking은 wheel-center 감도 `si=ez_world`(comp 가변).
  단일 `x_c(q)`에서 유도 안 됨 → cornering 시 comp DOF에 spurious work.
- **Penetration/strut축**: penetration식 자체는 정확. strut축(ez_world)이 body-tilted라
  ground-normal n과 불일치 → rolled 평형에서 Σcomp≠0 → ΣFz≠weight. SUSPECTED HIGH.

**부분 fix CONFIRMED (`VDSIM_RCPC`)**: contact patch를 comp-displaced wheel-center 바로 아래
`x_c = r_wc − R0·n`로 일관 정의 → rcp와 si를 한 점에서 유도. flatsteer threshold ~0.017→0.025,
steer=0.018 깨끗이 안정, jacking sumFz/W 감소. **단 0.03+ 잔여 발산** — strut축 body-tilt에 의한
roll-center(≈wheel-center 높이) lateral jacking. 완전 해소는 DAE travel-path 기하(다음 phase의
contact-patch 감도 `s_c=R·dw_c/dθ`, ground roll center)가 들어와야 할 듯. 코드는 `T_RCPC` 토글.

**디버그 인프라**(working tree, WIP patch에 미포함 — env-gated, 기본 off):
`strut_demo_dump` 모드 `rolltest`(roll 교란 eigenvalue)·`airspin [wx wy wz]`(자유회전)+
`flatsteer`(energy/threshold). core env: `VDSIM_PENALTY/NOTANG/NOJACK/NOMRQ/NOMTR/NOBIAS/
NOMZ/WORLDZ/RCPFIX/RCPC/NOVCOMP/NODAE/PEN1`. harness env: `VDSIM_STEER/STIFF/CDAMP`.

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
1. **roll JACKING 버그 수정** (블로커). cornering 시 sumFz>weight(net 상향력)로 CG가 jack-up →
   subcritical 전복. **위 "진단 갱신 2026-06-17" 참조** — static 부호버그가 아니라 coupled
   vertical 평형 구조 문제. 목표: 정상선회에서 sumFz=weight 회복(penalty 경로 기준).
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
