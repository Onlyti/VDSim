# Actuator/Sensor Nonlinearity & Delay — Reference

actuator/sensor 의 물리 비선형성·지연을 VDSim 에 반영하기 위한 조사 레퍼런스.
모델 형태(식), typical parameter, 학술/산업 근거(DOI), 그리고 control 사다리
Lc1-Lc8 별 적용 방식을 정리한다. 구현 전 spec 근거 문서이며, thesis 인용 시
flag 표시된 항목은 publisher 페이지로 재확인할 것.

신뢰도 표기: [S] 출처 확인 수치 · [E] engineering 추정 · [F] citation 미확정(재확인 필요).

---

## 1. 적용 항목 카탈로그 (모델 형태 + typical)

handling 영향도: P0 필수 · P1 선택 · P2 대역밖(NVH/feel flag).

| class | 항목 | 모델 (ASCII) | typical | P |
|---|---|---|---|---|
| delay | transport delay (pure) | `y[k]=u[k-N]`, `N=round(L/dt)`, fractional `f=L/dt-N` | CAN+compute 수~100 ms | P0 |
| delay | first-order lag | `tau*dy/dt + y = u` | tau 0.05–0.15 s | P0 |
| delay | FOPDT | `G(s)=K e^{-Ds}/(1+tau s)` | powertrain D=0.1 s, tau=0.15 s | P0 |
| delay | Pade(1,2) of e^{-Ds} | `(1-Ds/2)/(1+Ds/2)` 등 | MPC/linearization 용 | P1 |
| hyst | steering friction (LuGre) | `dz/dt=w-(|w|/g)z`, `T=s0 z+s1 dz/dt+s2 w`, `g=Tc+(Ts-Tc)e^{-(w/ws)^2}` | Tc 0.5–2 Nm, Ts/Tc 1.3–2 | P0 |
| hyst | backlash/lash | deadband-hysteresis, gap `2b` | steer 0.05–0.3°@wheel; driveline gap | P1 |
| hyst | brake mu(T) | `mu=mu(T)` lookup | 0.4→0.6(180°C)→0.2(350°C) | P0(long) |
| hyst | EHB pressure hysteresis | 위치의존 Coulomb+viscous (reversal jump) | — | P1 |
| nonlin | saturation + rate limit | `clip(u,-umax,umax)`, `|du/dt|<=R` | steer rate ~2 rad/s | P0 |
| nonlin | dead-zone | `sgn(u)*max(|u|-d,0)` | brake fill, throttle tip-in | P1 |
| nonlin | drive/coast 비대칭 (regen) | drive≠coast torque map | EV regen 한계 | P1 |
| noise | motor torque ripple/cogging | `sum_k a_k cos(k*theta_e+phi_k)`, k=6,12 | 3.5–10% rated | P2 |
| noise | PWM/inverter ripple | switching 8–20 kHz | few % | P2 |
| noise | sensor quantize+noise | round-to-step + white | steer 0.05–0.1° | P1(steer)/P2 |

handling 대역(~1–3 Hz) 유효 = P0: steering friction(LuGre), 전 채널 saturation+rate
limit, brake mu(T)+pressure dead-zone, powertrain FOPDT. ripple/cogging/PWM/고주파
noise(P2)는 회전관성 low-pass 로 무의미.

---

## 2. WEVJ 2024 (Lee & Jo) — powertrain delay 실측 파라미터

Lee, J.; Jo, K. "MPC with Powertrain Delay Consideration for Longitudinal Speed
Tracking of Autonomous EVs," World Electric Vehicle Journal 15(10):433, 2024.
DOI 10.3390/wevj15100433. [S]

- plant 측 actuator(CarMaker IONIQ5): pure delay D=0.1 s (queue/ring buffer) +
  first-order lag tau=0.15 s, throttle/brake 입력 직전 적용. 실측 총지연 ~0.2 s.
- MPC 측 보상: state augmentation `x=[v,F]→[v,F_lag,F_{Nd-1},...,F_0,F]`,
  N_d=5, dt=0.02 s. Taylor 선형화 → QP → HPIPM(BLASFEO).
- MPC param: dt 0.02, T 100, N_d 5, tau 0.15, Q diag(300,0,..), R 1e-4,
  F_max 10819 N, F_min -14485 N.
- IONIQ5 (public): m 2300 kg, f 0.015, rho 1.21, A 2.88 m², Cd 0.35, r_wheel 0.32 m.
- 결과: real-vehicle mean speed error 0.54 km/h, compute 1.32 ms.

이 값들이 VDSim powertrain actuator 채널 + thesis MPC augmentation 의 seed.

---

## 3. Bibliography

### Steering (EPS/SBW)
1. Beal, C.E.; Brennan, S. "Modeling and friction estimation for automotive steering torque at very low speeds." Vehicle System Dynamics 59(3) 458–484, 2021. DOI 10.1080/00423114.2019.1708416. [S] — load·rate 의존 steering friction, LuGre, 실차 ID. HIGH.
2. Canudas de Wit, C.; Olsson, H.; Åström, K.J.; Lischinsky, P. "A new model for control of systems with friction." IEEE TAC 40(3) 1995. DOI 10.1109/9.376053. [S] — LuGre 원전. 모델 토대.
3. Al-Bender, F.; Lampaert, V.; Swevers, J. "The Generalized Maxwell-Slip Model." IEEE TAC 50(11) 2005. DOI 10.1109/TAC.2005.858676. [S] — presliding hysteresis + nonlocal memory. MED.
4. Huang, X.; Wang, J. "Identification of Ground Vehicle Steering System Backlash." ASME JDSMC 135(1) 011014, 2013. DOI 10.1115/1.4007558. [S] — steering lash 실측 ID. HIGH.
5. Zhou, X.; Wang, Z.; Shen, H.; Wang, J. "Robust Adaptive Path-Tracking Control ... Steering System Backlash." IEEE T-IV 7(2) 2022. DOI 10.1109/TIV.2022.3146167. [S] — backlash↔tracking + inverse 보상. HIGH.
6. Liang, X. et al. "A novel steer-by-wire system with road sense adaptive friction compensation." MSSP 169:108741, 2022. DOI 10.1016/j.ymssp.2021.108741. [S] — SBW LuGre FF 보상. HIGH.
7. Lee, M.H. et al. "Improvement of the steering feel of an EPS ... by torque map modification." J. Mech. Sci. Tech. 19(3) 2005. DOI 10.1007/BF02916127. [S] — assist-map nonlinearity + dead-zone. MED.
8. (flag) "Adaptive steering control with dead-zone compensation for autonomous vehicles." MSSP 2025, S0888-3270(25)00373-5. [F] — SBW dead-zone↔tracking.

### Brake (hydraulic / EHB / EMB)
9. Han, W.; Xiong, L.; Yu, Z. "Braking pressure control in EHB based on pressure estimation with nonlinearities and uncertainties." MSSP 131, 2019. DOI 10.1016/j.ymssp.2019.02.009. [S] — EHB pressure-position hysteresis + LuGre. HIGH.
10. Han, W. et al. "Integrated Pressure Estimation and Control for EHB of EVs ..." IEEE/ASME T-Mech 2023. DOI 10.1109/TMECH.2021.3119414. [S] — actuator 비선형↔longitudinal decel tracking. HIGH.
11. Park, G.; Choi, S.B.; Hyun, D. "Clamping force estimation based on hysteresis modeling for EMB." Int. J. Automotive Technology 18(5) 2017. DOI 10.1007/s12239-017-0086-5. [S] — clamp-force/motor-angle hysteresis + clearance dead-zone. HIGH.
12. Park, G.; Choi, S.B. "Clamping force control based on dynamic model estimation for EMB." Proc. IMechE Part D 2017. DOI 10.1177/0954407017738394. [S] — EMB friction + dead-zone. HIGH.
13. Li, Y. et al. "A Review of EMB System: Structure, Control and Application." Sustainability 15(5):4514, 2023. DOI 10.3390/su15054514. [S] — EMB 비선형 taxonomy(survey). HIGH(orient).
14. Bellini, C. et al. "Temperature Influence on Brake Pad Friction Coefficient Modelisation." Materials 17(1):189, 2023. DOI 10.3390/ma17010189. [S] — mu(T) fade. HIGH(decel gain).

### Powertrain / driveline
15. Lagerberg, A.; Egardt, B. "Backlash Estimation With Application to Automotive Powertrains." IEEE TCST 15(3) 2007. DOI 10.1109/TCST.2007.894643. [S] — driveline backlash gap + EKF(PWA). HIGH(canonical).
16. Nordin, M.; Gutman, P.-O. "Controlling mechanical systems with backlash — a survey." Automatica 38(10) 2002. DOI 10.1016/S0005-1098(02)00047-X. [S] — backlash 제어 survey(taxonomy). HIGH.
17. Templin, P.; Egardt, B. "An LQR torque compensator for driveline oscillation damping." IEEE CCA 2009. DOI 10.1109/CCA.2009.5281020. [S] — shaft compliance/shuffle anti-jerk. HIGH.
18. Petit, N. et al. "Control-Oriented Modeling of a Vehicle Drivetrain for Shuffle and Clunk Mitigation." SAE 2019-01-0345. DOI 10.4271/2019-01-0345. [S] — torque source + dead-zone backlash + compliant half-shaft. HIGH(sim spec).
19. Lee & Jo, WEVJ 2024 15(10):433 (§2). [S] — FOPDT delay in MPC. HIGH(thesis).
20. Qian, W.; Panda, S.K.; Xu, J.X. "Torque ripple minimization in PMSM using ILC." IEEE TPEL 19(2) 2004. DOI 10.1109/TPEL.2003.823312. [S] — cogging+ripple angle-domain ILC. MED.
21. Lagerberg, A. "Control and Estimation of Automotive Powertrains with Backlash." PhD thesis, Chalmers, 2004. [S] — PWA model + gap estimation 종합. HIGH(textbook급).

### 일반 / 표현법
22. Bagheri, P.; Sedigh, A.K. "Analytical approach to tuning of MPC for first-order plus dead time models." IET Control Theory Appl. 7, 2013. DOI 10.1049/iet-cta.2012.0026. [S] — FOPDT MPC 튜닝(WEVJ ref [21]).
23. (flag) HEV launch backlash DOB FF-FB, IEEE 2022 doc 9709676 [F]; PWA-MPC backlash IFAC 2005 [F]; regen torque limiting CEP 2026 in-press [F].

먼저 읽을 것: #2(LuGre) · #1(steering friction physics) · #4(steering backlash ID) ·
#9(EHB nonlinear pressure) · #11(EMB clamp hysteresis) · #15+#16(driveline backlash) ·
#19(delay-aware MPC).

---

## 4. Lc1-Lc8 별 적용 표

핵심 원리: 물리 비선형성(friction/backlash/ripple/saturation)은 actuator 의 물리
인터페이스(torque/force/steer-angle)에 존재한다. 따라서 plant-side actuator layer 는
가장 낮은 물리 레벨(Lc1 등가)에 한 번 두고, 상위 레벨은 lowering 으로 그 인터페이스에
도달하며 자동 상속한다. 레벨에 따라 달라지는 것은 (a) 명령을 물리 인터페이스로
내리는 경로, (b) 그 결과 지연을 controller 가 보상할 수 있는지다 (optimization 레벨만 가능).

| Lc | 명령 quantity | 직접 노출되는 actuator 항목 | actuator layer 삽입 위치 | delay 보상 |
|---|---|---|---|---|
| Lc1 | per-wheel motor τ, brake τ, steer δ | motor lag/ripple/cogging, driveline backlash, brake clamp-hyst+dead-zone+mu(T), steer EPS friction+backlash+rate+lag — 전부 직접 | 각 wheel torque·steer 에 직접 (자연 삽입 레벨) | 없음 (open-loop) |
| Lc2 | axle drive τ, brake τ, steer δ | 동일, axle 단위 (diff 분배 후 per-wheel) | axle→per-wheel torque, steer | 없음 |
| Lc3 | Fx_total, steer δ | Fx→torque lowering 후 motor+brake+driveline; steer EPS | force→torque 변환 뒤 + steer | 없음 |
| Lc4 | throttle/brake pedal, steer δ | pedal→torque map(WEVJ table); pedal tip-in/brake-fill dead-zone, FOPDT lag, mu(T); steer EPS | pedal→torque map 직후 + steer | 없음 (CARLA 호환 직결) |
| Lc5 | ax target, steer δ | ax→force→torque cascade; powertrain FOPDT (WEVJ 레벨); steer EPS | cascade 하류 plant interface | 가능 (MPC state augmentation) — thesis 레벨 |
| Lc6 | v target, steer δ | v→ax→... 동일 | 동일 | 가능 (MPC) |
| Lc7 | curvature κ, v target | steer 내부 생성(PP/MPC)→steer EPS; 종방향 Lc6 와 동일 | 생성된 steer + 종방향 cascade 하류 | 가능 (양 채널, MPC) |
| Lc8 | waypoint path | path→κ,v 생성→Lc7; 양 채널 actuator | Lc7 와 동일, 최상위 | 가능 (양 채널, MPC) |

요지:
- plant-side actuator nonlinearity 는 단일 `IActuatorModel` 로 물리 인터페이스에 한 번
  구현 → Lc1-Lc8 이 lowering 으로 상속. 레벨마다 재구현 불필요.
- steering actuator 모델은 steer 를 내보내는 모든 레벨(Lc1-Lc6 직접, Lc7-Lc8 생성 steer)에 적용.
- controller-side delay 보상(state augmentation)은 optimization 레벨(Lc5-Lc8 이 MPC 일 때)만
  활용 가능. Lc1-Lc4 는 feedforward/직접 명령이라 지연을 그대로 겪는다.
- 따라서 actuator 카탈로그는 plant interface 에서 균일 적용, Lc 레벨이 바꾸는 것은
  lowering 경로와 delay 보상 가능성뿐이다.

sensor delay layer(feedback): 모든 레벨 공통으로 state 읽기 경로에 transport delay
(+옵션 lag). MPC 레벨은 augmented state 로 sensor delay 도 함께 보상 가능.
