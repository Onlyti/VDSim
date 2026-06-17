# VDSim Plant — 인수 노트 (thesis-side, BETA #1)

> 이 문서를 thesis repo(예: `~/vla_design/`) 세션에 그대로 붙여넣어 시작하세요.
> 전체 튜토리얼: `VDSim/docs/VDSIM_PLANT_TUTORIAL.md`, 빠른 참조: `VDSim/python/VDSIM_PLANT_README.md`.

## 0. 무엇을 받았나

기존 Python dynamic-bicycle plant를 **VDSim Ld2 7DOF + Pacejka MF2002 .tir** plant로 교체합니다.
MPC(acados)는 **그대로**, plant 객체만 교체. `step([delta, Fx])` lockstep API.

- 입력 `u = [delta_roadwheel_rad, Fx_total_N]` (steer + 종방향 force intent, +구동/−제동).
- 출력 obs = ground-truth 차량상태 + per-wheel tyre force (contact frame), 그 step에서 쓴 실제 μ.
- ISO 8855, wheel `FL=0 FR=1 RL=2 RR=3`. RNG 없음 = 완전 deterministic.
- tyre saturation이 진짜다: 과도한 command는 clamp가 아니라 **grip loss(슬립)** 로 나타남.
- spatial μ patch (저마찰 구간) 지원 — "unseen low-μ" 시나리오.

## 1. 셋업 (1회)

```bash
cd /home/ailab-12/git/VDSim       # branch: VDSim-Thesis
git checkout VDSim-Thesis
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j            # libvdsim_core + python module (build/python/)
```

```python
import sys
sys.path += ["/home/ailab-12/git/VDSim/build/python",
             "/home/ailab-12/git/VDSim/python"]
from vdsim_plant import VDSimPlant
```

## 2. 교체 레시피 (closed_loop_sim.py)

기존 bicycle plant의 `plant_deriv`/`rk4`를 아래로 교체. MPC는 손대지 않음.

```python
plant = VDSimPlant(
    config="ioniq5_awd",                 # configs/vehicles/ioniq5_awd.yaml
    base_mu=0.9,
    friction_map=[(40.0, 60.0, 0.5)],    # 선택: x=40~60m μ=0.5 저마찰 패치
    control_dt=0.1,                      # = MPC sample (ZOH). substep_dt로 나누어떨어져야 함
    substep_dt=1e-3,                     # 내부 적분 step (wheel-spin stiff → ≤1ms)
)
obs = plant.reset(state0=[X0, Y0, psi0, vx0, vy0, r0])

for k in range(N):
    x = [obs["X"], obs["Y"], obs["psi"], obs["vx"], obs["vy"], obs["r"]]
    delta, Fx = mpc.solve(x)             # 당신의 acados MPC — UNCHANGED
    obs = plant.step([delta, Fx])
```

obs → MPC state 매핑 (동일 convention이라 그대로 사용):

| obs | 의미 | MPC state |
|---|---|---|
| `X, Y, psi` | world pose | x, y, ψ |
| `vx, vy, r` | body vel + yaw rate | v_x, v_y, r |
| `ax, ay, beta` | 가속도(specific force), sideslip | (추정/로깅용) |
| `wheel[i].{Fx,Fy,Fz,alpha,kappa,mu}` | per-wheel GT (contact frame) | friction-circle 분석용 |

friction-circle 판정은 분석자 몫: `‖[Fx,Fy]‖ / (mu·Fz)` per wheel. plant는 usage metric을 내보내지 않음(thesis-side 전용).

## 3. 이번 베타에서 바뀐 것 — tyre를 MF2002로 (당신 서사와 직결)

이전 MF96은 Pacejka 계수가 하중 무관(상수 B/C/E, Ca∝Fz 선형)이었음. → **MF2002 .tir**로 교체:

| 효과 | 수치 (검증됨) |
|---|---|
| cornering stiffness 하중 포화 (concave) | `Kya(2Fz0)/Kya(Fz0)=1.435 (<2)` |
| peak μ 하중 민감도 | `0.90 → 0.81 @ 2·Fz0` (PDY2=−0.10) |
| 단일 .tir로 front/rear Ca 자동 분리 | `Kya(7011)=109.3k→Caf≈218k`, `Kya(4558)=79.5k→Car≈159k` (목표 220/160, ~1%) |
| friction patch | contact μ=0.5 → peak 0.50 (정상 스케일) |

의미: 하중이전(load transfer) → 바깥 휠 Fz↑ → Ca 포화 + μ 저하 → 조기 한계/understeer 가
plant에 들어옴. 즉 **Fz 변동이 grip에 비선형으로 작용**. σ_Fz를 covariance로 넘겨
chance-constraint로 쓰는 구조의 plant-side 물리근거가 생김. MPC 내부모델(linear-Ca bicycle)과
plant(load-dependent)의 mismatch는 의도된 robustness stress — matched plant보다 강한 서사.

## 4. 검증 상태

- `cd build && ctest -R VlaPlant --output-on-failure` → 10/10. 전체 394/394 green.
- dry handling = linear bicycle 내 ~4% (analytic-B tyre + 실제 MF2002 .tir 양쪽 확인).
- friction circle `‖[Fx,Fy]‖ ≤ μ·Fz` 매 step, 제동이 휠을 역회전시키지 않음.
- 5s 궤적 ≈ 25ms wall-clock (≫ real-time → sweep 저렴).

## 5. 베타 피드백 요청 (이게 핵심)

당신은 첫 베타테스터입니다. 못박힌 게 아니라 **써보고 장단점 회신 → 다음 iterate** 단계.
다음을 중심으로 회신 바람:

1. API 마찰 — `step/reset` 시그니처, obs 키/단위/frame이 MPC 루프에 그대로 맞물리는가?
2. tyre 거동 — dry yaw-rate gain이 기존 bicycle 결과와 충분히 일치하나? 한계영역(패치 진입)
   거동이 thesis 시나리오에 쓸만한가? μ/Fz 수치가 기대범위인가?
3. 누락 채널 — MPC/estimator가 필요한데 obs에 없는 양(예: 개별 휠 Fz GT 외 추가 신호)?
4. 성능 — 당신 sweep 규모에서 속도/결정론이 충분한가?
5. 파라미터 — ioniq5_awd 제원이 thesis 차량 가정과 어긋나는 부분?

회신 형식 자유(슬랙/이슈/메모). 회신 받으면 VDSim-Thesis 브랜치에서 개선 → 안정화 후 main 병합.

## 6. 제약 (참고)

- 현재 VDSim-Thesis 브랜치. main은 보호 중(아직 미병합).
- tyre 계수는 전부 public-synthetic Ioniq5급 근사 — 실측/대외비 데이터 아님.
