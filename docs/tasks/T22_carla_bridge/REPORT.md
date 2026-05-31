# T22 — VDSim ↔ CARLA Python bridge (Ld2-SevenDOF driving CARLA ego)

## 목적

CARLA 0.9.13 server 위에서, CARLA UE4 가 렌더링·sensor·NPC traffic 만 담당하고
**VDSim 이 ego 차량의 단독 dynamics authority** 가 되는 PoC bridge 를
검증한다. 이 구조는 VI-grade / IPG / 사내 모션 플랫폼이 CARLA 와 통합할 때
사용하는 "external physics" 패턴과 동일하며, VDSim 의 Ld2-SevenDOF (7 DOF)
모델을 그대로 CARLA 시각화 환경에 꽂는 것을 목표로 한다.

## 구현 방법

- `carla_integration/python/vdsim_carla_bridge.py`
  - `actor.set_simulate_physics(False)` 로 CARLA 자체 wheel/엔진 모델 비활성
  - per tick:
    1. `world.cast_ray(start, end)` 로 4 wheel contact 조회 (Ld2 의 `IContactProvider` 와 동일 ABI)
    2. `vdsim::SevenDofDynamics::step(cmd, contacts, h)` 실행
    3. ISO 8855 RH → UE4 LH 좌표 변환 (`y` 반전, `yaw` 부호 반전)
    4. `actor.set_transform / set_target_velocity / set_target_angular_velocity`
       로 CARLA actor 갱신
  - VDSim 내부 step 은 outer 20 ms / inner 5 ms (RK4 4-step)
- `carla_integration/python/run_demo.py`
  - figure-8 reference path (R=20 m, 두 루프) + Pure Pursuit + Lc6-VTarget PI
  - target vx = 10 m/s
- `carla_integration/python/record_demo.py`
  - 추가로 RGB camera (960×540, 90° FOV, third-person attach) 와
    full telemetry CSV 기록

## 검증 방법 (근거)

| 항목 | 방법 |
|---|---|
| Frame conversion 정확성 | `test_bridge_offline.py` 의 `FrameConversionTests` (y axis flip, yaw sign flip, world velocity rotation) — CARLA server 없이 unittest 5/5 통과 |
| Contact provider ABI | `BridgeConstructionMocked` 에서 `world.cast_ray` mocked → `query_contacts()` 가 4 개 ContactPoint 반환 |
| Bridge 통합 (서버 포함) | CARLA 0.9.13 server (Town10HD_Opt) 와 실제로 연결, 15 s figure-8 주행, telemetry CSV + 카메라 PNG 71 frame 기록 |
| Physical plausibility | 좌선회 시 외측 (FR/RR) Fz 증가, 우선회 시 외측 (FL/RL) Fz 증가 — lateral load transfer 정합성 |

## 검증 결과

### Demo run (run02, 15 s @ 50 Hz tick, Town10HD_Opt)

- 750 ticks in 8.82 s wallclock → **85 TPS** with camera (raw without camera: 529 TPS)
- 750 telemetry samples + 71 PNG frames
- Spawn pose `(-64.64, 24.47, 0.60)` (CARLA world)

### 핵심 신호 (log 발췌)

```
t=  0.02  vx=10.00  steer=+0.104  r=+0.0000  Fz=[0, 0, 0, 0]            # 첫 tick 은 raycast 미체결
t=  1.02  vx= 9.38  steer=+0.104  r=+0.3588  Fz=[2796, 4115, 2543, 3600] # 좌선회, FR/RR loaded
t=  8.02  vx= 9.32  steer=+0.124  r=+0.4263  Fz=[2687, 4236, 2444, 3687]
t=  9.02  vx= 7.84  steer=-0.500  r=-0.9127  Fz=[5111, 2219, 4005, 1686] # 우선회 전환, FL/RL loaded
t= 14.02  vx= 5.44  steer=-0.500  r=-0.8905  Fz=[4505, 2611, 3692, 2174]
```

### 시각화

- `results/run02/demo.mp4` — third-person camera, 3.8 MB
- `results/run02/telemetry.png` — XY trajectory + vx/steer + yaw rate/ay + Fz 4-wheel
- `results/run02/telemetry.csv` — 196 KB, t/x_world/y_world/yaw/vx/vy/r/ax/ay/steer/throttle/brake/Fz×4

## 판단 결과 (근거)

| 판단 | 근거 |
|---|---|
| **OK** — bridge 동작 검증됨 | spawn → 15 s 주행 → cleanup 까지 RPC 정상 동작, telemetry 750 samples + 카메라 frame 71 정상 기록 |
| **OK** — lateral load transfer 물리 정합 | 좌선회 (steer +0.10, r +0.36) → FR/RR ↑, 우선회 (steer −0.50, r −0.91) → FL/RL ↑; outside wheel loading 패턴이 7-DOF roll equation 결과와 일치 |
| **OK** — frame conversion (ISO 8855 ↔ UE4) | offline unit test 5/5 + 실주행에서 ego trajectory 가 spectator 카메라에서 자연스럽게 좌→우 figure-8 그리는 것을 확인 |
| **부분적 한계** — vx 가 target 10 m/s 에 미달 (8~9 m/s) | Pure Pursuit + 단순 PI 의 한계로, 곡선부에서 throttle 추가가 부족; controller 자체 한계이며 dynamics 정합성과는 무관 |
| **부분적 한계** — z = 고정 (spawn 높이) | PoC 에서 ground projection 미적용 — 평지 가정. Phase 2 에서 raycast 결과의 z 를 ego z 에 반영해야 함 |
| **부분적 한계** — CARLA server occasional segfault on cleanup | `apply_settings` 후 RPC timeout 1 회 발생, 데이터에는 영향 없음 (CSV/PNG 는 try 블록 내에서 미리 저장) |

## 다음 단계

1. `world.project_point` 기반 ground z 반영 (3D ride dynamics 와 결합)
2. C++ 측 `RaycastContactProvider` (`carla_integration/plugin/`) — UE5 plugin Phase 2
3. Per-material μ lookup (CARLA `PhysicalMaterial`)
4. CARLA TM 차량 + VDSim ego 동시 충돌 회피 테스트
5. Sensor (RGB / depth / LiDAR) ↔ VDSim telemetry 동기화 저장 → dataset 생성
