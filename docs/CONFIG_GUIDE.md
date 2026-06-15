# VDSim 설정 가이드 — batch(CLI) / scenario / comms

한 번의 VDSim 실행을 **코드 없이 설정 파일로** 구성하는 법. 설계 근거는
`design/SIM_CONFIG_ARCH.md`(런모드·comms·sensor host), `design/BATCH_RUNNER.md`(배치),
`design/PARTS_CATALOG.md`(부품·blueprint·scene). 이 문서는 **실제로 동작하는 현재 스키마**와
**계획(제안) 스키마**를 구분해 적는다. 표기: 🟢 동작 · 🟡 부분 · 🔴 계획.

---

## 0. 한 장 요약

```
parts/*.yaml ─┐
              ├─▶ blueprints/*.yaml ─┐
tire/*.yaml ──┘   (차량 = 부품 조합)  ├─▶ scenes/*.yaml ─┬─▶ vdsim_realtime  (실시간/외부제어, rt_comms)
                                      │   (에이전트 fleet) ├─▶ vdsim_lab/batch (헤드리스 동기, api)
maps · sensors · comms ──────────────┘                   └─▶ gui/server.py   (저작 + Play)
campaign.yaml ─▶ tools/vdsim_batch.py ─▶ 위 scene/experiment 들을 sweep/MC 로 다수 실행
```

두 실행 모드:
- **batch / api** (🟢) — 헤드리스 동기 실행. 네트워크 없음. 제어 = 코드(또는 시나리오 autopilot). 배치 평가·sweep·MC.
- **realtime / rt_comms** (🟢 단일포트 / 🔴 comms.yaml 라우터) — plant 가 free-run, 제어가 UDP 로 들어옴(외부 노드/HIL). 송수신은 comms 로 라우팅(라우터는 계획).

---

## 1. 배치(CLI) 설정 — campaign.yaml 🟢

실행: `python3 tools/vdsim_batch.py run campaign.yaml [--dry]` (RUNNING.md §C)

campaign 은 **다수 실행을 펼치는 드라이버**다. 각 run 은 explicit / sweep(grid 카르테시안) / monte_carlo 중 하나:

```yaml
name: fdr_vs_surface
runs:
  - scenario: skidpad                       # 등록된 experiment/scene 를 그대로 1회
  - sweep:                                   # base + grid → 카르테시안 곱
      base: skidpad
      grid:
        vehicle.final_drive_ratio: [4.0, 5.0, 6.0]
        mu: [0.8, 1.0]
        maneuver.v: [25, 30]
  - monte_carlo:                             # base + 확률 샘플 N개
      base: skidpad
      n: 200
      vary:
        vehicle.mass: { dist: normal, mean: 1500, std: 50 }
        mu:           { dist: uniform, lo: 0.7, hi: 1.0 }
metrics: [lap_time, peak_ay, understeer_K, max_Fz]
output: results/fdr_vs_surface/             # per-run CSV + summary.csv + resolved/
parallel: 8
duration: 40
```

- **override 는 dotted path**: `vehicle.* / tire.* / road.* / maneuver.* / mu / level`. `vehicle.X` 는 preset 로드 후 필드 X 만 in-memory override.
- 각 run 은 독립 → `multiprocessing.Pool(parallel)` 병렬. 실패한 run 은 행만 failed 로 표시, 배치 중단 안 함.
- 출력: `results/<name>/<run_id>.csv` + `summary.csv`(run 당 한 행 = params + metrics) + `resolved/<run_id>.yaml`(재현용).
- metrics 는 name→fn 레지스트리(`apps/doe/metrics.py` 확장).

여러 **차량 preset 비교**만 필요하면 배치 대신 `tools/vdsim_compare.py`(🟢) 가 더 직접적: 같은 ISO maneuver 를 preset N대에 돌려 지표 표 + yaw-rate overlay.

---

## 1.5 시뮬레이션 세팅 — 시간 / dt / 배속 / 적분기

"몇 초에서 몇 초까지, dt 는 얼마, realtime 배속은?" 을 정하는 설정. **현재는 여러 곳에
흩어져 있다**:

| 항목 | 현재 위치 | 기본 |
|---|---|---|
| step rate (→ dt = 1/rate) | scene top-level `rate:` [Hz] | 200 → dt 5 ms |
| cmd timeout (외부 cmd 끊김 failsafe) | scene `cmd_timeout:` [s] | 0.1 |
| 적분기 / 서브스텝 | SolverParams (`configs/solvers/*.yaml`): `integrator`(rk4\|euler), `max_substep_dt`(1e-3), `max_substeps`(10) | RK4 |
| realtime 배속(run speed) | `vdsim_realtime` 가 scene 폴더의 `time_scale` 사이드카 파일을 읽음(GUI 슬라이더가 씀). 1.0=실시간 | 1.0 |
| 실행 길이(duration, t_end) | **batch 전용**: campaign `duration:` [s] | — |
| 시작/끝 구간 t_start..t_end | 통합 필드 없음. maneuver 가 자체 `t0` 보유 | — |

### 제안: scene 에 통합 `sim:` 블록 🔴

위를 한 곳에 모으면 시나리오만 보고 시간/배속을 안다:

```yaml
sim:
  dt: 0.005          # 고정 스텝 [s]  (또는 rate: 200)
  t_end: 40          # 실행 끝 시간 [s]  (t_start 기본 0)
  time_scale: 1.0    # realtime 배속: 1=실시간, >1 빠르게, 0=as-fast-as-possible(헤드리스)
  integrator: rk4    # rk4 | euler
  max_substeps: 10
  cmd_timeout: 0.1   # realtime 외부제어 failsafe
```

- **batch/api**: `dt`+`t_end` 가 실행 구간. `time_scale` 무시(가능한 빨리). 현재는 campaign `duration` 가 t_end 역할.
- **realtime**: `dt`(=1/rate) + `time_scale` 로 페이싱. `t_end` 있으면 자동 종료, 없으면 무한 free-run.
- 적분기/서브스텝은 SolverParams 와 1:1 매핑(통합 시 scene 이 solver preset 참조 또는 inline).

---

## 2. 시나리오(scene) 설정 — scenes/*.yaml

차량(부품 조합) + 위치 + 환경을 한 파일에 조립. 실행: `vdsim_realtime --scene=configs/scenes/<name>.yaml`.

### 2.1 현재 스키마 🟢

```yaml
id: scene.two_vehicle_race
version: 1
label: Two-car straight launch
rate: 200                 # Hz
cmd_timeout: 0.1          # s, 외부 cmd 끊기면 failsafe
mu: 1.0                   # 환경: 노면 마찰
# grade / bank / stunt{...} / terrain  도 top-level 환경 키 (선택)
fleet:
  - id: 0
    blueprint: vehicle.sedan_comfort   # = 부품 조합(body/aero/ride/chassis/tire/brake/steering/drivetrain)
    level: L2                          # L1|L2|L3|L4|L5
    x0: -15
    y0: -1.5
    yaw0: 0
    vx0: 12
  - id: 1
    blueprint: vehicle.sports_aggressive
    level: L2
    parts: { tire: tire.sport_grip }   # 인스턴스 부품 swap (선택)
    x0: -15
    y0: 1.5
    vx0: 12
```

- `blueprint` 가 차량=부품 조합(자세한 부품 taxonomy 는 PARTS_CATALOG.md).
- `parts:` 로 그 에이전트만 슬롯 교체.
- GUI 가 이 scene 을 materialize(blueprint→resolved vehicle.yaml/tire.yaml 경로)한 뒤 `vdsim_realtime` 에 넘긴다.

### 2.2 제어방법 — per-agent `control` 🟡 (이 가이드와 함께 추가)

각 에이전트가 **내부 제어기**로 굴러갈지 **외부 노드**가 제어할지 선언:

```yaml
fleet:
  - id: 0
    blueprint: vehicle.sedan_comfort
    control: external        # 외부 노드/HIL 가 comms(UDP)로 제어 (기본). cmd 끊기면 failsafe.
  - id: 1
    blueprint: vehicle.race_gt
    control: internal        # 빌트인 제어기가 굴림 (외부 노드 불필요)
```

- `external` (기본): 현재 realtime 동작 — UDP cmd + 타임아웃 failsafe.
- `internal`: 빌트인 제어기. v1 = 스폰 속도 유지(speed-hold) 크루즈. **경로추종(pure-pursuit) 내부제어는 scene `path` 필드(🔴)와 함께 후속.**
- 용례: SUT 한 대는 external(외부 컨트롤러 평가), 나머지 reference/traffic 차량은 internal 로 알아서 굴림.

### 2.3 계획 스키마 🔴 (SIM_CONFIG_ARCH 제안, 아직 scene 에 없음)

```yaml
run: { mode: rt_comms, comms: hil_setup }   # api | rt_comms + comms 참조
fleet:
  - id: 0
    path: map.figure8.centerline            # 경로(Trajectory) 참조 → CTE/lap 평가 + 내부 경로추종
    sensors: sensors.sedan_default          # 센서 suite (host=ego/infra)
maneuver: { ref: maneuvers.step_steer, params: {...} }
```

---

## 3. 실시간 통신 설정 — comms/*.yaml 🟡

realtime 모드에서 **데이터가 어디로 흐르는지** 선언. scene `run:{mode: rt_comms, comms: <name>}` 로 참조(🔴).

```yaml
name: hil_setup
channels:
  # 송신 TX: source → 규약(template) → 목적지(들) (fan-out)
  - source: ego.state            # 차량 동역학 상태
    template: vds1_state         # source 타입이 packet 규약을 고정
    transport: udp
    to: [ {ip: 10.0.0.5, port: 7002}, {ip: 127.0.0.1, port: 7100} ]
  - source: ego.sensor.gnss      # 한 센서 → 두 소비자
    template: nmea_gga
    to: [ {ip: 10.0.0.5, port: 9001} ]
  # 수신 RX: 포트에서 listen → 규약 → 어느 제어로 (fan-in)
  - direction: in
    template: vds1_cmd
    listen: { port: 7001 }       # 누구나 이 포트로 cmd 전송 (last-writer/ZOH)
```

- **TX(송신)** = `source`(차량 상태/센서) → `template`(규약) → `to`(ip:port, fan-out).
- **RX(수신)** = `direction: in` + `listen.port` → `template` → 제어 입력 (fan-in, 여러 컨트롤러 가능).
- template 은 source 타입이 고정: `vds1_state`/`vds1_cmd`(`cosim/cosim_protocol.hpp` VDS1 바이너리) + 센서별(NMEA 등). 바이트 레이아웃 직접 안 짬.

**구현 상태**: comms.yaml 은 스펙·예시(🟡)만 있고 **이를 실행하는 라우터는 미구현(🔴)**. 현재 `vdsim_realtime` 는 CLI 플래그로 cmd-in 1포트 / state-out 1목적지만. 다중 fan-out/in·template 라우팅은 후속(RUNTIME_ARCH.md).

---

## 4. 현재 vs 계획 요약

| 항목 | 상태 | 비고 |
|---|---|---|
| parts/blueprint/scene(fleet+pose+환경) | 🟢 | PARTS_CATALOG.md |
| batch campaign 러너(sweep/MC/metrics, 병렬) | 🟢 | `tools/vdsim_batch.py` |
| 멀티차량 비교 도구(지표 표+overlay) | 🟢 | `tools/vdsim_compare.py` |
| realtime 단일 cmd-in/state-out (VDS1) | 🟢 | `vdsim_realtime`, external 제어 |
| 시뮬 세팅(rate/dt·time_scale·integrator·duration) | 🟡 | §1.5, 현재 분산 / 통합 `sim:` 블록은 🔴 |
| per-agent `control{internal|external}` | 🟡 | §2.2 (이번 추가; internal=speed-hold v1) |
| scene `path`/`maneuver`/`run{mode}` | 🔴 | §2.3, SIM_CONFIG_ARCH |
| comms.yaml 라우터(fan-out/in, template) | 🔴 | §3, RUNTIME_ARCH |
| sensor host / map-path CTE / vdsim.Simulation facade | 🔴/🟡 | SIM_CONFIG_ARCH §7 |
