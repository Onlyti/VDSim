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
- **타이어 슬롯 (고급):** `tire` = FL 기본(미지정 시 전체 fallback). `tire_rear` = RL+RR.
  `tire_fr` / `tire_rl` / `tire_rr` 로 코너별 교체 가능 (materialize → `tire_fr.yaml` 등).
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

### 2.3 계층 (확정) + 계획 필드 🔴

시나리오가 최상위. **map · comms · sim 은 시나리오 단위(공유)**, **센서는 차량(에이전트)
안**, **path(주행경로)는 에이전트 단위(map 위)**:

```yaml
sim: { dt: 0.005, t_end: 40, time_scale: 1.0 }   # 🔴 시나리오 단위
map: map.figure8                                 # 🔴 시나리오 단위 (공유 환경/노면)
comms: hil_setup                                 # 🔴 시나리오 단위 (configs/comms/<name>.yaml 참조; §3)
run: { mode: rt_comms }                           # 🔴 api | rt_comms
agents:                                           # (= 현재 fleet)
  - id: 0
    vehicle:                                      # 차량 = 부품(blueprint+parts) + 센서
      blueprint: vehicle.sedan_comfort            # 🟢 완성 레시피(모든 슬롯 채움)
      parts: { tire: tire.sport_grip }            # 🟢 그 위 슬롯 override(diff). 없으면 blueprint 원본
      sensors:                                    # 🟢 파싱/저장 (mount = 위치+자세 pose) — 아래 §2.3.1
        - { id: gnss, type: gnss, mount: { pos: [1.4,0,1.0], rpy: [0,0,0] }, rate: 10 }
        - { id: cam,  type: camera, mount: { pos: [1.6,0,1.2], rpy: [0,-0.05,0] }, rate: 30 }
    spawn: { x: -15, y: -1.5, yaw: 0, vx: 12 }    # 🟢 (현재 x0/y0/z0/yaw0/vx0)
    path: paths/figure8_centerline.yaml           # 🔴 trajectory 파일(waypoints/속도) → CTE/lap + path-follow
    control: external | internal                  # 🟢
```

control 의미(런타임 무관): action 을 sim core 의 `SimSession::set_input(CmdL4)` 로 주입하는 seam.
- `external` — 코어 밖에서 계산한 action 을 주입. 소스는 `run.mode` 가 결정:
  `rt_comms` → UDP(comms) → set_input · `api` → 알고리즘/driver → set_input (vdsim_lab).
- `internal` — 내장 제어기가 루프 안에서 직접 set_input (v1 = speed-hold; path-follow 는 `path` 와 함께 🔴).

blueprint vs parts: blueprint = base 레시피, parts = 그 위 슬롯 patch. materialize 가
`blueprint + parts → resolved vehicle.yaml/tire.yaml` 로 합친다.

> 현재 구현은 평평한 `fleet:` (agent=blueprint+pose+control)이고, 위 `path`(trajectory 파일)
> / `map` / scenario-level `sim`·`comms` 는 단계적으로 추가(🔴). `agents.vehicle.sensors`
> 는 파서까지 들어왔다(§2.3.1). 계층 자체는 위가 확정.

### 2.3.1 `sensors:` — 에이전트 센서 선언 🟢(YAML→WorldScenario 파서) / 🔴(렌더 커플링)

`fleet[].sensors`(= `agents.vehicle.sensors`)를 `cosim/world_scenario.cpp` 가 파싱해
`VehicleSpawn` 에 싣는다. 한 항목이 서로 다른 두 가지를 동시에 나른다:

- **노이즈** `noise_std`/`bias`/`bias_rw` → `VehicleSpawn::sensors` (`vdsim::SensorParams`,
  신호 그룹 단위) → 시뮬레이션에 **실제로 적용된다**.
- **장착** `id`/`type`/`mount{pos,rpy}`/`rate`/`params` → `VehicleSpawn::scene_sensors`
  (`SceneSensor`) → **파싱해서 저장하는 데까지만**. mount pose 를 읽는 소비자는 아직 없고
  (향후 렌더/센서 프레임 커플링 🔴), 지금은 시뮬 결과에 아무 영향을 주지 않는다.

세 가지 형태를 받는다:

```yaml
sensors:                                   # (1) 목록형 — §2.3 의 정식 형태
- { id: gnss_roof, type: gnss,   mount: { pos: [0.2,0,1.42], rpy: [0,0,0] },    rate: 10, noise_std: 0.3 }
- { id: cam_front, type: camera, mount: { pos: [1.6,0,1.2],  rpy: [0,-0.05,0] }, rate: 30, params: { fov_deg: 90 } }

sensors:                                   # (2) suite 형 — enabled/seed 까지 지정
  enabled: false
  seed: 42
  list:
  - { id: gnss, type: gnss, mount: { pos: [1.4,0,1.0] }, rate: 10 }

sensors: configs/sensors/noisy.yaml        # (3) SensorParams 파일 경로 (노이즈만, mount 없음)
```

- `type`: `gnss`·`gnss_pos`·`gnss_vel`·`imu`·`imu_accel`·`imu_gyro`·`wheel_speed`·`steer`
  ·`camera`·`lidar`. `camera`/`lidar` 는 코어에 계측 모델이 없어 **장착 선언 전용**이다
  (노이즈 키를 주면 에러).
- `mount.pos` [m] = 차체 좌표(x 전방 / y 좌 / z 상), `mount.rpy` [rad] = roll·pitch·yaw.
  `rate` [Hz] 는 선언값일 뿐 다운샘플링은 아직 없다(0/생략 = 미지정).
- `params:` 는 타입별 확장 knob 을 담는 숫자 map(`fov_deg` 등). 역시 저장만 한다.
- `id` 생략 시 `type` 이 id 가 되고, 한 차량 안에서 id 중복은 에러.
- **잘못된 입력은 전부 hard error**: 모르는 키/타입, 원소가 3개가 아닌 `pos`/`rpy`, 숫자가
  아닌 값, 값이 빈 `sensors:`, `list:` 없는 map — 조용히 무시하지 않고 차량 번호와 문제 키를
  지목해 throw 한다.
- 우선순위: 차량별 `sensors` 가 시나리오 레벨 `sensors:`(파일 경로)를 덮는다
  (`effective_sensor_params()`, `cosim/world_scenario.hpp`).
- 동작 샘플: `configs/scenes/two_vehicle_race.yaml` (0번 차량이 mount pose 4개를 선언).

---

## 3. 실시간 통신 설정 — comms/*.yaml 🟢(vds1·json·nmea_gga) / 🔴(sensor.*)

realtime 모드에서 **데이터가 어디로 흐르는지** 선언. comms 는 **시나리오 단위** — scene 의
`comms: <name>` 키가 `configs/comms/<name>.yaml` 를 가리킨다(에이전트별이 아니라 시나리오가 소유).

```yaml
name: vds1_loopback              # 동작 샘플: configs/comms/vds1_loopback.yaml
channels:
  # 송신 TX: source → 규약(template) → 목적지(들) (fan-out)
  - source: 0.state              # <id>.state | ego.state(=첫 차)
    template: vds1               # 🟢 VDS1 바이너리 (cosim/cosim_protocol.hpp)
    to: [ {ip: 127.0.0.1, port: 7100}, {ip: 127.0.0.1, port: 7101} ]
  - source: 0.state              # 🟢 JSON 한 줄 (HTTP/MQTT 브리지용)
    template: json
    to: [ {ip: 127.0.0.1, port: 7100} ]
  - source: 0.state              # 🟢 NMEA 0183 $GPGGA ("*HH\r\n" 종단)
    template: nmea_gga
    origin: { lat: 37.5665, lon: 126.9780, alt: 38.0 }   # ENU 미터의 측지 원점(선택, 기본 0/0/0)
    to: [ {ip: 10.0.0.5, port: 9001} ]
  - source: 0.sensor.gnss        # 센서 소스 지정 (🔴 미구현, 경고 후 skip)
    template: nmea_gga
    to: [ {ip: 10.0.0.5, port: 9002} ]
  # 수신 RX: 포트에서 listen → 규약 → (vehicle_id 헤더로) 어느 제어 (fan-in)
  - direction: in
    template: vds1_cmd           # 🟢 패킷 헤더 vehicle_id 로 대상 에이전트 선택
    listen: { port: 7001 }       # 누구나 이 포트로 cmd 전송 (last-writer/ZOH)
```

- **TX(송신)** = `source`(`<id>.state`) → `template`(규약) → `to`(ip:port, fan-out).
- **RX(수신)** = `direction: in` + `listen.port` → `template` → 제어 입력 (fan-in). 어느 에이전트인지는 VDS1 cmd 헤더의 `vehicle_id` 로 결정.

**구현 상태**: `vdsim_realtime` 가 comms.yaml 을 실행하는 라우터(🟢) — TX `<id>.state` 를 `vds1`(=`vds1_state`) / `json` / `nmea_gga` 로 fan-out, RX `vds1_cmd` listen-port fan-in 동작. comms 키가 없으면 레거시(단일 cmd-in/state-out CLI 플래그)로 fallback. 그 외 template 이름과 `sensor.*` source 는 아직 미구현(🔴) — 경고 찍고 skip.

`nmea_gga` 세부(구현: `cosim/comms_templates.hpp`, 샘플: `configs/comms/json_nmea.yaml`):
- **origin**: 채널별 선택 필드 `origin: {lat, lon, alt}`. 시뮬 ENU 미터(X east / Y north / Z up, `core/include/vdsim/coordinate.hpp`)의 측지 기준점. 미지정 시 (0, 0, 0).
- **변환**: WGS84 정밀 폐형해 — ENU → ECEF (origin ECEF + ENU 회전행렬) → geodetic (Bowring + Newton). 근사가 아니므로 **위도 의존성이 없다**. lat∈[-90,90], lon∈(-180,180] 로 항상 정규화된다.
  - **정확도 기준은 해석적(analytic) ECEF**: 결과 (lat, lon, h) 를 다시 ECEF 로 정변환해 60자리 정밀도로 계산한 기준점과 비교했을 때, 234,234 점 격자(datum lat −90~90 극점 포함, lon −180~180 날짜변경선 포함, datum alt 0/38/8848 m, east/north ±1 m~±1e6 m, up −1000~+1e4 m)에서 **최대 5.2e-9 m**.
  - **PROJ 는 기준이 아니라 교차검증**이다. `+proj=topocentric` / `+proj=cart` (pyproj 3.5 / PROJ 9.2) 와는 **결과 타원체고(HAE)가 약 2 km 이하이고 datum 위도가 극점을 벗어난(|lat0| < 89°) 범위에서 6e-8 m 이내**로 일치한다 — 수평 ENU 100 km(접평면 상승으로 HAE ≈ 800 m)까지 포함해 지상 차량 시나리오 전 구간이 여기 들어간다. 두 단서는 모두 필요하고, 벌어지는 쪽은 두 경우 모두 **PROJ** 다.
    - **datum 이 극점(|lat0| ≥ 89°)**: 일치 범위가 **2e-7 m** 로 넓어진다(실측 최댓값 1.5832e-07 m — lat0 −90, lon0 −180, alt0 0, e −1, n 1, up −1000). 같은 점에서 60자리 해석적 기준 대비 이 구현은 3.3e-10 m, PROJ 는 1.581e-7 m 다. 초과분은 datum 이 정확히 자전축 위(±90°)일 때만 나타난다 — 동일 격자에서 lat0 ±89.9° 는 2.8e-9 m.
    - **HAE 2 km 초과**: PROJ 의 `cart` 역변환이 벌어진다. HAE 11 km 에서 1.4e-6 m, HAE 165 km(lat0 60, e=n=−1000 km, up=10 km)에서 3.4e-4 m 이며, 같은 점에서 PROJ 자신이 해석적 기준과 동일한 크기로 어긋난다(이 구현은 여전히 ~2.6e-9 m).
    따라서 PROJ 수치는 성립 범위(고도·위도)를 반드시 함께 적어야 하고, 이 구현의 오차 상한이 아니다.
  - 직전 버전은 등거리원통(equirectangular) 근사(`lon = lon0 + east/(N·cos lat0)`)로, 서울 datum(lat 37.5665) 기준 10 km 에서 6.75 m / 100 km 에서 678 m 오차였다 — 문서에는 0.1 m / 100 m 로 적혀 있었으나 그것은 적도 근처에서만 성립하는 값이다.
  - 남은 오차는 구현이 아니라 ENU **정의** 자체다: 접평면이므로 수평 거리 d 에서 타원체 위로 ~d²/(2R) 만큼 뜨고(10 km 에서 7.8 m, GGA 고도에 반영됨), 진짜 측지선 대비 ~d³/(3R²) 차이가 남는다(10 km 에서 8 mm, 100 km 에서 8.2 m). 고도는 타원체고(HAE)이며 geoid 모델은 없다.
- **비정상 상태 처리**: 솔버가 발산해 NaN/Inf, 비상식적으로 큰 ENU 오프셋(|offset| > 1e8 m), 또는 **역변환이 유일하지 않은 지점**(지구 중심 근처 — 타원체 evolute 위/내부)이 들어오면, 실제 수신기처럼 **no-fix 문장**(quality 0, 00 satellites, 99.9 HDOP, 빈 위치/고도 필드)을 내보낸다. quality 1 문장에 검증되지 않은 필드가 실리는 경로는 없다.
  - 폐형해는 결과를 ECEF 로 되돌려 입력점과 대조한 뒤에만 성공(`Geodetic::ok`)으로 보고한다. 실패 시 lat/lon/alt 는 NaN 이며, 기본 생성된 `Geodetic` 역시 `ok=false` + NaN 이다.
  - **evolute 판정은 기하학적 판정**이다(`inside_evolute()`): 자전축 위뿐 아니라 축을 벗어난 점도 포함해, 자오면 좌표 (p, z) 에 대해 astroid 부등식 `(p/42697.7)^(2/3) + (|z|/42841.3)^(2/3) ≤ 1` 을 직접 평가한다. 왕복(round-trip) 검사로는 이 영역을 걸러낼 수 없다 — 내부의 점 하나에 대해 **네 개의 위도 해가 모두 입력 ECEF 를 정확히 재현**하기 때문이다. 실제로 예전에는 축 위(p ≈ 0)에서만 판정해서, 예컨대 datum (0,0,0) + `m_gnss_x=-1595.7, m_gnss_y=23172.3, z=-6380182.2`(ECEF 원점에서 23.3 km)가 **지구 중심의 quality 1 · 위성 12 · HDOP 0.9 문장**으로 나갔다. 이 영역은 타원체면에서 약 6335 km 아래(지오센터 반경 42.9 km 이내)라 정상 차량 상태와는 무관하다.
  - NMEA 각도 필드 범위 검사는 필드별 상한을 쓴다 — 위도(2자리) 90°, 경도(3자리) 180°. 예전에는 둘 다 180° 로 검사해서, 예컨대 datum 서울 + `z = -6.35e6` m 같은 발산 상태가 위도 −163.55° 를 `16333.1094`(GGA 규격 9자 자리에 10자)로 실어 **체크섬이 맞는 quality 1 문장**으로 나갔다.
  - 과거에는 `static_cast<int>(NaN)` UB 로 필드 안에 공백이 들어간 문장이 체크섬까지 맞은 채 전송됐다.
- **위치 소스**: 측정 GNSS(`m_gnss_x/_y`). GGA 는 수신기 출력이므로 센서 노이즈가 그대로 반영된다. 노이즈 미설정 시에도 `SensorModel` 이 항등 경로로 truth 를 채우므로 값이 비지 않는다. 고도만 truth `z`(GNSS 고도 채널 모델 없음).
- **UTC 필드**: 시스템 wall clock (STATE 의 `timestamp` 는 steady_clock 기반 monotonic 이라 UTC 가 아님). 초는 출력 정밀도(1/100 s)로 **먼저 반올림한 뒤** 초→분→시→날 자리올림을 하므로 `125960.00` 같은 불법 시각은 나오지 않는다.
- **satellites / HDOP / geoid separation**: 위성·DOP 모델이 없어 고정 placeholder(12 / 0.9 / 0.0).

`json` 세부: 모든 숫자 필드는 유한값이 아니면 RFC 8259 의 `null` 로 출력된다(`nan`/`inf` 는 JSON 이 아니라 `json.loads`·`JSON.parse` 모두 거부한다). 발산 시 프레임을 버리지 않고 `null` 로 신호하는 이유는, 조용히 사라진 데이터그램은 단순 UDP 손실과 구분되지 않기 때문이다.

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
| comms.yaml 라우터 — vds1 TX fan-out / RX fan-in | 🟢 | §3, `vdsim_realtime` (scene `comms:` 참조) |
| comms json·nmea_gga TX template (+ 채널별 `origin`) | 🟢 | §3, `configs/comms/json_nmea.yaml` |
| comms sensor.* source | 🔴 | §3, 경고 후 skip |
| scene `path`(trajectory 파일)/`maneuver`/`run{mode}` | 🔴 | §2.3, SIM_CONFIG_ARCH |
| agent `vehicle.sensors` 파서(mount pose→`WorldScenario`, 검증 포함) | 🟢 | §2.3.1, `cosim/world_scenario.cpp` |
| mount pose 를 쓰는 렌더/센서프레임 커플링 / map-path CTE | 🔴 | §2.3.1, SIM_CONFIG_ARCH §7 |
