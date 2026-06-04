# 18. Runtime Kernel & Co-simulation

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. SimSession kernel 의 한 tick (latched cmd → actuator → step → sensor) 과 그것이
   왜 mode-agnostic 한지 설명한다.
2. free-run (real-time), external-step (third-party stepping), scenario replay
   세 가지 run mode 가 같은 kernel 위의 얇은 wiring 임을 안다.
3. RealTimeRunner 의 wall-clock pacing 과 command-timeout fail-safe(ECU
   watchdog) 의 역할을 기술한다.
4. UDP co-sim wire protocol (24B header + CMD/STATE + CRC32) 과 GUI data port
   (JSON HTTP/UDP) 의 역할 분담 — 언제 바이너리, 언제 JSON 인지 판단한다.

## Prerequisites

- **Chapter 17** — actuator/sensor layer (kernel 이 wrap 하는 대상).
- **Chapter 12** — software architecture (factory, ABI).
- **Reference** — `docs/vdsim_bridge_interface_requirements.md` (wire contract),
  `docs/gui_architecture.md`.

---

## 18.1 동기 — 하나의 kernel, 여러 run mode

실차 경계(real-vehicle-equivalent)는 "고정 dt 로 도는 plant + 비동기 명령 stream +
지연된 feedback" 이다. 이걸 한 군데(SimSession)에 모으면, 실행 방식(실시간/외부
스텝/리플레이)은 그 위의 얇은 wiring 으로 갈린다. clock 과 I/O 를 kernel 에서
빼냄으로써 동일 plant 코드가 모든 mode 에서 재사용된다.

---

## 18.2 SimSession — mode-agnostic kernel

한 tick 의 순서 (clock 없음, I/O 없음):

```
set_input(u)   # 비동기 명령을 ZOH 로 latch (mutex)
tick(dt):
    contacts = ground.query(state)
    realized = actuator.apply(latched_cmd, speed, dt)   # ch17
    dyn.step(realized, contacts, dt)                     # ch04-06
    measured = sensor.apply(dyn.state(), dt)             # ch17 sensor delay
    # diagnostics(ax, ay, roll, pitch, Fz, tire_forces, rack) 스냅샷
```

- **ZOH latch**: `set_input` 은 명령을 보관만 한다. 비동기 producer(UDP/HTTP/스크립트)
  와 고정-dt tick 이 **decoupled** — ECU/actuator 가 마지막 명령을 hold 하는 것과
  동일. `set_input`/`state`/`output` 은 mutex 로 보호되어 producer 스레드와 sim
  스레드가 동시 동작.
- **출력**: `state()`(true plant), `measured_state()`(controller feedback, 지연됨),
  `output()`(전체 스냅샷). pull 방식.

---

## 18.3 Run modes — kernel 위의 얇은 wiring

| mode | 누가 tick 을 모나 | 용도 |
|---|---|---|
| **experiment / external-step** | 외부가 `set_input()+tick()` 을 자기 루프에서 호출 | 결정론적 co-sim, HIL, batch DoE |
| **free-run (real-time)** | `RealTimeRunner` 가 wall-clock 으로 tick 페이싱 | 실시간 시각화·UDP co-sim |
| **scenario replay** | 루프가 파일에서 `set_input` 피드 | 시나리오 재현 |

세 mode 가 같은 SimSession 을 공유하므로 plant·actuator·sensor 동작이 일관된다.

---

## 18.4 RealTimeRunner — wall-clock pacing + watchdog

free-run mode 의 페이서. `sleep_until` 로 고정 dt 를 벽시계에 맞춰 tick 하고,
명령이 `cmd_timeout_s` 동안 안 오면 **fail-safe** 명령으로 latch (ECU watchdog):

```cpp
struct Config { double dt{0.005}; double cmd_timeout_s{0.1};
                CmdL4 failsafe{0.0, 0.3, 0.0, 1, false}; };  // throttle0, 적당 brake
// 루프: cmd 없은 지 timeout 초과 -> set_input(failsafe); tick(dt); sleep_until(next)
```

- fail-safe = "명령 끊기면 적당히 제동하고 직진" — 통신 두절 시 안전.
- pacing 은 `next += dt` 누적 + `sleep_until` 로 drift 없이.

---

## 18.5 UDP co-sim wire protocol (binary)

결정론적·고빈도 외부 제어를 위한 바이너리 contract
(`cosim/cosim_protocol.hpp`, 상세 `docs/vdsim_bridge_interface_requirements.md`).

- **공통 24B header**: magic `0x56445331`("VDS1"), version(u16), msg_type(u16),
  seq(u32), pad(u32), timestamp(f64). little-endian, tightly-packed.
- **CMD 76B** (controller → sim): steer_tire, throttle, brake, gear, handbrake,
  aux_accel, aux_speed + trailing CRC32.
- **STATE 220B** (sim → controller): x,y,z / roll,pitch,yaw / vx,vy,vz /
  roll·pitch·yaw rate / ax,ay / wheel_spin[4] / steer_applied / wheel_radius /
  Fz[4] + CRC32.
- **CRC32** = IEEE 802.3, Python `zlib.crc32` 호환 (헤더+payload 전체).
- seq 로 out-of-order/stale 폐기. `vdsim_realtime <veh.yaml> <tire.yaml>
  [--level --cmd-port --state-ip --state-port --rate --vx0 --cmd-timeout]` 가
  SimSession+RealTimeRunner 를 띄워 이 프로토콜을 말한다.

---

## 18.6 GUI data port (JSON over HTTP/UDP)

접근성·툴링용 경량 경계. 실시간 결정론은 §18.5 바이너리가 담당하고, JSON 은
설정·대시보드·agent 제어용.

- **inbound** `POST /api/io {vehicle,throttle,brake,steer}` → 명령 라우팅(차량 id
  별) + 현재 state JSON 반환 (command-in / state-out, free-run).
- **outbound telemetry** `/api/io/targets` — 차량별로 설정한 ip:port 들에 state(+
  cmd 확인용)를 JSON UDP fan-out.
- **co-sim 구동 front-end**: GUI 가 현재 vehicle/tire 설정을 temp yaml 로 써서
  `vdsim_realtime` 를 subprocess 로 띄우고, 그 바이너리 STATE 를 받아 3D 로
  시각화하며 제어를 바이너리 CMD 로 relay (single source of truth).

### 언제 무엇을

| | 바이너리 UDP (§18.5) | JSON HTTP/UDP (§18.6) |
|---|---|---|
| 지연/결정론 | 최저, seq+CRC | 중, 편의 |
| 적합 | ≥100Hz 폐루프·HIL·다차량 | 설정·대시보드·agent·≤60Hz |
| 장점 | compact, jitter↓ | self-describing, 다언어, 디버깅 |

---

## 18.7 한계 / 다음

- 현재 `vdsim_realtime` 는 vehicle/tire yaml 만 받아 **actuator/solver 설정은
  미전달** (CLI 확장 여지).
- STATE 목적지가 단일 → 외부 controller 와 GUI 동시 수신은 GUI relay 또는
  state-ip 노출 필요.
- multi-vehicle: 데이터 포트는 차량 id 로 keyed 되어 있으나 sim 엔진은 현재 단일
  (SimSession N개 확장 시 wire/GUI 재작업 불요).

---

## 18.8 Self-check

<details>
<summary>1. SimSession 이 clock 을 안 갖는 이유?</summary>

clock·I/O 를 빼면 동일 kernel 이 free-run / external-step / replay 에 그대로
쓰인다. 페이싱은 RealTimeRunner 같은 얇은 wiring 이 담당.
</details>

<details>
<summary>2. ZOH latch 가 필요한 이유?</summary>

비동기 명령 stream(UDP/HTTP)과 고정-dt tick 을 분리. tick 은 항상 마지막 명령을
사용 — 실제 ECU/actuator 의 sample-and-hold 와 동일.
</details>

<details>
<summary>3. 폐루프 제어에 HTTP /api/io 가 부적합한 이유?</summary>

step 마다 요청/응답 round-trip + JSON 파싱이 지연·jitter 를 키운다. 결정론적
고빈도는 바이너리 UDP(§18.5).
</details>

---

## References

- `docs/vdsim_bridge_interface_requirements.md` — wire contract (§3-5).
- `docs/gui_architecture.md` — data port / 시각화 분리.
- Chapter 17 (actuator/sensor), Chapter 16 (FMI co-simulation).
