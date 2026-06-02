# VDSim <-> AutoHYU-Control Interface Requirements (UDP Co-Simulation Contract)

Status: DRAFT contract for BOTH parties.
Audience: (1) AutoHYU-Control bridge developer, (2) VDSim developer.
Reference real-vehicle path: `can_transmitter` / `can_parser` (this repo).
Reference sim path: existing CarMaker bridge `src/bsw/simulator/carmaker_bridge`.

This document defines the wire interface across a PROCESS / MACHINE boundary between
the AutoHYU control stack and the VDSim vehicle-dynamics simulator. The boundary is
intentionally the same kind of decoupling as a real vehicle: the controller and the
"plant" run as independent processes (optionally on different machines) and exchange
fixed-format messages over a network link. There is NO shared memory, NO library
linkage, NO shared clock. VDSim is a standalone process; AutoHYU runs ROS.

---

## 0. Design principle — real-vehicle-equivalent decoupling

| Real vehicle | VDSim co-simulation |
|---|---|
| Control PC <-> CAN bus <-> ECU/actuators + sensors | AutoHYU bridge <-> UDP <-> VDSim process |
| `can_transmitter` encodes commands to CAN frames | bridge encodes commands to UDP cmd packet |
| `can_parser` decodes vehicle-state CAN frames | bridge decodes UDP state packet |
| Both sides run real-time, free-running, no shared clock | identical: free-run + timestamps |
| Loss / timeout -> actuator fail-safe | loss / timeout -> sim fail-safe (Section 8) |

Consequence: the bridge in this repo occupies the SAME architectural slot as
`can_transmitter`+`can_parser`. It is a protocol gateway (ROS <-> UDP), not a
simulator host. VDSim is responsible for running its own dynamics loop.

Confirmed decisions:
- Transport = UDP, fixed-length little-endian binary packets (Section 3-5).
- Timing = real-time free-run, both sides; every packet carries a timestamp (Section 8).
- Command abstraction = real-vehicle-equivalent pedals: the bridge sends
  {steering_tire [rad], throttle [0,1], brake [0,1], gear} (+ aux accel/speed),
  having already done torque->APS/BPS conversion exactly like the real ECU path.
- Baseline vehicle model in VDSim = L2 7-DOF (`create_seven_dof`).

---

## 1. Architecture

```
 +-------------------- AutoHYU host (ROS Noetic) --------------------+
 |  acados_scc_control / lateral / longitudinal / grip_manager       |
 |        |  app/con/gripped_steer|torque|accel  (ROS)               |
 |        v                                                          |
 |   vdsim_bridge  (ROS node == protocol gateway)                    |
 |    - sub gripped_* ; torque->APS/BPS ; pack CMD packet -----------)----> UDP :CMD_PORT
 |    - recv STATE packet ; unpack ; publish app/loc/vehicle_state <-)----- UDP :STATE_PORT
 +-------------------------------------------------------------------+
                                |  UDP (binary)  |
 +-------------------------- VDSim host (no ROS) --------------------+
 |   vdsim_udp_server (VDSim-side, NEW deliverable)                  |
 |    - recv CMD packet -> CmdL4 -> IVehicleDynamics::step(dt)        |
 |    - read State -> pack STATE packet -> send @ >=100 Hz            |
 |   create_seven_dof() + create_flat_ground() ; own real-time loop  |
 +-------------------------------------------------------------------+
```

Two unidirectional UDP streams (or one bidirectional socket pair):
- CMD stream  : AutoHYU bridge -> VDSim  (control -> plant)
- STATE stream: VDSim -> AutoHYU bridge  (plant -> control/sensors)

---

## 2. Responsibility split

### 2.1 VDSim side (deliverable for the VDSim team)
- Run a standalone process that owns the dynamics loop (`create_seven_dof`,
  `initialize(...)`, `create_flat_ground`, fixed-dt `step`), real-time free-run.
- Open a UDP socket; receive CMD packets (Section 4); map fields to `vdsim::CmdL4`
  (throttle, brake, steer_angle_wheel = steering_tire, gear, handbrake) and apply on
  the next step. NaN/out-of-range already clamped by `step()`.
- Send STATE packets (Section 5) at >= 100 Hz from `dyn->state()` + `ax_body_est()` /
  `ay_body_est()` + `wheel_spin`.
- Honor the fail-safe and timestamp rules (Section 8).
- Expose config: cmd listen port, state dest ip:port, rate, vehicle/tire/solver yaml,
  initial state, world origin convention.

### 2.2 AutoHYU bridge side (this repo, `src/bsw/simulator/vdsim_bridge`)
- Subscribe `app/con/gripped_steer` [deg, tire], `app/con/gripped_torque` [Nm],
  `app/con/gripped_accel` [m/s^2]; cache latest.
- Reproduce the real-vehicle longitudinal map: torque -> APS via
  `resources/torque_map/*.csv`, torque -> BPS via cubic poly (`interface_constants.hpp`
  BRAKE3_*, T_NEUTRAL). Send `throttle = APS/100`, `brake = BPS/100`.
- Convert steer: tire deg -> rad (NO steering-ratio multiply; VDSim takes wheel angle).
- Pack/send CMD packets @ 100 Hz; receive/unpack STATE packets.
- Publish `app/loc/vehicle_state` (`autohyu_msgs/VehicleState` + `VehicleCAN`) and
  latched `app/loc/reference_point` (`autohyu_msgs/Reference`, projection="local_cartesian").
- Apply fail-safe / staleness handling (Section 8).

---

## 3. UDP transport layer

| Item | Value (default, configurable) |
|---|---|
| Protocol | UDP, unicast |
| Byte order | little-endian |
| Packing | tightly packed, NO implicit padding (use fixed-width types; explicit pad bytes only) |
| Float type | IEEE-754 `double` (float64) for all physical quantities |
| CMD port (VDSim listens) | 7001 |
| STATE port (AutoHYU bridge listens) | 7002 |
| CMD rate (bridge -> VDSim) | 100 Hz |
| STATE rate (VDSim -> bridge) | >= 100 Hz (200 Hz recommended) |
| Max packet size | <= 512 bytes (single datagram, no fragmentation) |

Every packet begins with the common 24-byte header:

| off | type | field | meaning |
|---|---|---|---|
| 0  | uint32 | magic | 0x56445331 ("VDS1") |
| 4  | uint16 | version | protocol version = 1 |
| 6  | uint16 | msg_type | 1 = CMD, 2 = STATE |
| 8  | uint32 | seq | monotonically increasing per stream |
| 12 | uint32 | _pad | 0 (align payload to 8) |
| 16 | double | timestamp | sender clock [s] (CMD: bridge send time; STATE: sim time) |

Payload follows at offset 24. Integrity: last 4 bytes of every packet = `uint32`
CRC32 over all preceding bytes (header+payload). Receivers MUST drop packets with
wrong magic/version/CRC and MUST drop CMD packets whose `seq` is older than the last
accepted (out-of-order discard).

---

## 4. CMD packet (control -> VDSim), msg_type = 1

Header (24) + payload + CRC32 (4). All physical values SI.

| off | type | field | unit / convention |
|---|---|---|---|
| 24 | double | steering_tire_angle | [rad], ISO 8855, + = left, front wheel/tire angle |
| 32 | double | throttle | [0,1] accelerator pedal fraction (== APS/100) |
| 40 | double | brake | [0,1] brake pedal fraction (== BPS/100) |
| 48 | int32  | gear | +1 = D/forward, 0 = N, -1 = R |
| 52 | uint8  | handbrake | 0/1 |
| 53 | uint8[3] | _pad | 0 |
| 56 | double | aux_accel_target | [m/s^2] optional (ACC); NaN if unused |
| 64 | double | aux_speed_target | [m/s] optional; NaN if unused |
| 72 | uint32 | crc32 | over bytes [0,72) |

Total = 76 bytes. VDSim maps: `CmdL4.steer_angle_wheel = steering_tire_angle`,
`CmdL4.throttle`, `CmdL4.brake`, `CmdL4.gear`, `CmdL4.handbrake`. aux fields are
advisory only (VDSim MAY ignore in pedal mode; reserved for an accel-tracking mode).

---

## 5. STATE packet (VDSim -> control), msg_type = 2

Header (24) + payload + CRC32 (4). World frame = ENU (X east, Y north, Z up),
origin = the world origin VDSim was configured with (Section 6). Body frame = ISO 8855.

| off | type | field | unit / convention |
|---|---|---|---|
| 24  | double   | x_world | [m] ENU |
| 32  | double   | y_world | [m] ENU |
| 40  | double   | z_world | [m] ENU |
| 48  | double   | roll  | [rad] body, ZYX |
| 56  | double   | pitch | [rad] body |
| 64  | double   | yaw   | [rad] body, normalized (-pi,pi] |
| 72  | double   | vx | [m/s] body forward |
| 80  | double   | vy | [m/s] body left |
| 88  | double   | vz | [m/s] body up |
| 96  | double   | roll_rate  | [rad/s] body |
| 104 | double   | pitch_rate | [rad/s] body |
| 112 | double   | yaw_rate   | [rad/s] body |
| 120 | double   | ax_body | [m/s^2] (`ax_body_est()`) |
| 128 | double   | ay_body | [m/s^2] (`ay_body_est()`) |
| 136 | double[4]| wheel_spin | [rad/s] FL,FR,RL,RR |
| 168 | double   | steering_tire_angle_applied | [rad] steer actually applied this step |
| 176 | double   | wheel_radius_nominal | [m] so bridge computes wheel speed [m/s] = spin*r |
| 184 | double[4]| tire_Fz | [N] FL,FR,RL,RR (0 if not modeled) — reserved/L3 |
| 216 | uint32   | crc32 | over bytes [0,216) |

Total = 220 bytes. Wheel index order FL=0, FR=1, RL=2, RR=3 (repo convention).

Bridge -> `autohyu_msgs/VehicleState`: x,y,z <- x/y/z_world; vx,vy,vz; roll,pitch,yaw;
yaw_vel <- yaw_rate; ax,ay <- ax_body, ay_body; `vehicle_can.lateral_accel=ay`,
`.longitudinal_accel=ax`, `.yaw_rate=yaw_rate`, `.steering_tire_angle=steering_tire_angle_applied`,
`.wheel_velocity_{fl,fr,rl,rr}=wheel_spin[i]*wheel_radius_nominal`.

---

## 6. Coordinate / unit / frame conventions (binding for both sides)

| Quantity | Convention |
|---|---|
| World frame | ENU, right-handed. X=east, Y=north, Z=up. Origin = sim start (0,0,0) unless configured. |
| Body frame | ISO 8855. X=forward, Y=left, Z=up. |
| Angles | radians everywhere on the wire. yaw normalized to (-pi, pi]. |
| Steering sign | positive steering_tire_angle = left turn (+yaw). |
| Velocity / accel | body frame, SI (m/s, m/s^2). |
| Forces | N. Torque N·m. |
| Wheel order | FL=0, FR=1, RL=2, RR=3. |
| Map projection (ROS side) | bridge publishes `reference_point.projection="local_cartesian"`; downstream map/waypoint code operates directly in VDSim's metric ENU frame. ref_lat/ref_lon configurable for georeferencing. |

VDSim already runs native metric ENU + ISO 8855, so no projection is needed on the
sim side; the bridge passes positions straight through and only declares the reference.

---

## 7. Real-vehicle equivalence mapping (why these signals)

| Real vehicle (CAN, `can_transmitter`) | VDSim CMD packet field | Note |
|---|---|---|
| `target_steering_angle` (wheel->steering via ratio) | `steering_tire_angle` [rad] | VDSim takes tire angle directly; bridge skips the steering-ratio multiply the real ECU needs |
| `torque -> APS` (TorqueMap CSV) | `throttle` = APS/100 | identical TorqueMap reproduced in bridge |
| `torque -> BPS` (cubic, BRAKE3_*) | `brake` = BPS/100 | identical brake poly reproduced in bridge |
| `target_acceleration` | `aux_accel_target` | advisory; real ECU also receives this |
| `target_speed` | `aux_speed_target` | advisory (ACC) |
| gear selector | `gear` | D/N/R |

State feedback equivalent to `can_parser` (vehicle-state CAN): position/attitude from
localization-equivalent, body velocity/accel/yaw-rate, wheel speeds, applied steer.

---

## 8. Timing, fail-safe, health (real-vehicle-equivalent)

- Free-run, no lockstep. VDSim steps at its own fixed dt (e.g. 5 ms substeps,
  RK4) and emits STATE asynchronously; bridge sends CMD at 100 Hz. Each side uses the
  packet `timestamp` to detect lag, NOT to synchronize stepping.
- CMD timeout (VDSim side): if no valid CMD packet for `cmd_timeout` (default 100 ms),
  VDSim enters fail-safe — hold steering, throttle=0, apply moderate brake (configurable),
  exactly like an actuator watchdog on a real vehicle. Resume on fresh CMD.
- STATE timeout (bridge side): if no valid STATE for `state_timeout` (default 100 ms),
  bridge stops publishing fresh `vehicle_state` (or marks `operation_mode=FAIL`) so the
  control stack degrades safely.
- Out-of-order / duplicate: discard by `seq`.
- Both report `seq` + `timestamp`; round-trip and rate are observable for diagnostics.

---

## 9. Bridge ROS contract (AutoHYU-internal, for completeness)

sim -> control (bridge publishes): `app/loc/vehicle_state`
(`autohyu_msgs/VehicleState`, 100 Hz), `app/loc/vehicle_state_filtered` (optional),
`app/loc/reference_point` (`autohyu_msgs/Reference`, latched, "local_cartesian").

control -> sim (bridge subscribes): `app/con/gripped_steer` (`std_msgs/Float32`, deg),
`app/con/gripped_torque` (`std_msgs/Float32`, Nm), `app/con/gripped_accel`
(`std_msgs/Float32`, m/s^2).

Wire into `launch/simulator.launch` under `mode=='vdsim'` (sibling of carmaker/morai/carla
branches). Objects/perception topics out of scope (no traffic source in VDSim yet).

---

## 10. Versioning & extension

- `version` in header gates format changes; bump on any field layout change.
- Reserved/expandable: tire_Fz already present for L3; future susp_compression[4],
  slip[4], surface mu can be appended with a new `version` (receivers parse by length).
- A JSON debug mode MAY mirror the same field set for human inspection but is NOT the
  control-loop transport.

---

## Appendix A. VDSim C++ API used on the sim side (confirmed)

```
auto dyn = vdsim::create_seven_dof();
dyn->initialize(VehicleParams::from_yaml(vp), TireParams::from_yaml(tp), SolverParams{});
dyn->reset(State{/* x0,y0,yaw0,vx0 */});
auto ground = vdsim::create_flat_ground(0.0, 1.0);
// real-time loop @ dt:
vdsim::ContactArray contacts; ground->query(dyn->state(), vparams, contacts);
vdsim::CmdL4 u{throttle, brake, steer_rad, gear, handbrake};
dyn->step(u, contacts, dt);
const vdsim::State& s = dyn->state();
// pack: s.position, s.velocity (body), s.yaw(), s.yaw_rate(), s.wheel_spin,
//       dyn->ax_body_est(), dyn->ay_body_est(), vparams.wheel_radius_nominal
```
