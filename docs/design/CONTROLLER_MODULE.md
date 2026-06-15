# VDSim — Controller / Network / Subsystem 아키텍처 설계

작성: 2026-06-15  
상태: 확정 (설계 논의 완료, 구현 진행 중)

---

## 1. 전체 신호 흐름

```
User Algorithm (CmdLx: LcLon + LcLat, 레벨 L1~L8)
    ↓
[VehNetwork Module]        ← 차량 내부 통신 네트워크 모델 (ECU/CAN)
  - stochastic deadtime
  - packet drop
    ↓ delayed CmdLx (레벨 그대로)
[Subsystem]                ← 물리 부품 (cascade + 액추에이터 동역학)
  IBrakeSystem
  IDrivetrain  → DrivetrainOutput { wheel_torque, brake_absorbed }
  ISteeringSystem → SteeringOutput { mode, rack_travel | motor_force }
    ↓ (IDrivetrain 먼저, 잔여 → IBrakeSystem)
[ISteeringKinematics]      ← rack_travel → FL/FR wheel angles (Ld레벨별)
    ↓
[IVehicleDynamics]         ← 적분기 (RK4), Rack EOM 포함
```

---

## 2. VehNetwork Module (`IVehNetwork`)

### 역할

차량 내부 통신 네트워크(CAN bus, Automotive Ethernet 등)가 명령 신호에 미치는 물리적 효과만 모델링. 순수 계산 레이어 (물리 없음, 단 deadtime은 물리적 효과).

### 모델링 대상

| 효과 | 설명 |
|------|------|
| Stochastic deadtime | CAN 충돌·arbitration, ECU 연산 지연. `N(μ, σ²)` 분포 또는 bounded uniform |
| Packet drop | 메시지 유실. Bernoulli 확률 `p_drop` |
| On-drop policy | `HoldLast` / `Zero` / `Failsafe` |

### 모델링 제외

- 양자화: 현대 16-bit+ 시스템에서 무시 가능
- 신호 필터링, lag, rate limit: → Subsystem 담당 (물리 부품 특성)

### 인터페이스

```cpp
struct VehNetworkParams {
    double deadtime_mean {0.005};    // [s] 평균 deadtime
    double deadtime_std  {0.001};    // [s] 0 이면 deterministic
    double drop_rate     {0.0};      // [0,1] 패킷 유실 확률
    enum class OnDrop { HoldLast, Zero, Failsafe } on_drop {OnDrop::HoldLast};
    unsigned seed {42};
};

struct IVehNetwork {
    virtual ControlInput apply(const ControlInput& cmd, double dt) = 0;
    virtual void reset() {}
    virtual ~IVehNetwork() = default;
};
```

### 기본 구현

- `DefaultVehNetwork`: Gaussian deadtime (ring buffer + jitter) + Bernoulli drop

---

## 3. LcLon / LcLat 계층

```
LcLon (종방향):           LcLat (횡방향):
  L1: per-wheel torque      L1: steer torque [Nm]
  L2: axle torque           L2: angular accel [rad/s²]
  L3: Fx force              L3: angular velocity [rad/s]
  L4: throttle/brake [0,1]  L4: steer angle [rad]      ← 기본 인터페이스 경계
  L5: ax_target             L5: ay_target
  L6: vx_target             L6: r_target (yaw rate)
  L7: —                     L7: curvature κ [1/m]
  L8: —                     L8: waypoint path
```

**Lc4가 Subsystem ↔ 외부 알고리즘의 자연스러운 경계.** L4 아래는 Subsystem 내부에서 물리 처리.

---

## 4. Subsystem 역할 확장

Subsystem은 다음을 모두 담당:

1. Lx → L4 cascade (자신의 `min_level` ~ `max_level` 범위 내)
2. 액추에이터 물리 (lag, rate limit, saturation)
3. 최종 force/torque 출력

```cpp
struct ISteeringSystem {
    virtual LcLevel min_level() const { return LcLevel::L4; }
    virtual LcLevel max_level() const { return LcLevel::L4; }
    // 범위 밖 입력 → 런타임 에러 ("Lc level mismatch")
    virtual SteeringOutput apply(const SubsystemContext&, const LcLatCmd&) = 0;
};
```

### 에러 케이스

| 상황 | 처리 |
|------|------|
| input > max_level | 에러: "no cascade handler for LcN" |
| input < min_level | 에러: "LcN below subsystem minimum" |

---

## 5. 스티어링 서브시스템

### SteeringOutput — Kinematic / Dynamic 분기

```cpp
struct SteeringOutput {
    enum class Mode { Kinematic, Dynamic } mode {Mode::Kinematic};

    // Kinematic 모드: ISteeringSystem이 rack 위치 직접 결정
    double rack_travel {0.0};    // [m]

    // Dynamic 모드: ISteeringSystem이 motor force만 출력
    // Rack EOM은 IVehicleDynamics 내부 RK4에서 적분
    double motor_force {0.0};    // [N at rack]
};
```

| 모드 | 대상 | 타이어 킥백 / FFB |
|------|------|------------------|
| Kinematic | Ld1/2, 단순 시뮬 | 없음 |
| Dynamic | Ld3+, 레이싱 FFB, EPS 개발 | 있음 (Mz 피드백) |

### Rack EOM (Dynamic 모드, IVehicleDynamics 내부)

```
m_rack × ẍ_rack = F_motor - F_tire_feedback - c_rack × ẋ_rack

F_tire_feedback = Mz / R_pinion + Fy × caster_trail
```

- `rack_travel`, `rack_velocity` → State에 추가 (RK4 적분)
- 타이어 피드백은 Mz (이미 계산됨)으로 계산

### ISteeringKinematics

`rack_travel` → FL/FR 독립 wheel angle 변환 (ISuspensionKinematics와 동일 패턴).

```cpp
struct ISteeringKinematics {
    struct Output {
        double angle_fl {0.0};   // [rad]
        double angle_fr {0.0};   // [rad]
    };
    virtual Output compute(double rack_travel) const = 0;
    virtual ~ISteeringKinematics() = default;
};
```

| 구현 | 대상 |
|------|------|
| `RatioSteeringKinematics` | Ld1/2: `δ = rack_travel / ratio` (단일 평균) |
| `AckermannKinematics` | Ld3: FL/FR Ackermann 분리 |
| `HardpointSteeringKinematics` | Ld4: lookup table |

---

## 6. 구동/제동 분기 — EV 회생제동 지원

### DrivetrainOutput 확장

```cpp
struct DrivetrainOutput {
    std::array<double, NUM_WHEELS> wheel_torque;  // (+)drive, (-)regen
    double brake_absorbed {0.0};  // [0,1] IDrivetrain이 처리한 brake demand 비율
                                  // ICE: 0.0, EV: regen_fraction, Hybrid: 일부
};
```

### 플랫폼 실행 순서 (IVehicleDynamics 내부)

```
1. DrivetrainOutput dt_out = drivetrain->apply(ctx)
2. effective_brake = cmd.brake × (1.0 - dt_out.brake_absorbed)
3. BrakeOutput br_out = brake->apply(ctx, effective_brake)
4. net_torque[i] = dt_out.wheel_torque[i] - br_out.wheel_torque[i]
```

IBrakeSystem은 잔여 brake demand만 처리. regen 로직 불필요.

---

## 7. 구현 우선순위

| 항목 | 규모 | 우선 |
|------|------|------|
| VehNetwork Module (interface + DefaultVehNetwork) | 소 | P1 |
| DrivetrainOutput.brake_absorbed + 순차 실행 | 소 | P1 |
| ISteeringKinematics (interface + RatioSteering 구현) | 소 | P1 |
| SteeringOutput mode (Kinematic/Dynamic) | 소 | P1 |
| Rack EOM → State 확장 + IVehicleDynamics 연결 | 대 | P2 |
| LcLon/LcLat typed commands (control.hpp 확장) | 중 | P2 |
| Subsystem min/max level 선언 + 에러 체크 | 중 | P2 |

---

## 8. 하지 않을 것 (이번 구현)

- Rack EOM State 통합 (P2, Ld3+ 전용, 큰 변경)
- LcLon/LcLat 완전 typed variant 확장 (P2)
- ABS / Traction Control 내장 모듈

---

## 9. 관련 파일

```
core/include/vdsim/veh_network.hpp     VehNetwork 인터페이스 (신규)
core/include/vdsim/subsystems.hpp      IDrivetrain / ISteeringSystem 확장
core/include/vdsim/steering_kin.hpp    ISteeringKinematics (신규)
core/src/default_veh_network.cpp       DefaultVehNetwork 구현 (신규)
core/src/default_subsystems.cpp        DrivetrainOutput.brake_absorbed
docs/design/CONTROLLER_MODULE.md       이 문서
```
