# Task 03 — D9 핵심 인터페이스 4종 명세

| Field | Value |
|---|---|
| Task ID | D9 |
| Type | Design |
| Date | 2026-05-28 |
| Commit | `1e15c5e` |
| Status | completed |

## 1. 목적

`IVehicleDynamics`, `ITireModel`, `IContactProvider`, `IRoughnessProvider` 의 가상 메서드 시그니처와 lifetime / 에러 정책 확정. CARLA plugin 과 core 간의 ABI 경계가 여기서 결정된다.

## 2. 구현 방법

### Class diagram (논리)

```
IVehicleDynamics  ───uses───>  ITireModel
        │                            │
        ├──uses──> ContactArray  <───┘ (Fz, mu input)
        │
        └──uses──> ControlInput (variant)

IContactProvider  ──produces──> ContactArray
IRoughnessProvider ─produces──> double (height)
```

### IVehicleDynamics

```cpp
class IVehicleDynamics {
public:
    enum class Level { L1_Bicycle, L2_SevenDOF, L3_FourteenDOF };
    virtual Level level() const noexcept = 0;
    virtual void  initialize(VehicleParams&, TireParams&, SolverParams&) = 0;
    virtual void  reset(const State&) noexcept = 0;
    virtual void  step(const ControlInput&, const ContactArray&, double dt) noexcept = 0;
    virtual const State& state() const noexcept = 0;
    // diagnostics
    virtual std::array<Vec3,   NUM_WHEELS> tire_forces_body()  const = 0;
    virtual std::array<double, NUM_WHEELS> tire_Fz()           const = 0;
    virtual std::array<double, NUM_WHEELS> wheel_slip_ratio()  const = 0;
    virtual std::array<double, NUM_WHEELS> wheel_slip_angle()  const = 0;
};
std::unique_ptr<IVehicleDynamics> create_bicycle();
std::unique_ptr<IVehicleDynamics> create_seven_dof();      // throws (Phase 1 W9-W10)
std::unique_ptr<IVehicleDynamics> create_fourteen_dof();   // throws (Phase 1 W11-W12)
```

### 핵심 결정

| 결정점 | 채택 | 근거 |
|---|---|---|
| step(u, contacts, dt) | contacts 인자로 전달 | Dynamics 는 pure computation, test 에서 contact 직접 주입 가능 |
| step() noexcept | yes | RT loop 에서 throw 금지. NaN / 이상치는 log + clamp |
| L1 에서 contact 사용 | yes | 일관성 — slope/mu 활용. bicycle 도 동일 시그니처 |
| Factory function vs ctor | Factory | impl 은 anonymous namespace 안에 hidden, ABI 안정 |
| Lifetime | `std::unique_ptr` | heap-only, UE5 GC 미관여 |
| initialize() / reset() | throw 허용 | setup 단계, RT 외 |

### ITireModel

```cpp
struct Input  { double Fz, kappa, alpha, mu_long, mu_lat, Vx_wheel; };
struct Output { double Fx, Fy, Mz; };
virtual Output compute(const Input&) const noexcept = 0;
```

Pacejka / Linear 의 공통 인터페이스. PoC 는 long/lat 만 (Mz=0).

### IContactProvider

```cpp
virtual void query(const State&, const VehicleParams&, ContactArray& out) = 0;
```

CARLA 의 raycast 와 자체 flat-ground 모두 동일 인터페이스. CARLA impl 은 `carla_integration/plugin/` 측.

### IRoughnessProvider (Phase 2 reserve)

```cpp
virtual double sample_height(const Vec2& world_xy) const = 0;
```

ISO 8608 PSD 는 미구현 (throw on factory).

## 3. 검증 방법 (근거)

명세 단계는 빌드 가능성 + factory 가 unique_ptr 반환 가능성만 확인. 본격 검증은 후속 impl task (10, 11) 의 단위/통합 테스트.

## 4. 검증 결과

| 항목 | 결과 |
|---|---|
| 모든 인터페이스 header include 시 빌드 | pass (commit `1e15c5e`) |
| Factory function 으로 abstract class 인스턴스 생성 (task 10, 11) | pass |
| step() noexcept 보장 (bicycle impl) | pass — invalid ControlInput 시 zero command 로 fall back |
| L2/L3 factory throw | pass (`BicycleStubs.SevenAndFourteenDoFThrow`) |

## 5. 판단

- 결과: **pass**
- 근거: 후속 impl (Pacejka MF96, Linear tire, Bicycle, FlatGround) 가 모두 본 인터페이스를 구현하여 빌드 + 37/37 test 통과. ABI 변경 없이 확장 가능 구조.
- Follow-up:
  - `IRoughnessProvider` 의 PSD 구현 (Phase 2).
  - `IContactProvider` 의 CARLA raycast impl (W7-W10).
  - `Mz` aligning moment 활성화 (Phase 2 MF2002).
