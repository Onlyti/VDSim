# Actuator Model Guide

VDSim 이 기본 제공하는 액추에이터 모델(데드타임, 1차 랙, 백래시, 레이트 제한, 포화, 데드존,
조향 LuGre 마찰, 브레이크 열 페이드)의 사용법과, 사용자 정의 단계(user stage) 확장법을 정리한다.
연구 코드에서 지연·랙을 파이썬 래퍼로 따로 구현할 필요가 없도록 하는 것이 목적이다.
모델 식과 근거 문헌은 `references/actuator_nonlinearity.md`, 이론은 `theory/17_actuator_and_sensing.md`.

## 1. 구조

```mermaid
flowchart LR
  U[명령 u] --> DZ[데드존]
  DZ --> DT[전송 지연<br/>dead_time_s]
  DT --> BL[백래시<br/>backlash]
  BL --> LAG[1차 랙<br/>tau_s]
  LAG --> RL[레이트 제한<br/>rate_limit]
  RL --> SAT[포화<br/>out_min/out_max]
  SAT --> US[사용자 단계<br/>user]
  US --> Y[실현 명령]
```

- 채널은 `steer`, `throttle`, `brake` 세 개이며 채널마다 같은 체인을 가진다.
- 모든 필드의 기본값은 꺼짐(0)이며, 전부 꺼져 있으면 기존 결과와 비트 단위로 동일하다.
- `steer` 는 선택적으로 서보+LuGre 마찰(`friction`) 모드를, `brake` 는 열 페이드(`thermal`)를 가진다.
- 조향 채널 단위는 노면 바퀴 각 [rad], `throttle`/`brake` 는 0..1 페달 비율이다.

## 2. 필드

| 필드 | 단위 | 의미 | 0 일 때 |
|---|---|---|---|
| `dead_time_s` | s | 순수 전송 지연 (분수 샘플 보간) | 없음 |
| `tau_s` | s | 1차 랙 시정수 | 없음 |
| `rate_limit` | 단위/s | 출력 변화율 상한 | 비활성 |
| `dead_zone` | - | 입력 데드밴드 후 재스케일 (페달 유격, 패드 간극) | 없음 |
| `out_min`, `out_max` | - | 포화 | 사실상 무제한 |
| `backlash` | - | 유격폭(전체 밴드). 입력이 출력에서 폭/2 이상 벗어나야 따라감 | 없음 |
| `user` | - | 사용자 정의 단계 (5절) | 없음 |
| `steer.friction` | - | LuGre `sigma0 sigma1 sigma2 Tc Ts ws` (켜면 서보 모드) | 꺼짐 |
| `steer.inertia/servo_kp/servo_kd` | - | 서보 모드 파라미터 | 기본값 |
| `brake.thermal` | - | `heat_coeff cool_coeff T_ambient mu_T_temp mu_T_scale` | 꺼짐 |

백래시는 play 연산자(속도 무관 히스테리시스)이다. 폭 0.02 에서 입력이 0.10 으로 올라가면 출력은 0.09,
입력이 0.095 로 내려가도 유지, 0.07 로 내려가면 0.08 이 된다.

## 3. 사용법 (파이썬)

```python
from vdsim_plant import VDSimPlant

plant = VDSimPlant(config="generic_sedan.yaml", level="L3",
                   control_dt=0.01, substep_dt=0.001,
                   actuator={
                       "steer":    {"dead_time_s": 0.05, "tau_s": 0.08, "backlash": 0.002},
                       "throttle": {"dead_time_s": 0.10, "tau_s": 0.25},
                       "brake":    {"dead_time_s": 0.08, "tau_s": 0.15, "dead_zone": 0.03},
                   })
plant.reset([0, 0, 0, 15.0, 0, 0])
plant.step([delta, fx_total])          # 계약은 그대로 u = [delta, Fx_total]
plant.actuator                          # 현재 ActuatorParams
plant.set_actuator({"throttle": {"tau_s": 0.5}})   # 롤아웃 도중 교체 (메모리 유지)
```

- `actuator` 인자는 `None`, `ActuatorParams`, 카탈로그 ID 문자열, dict 중 하나이다.
- dict 는 `"preset": "actuator.generic_sedan"` 키로 프리셋을 깔고 그 위에 필드를 덮어쓸 수 있다.
- 알 수 없는 키나 비유한 값은 `ValueError` 로 거부된다 (오타가 조용히 무시되지 않음).
- `set_actuator()` 는 전송 지연 링과 랙·서보·열 상태를 지우지 않고 파라미터만 바꾼다. 열화·고장 주입·지연 스텝에 쓴다.
- `reset()` 은 생성 시 지정한 액추에이터로 돌아간다.
- 직접 제어 세션(`make_direct_control_session`)은 `DirectControlSessionOptions.actuator` 로 같은 구조체를 받는다.
- 벡터 환경은 `VecSession` 에서 env 별로 교체한다.

## 4. 프리셋 (차량별 YAML)

위치는 `configs/parts/actuator/` 이고 `configs/catalog/manifest.yaml` 에 등록되어 있다 (type `actuator`, schema `actuator_v1`).

| ID | 내용 |
|---|---|
| `actuator.ideal` | 모든 채널 꺼짐 (기본 동작과 동일) |
| `actuator.generic_sedan` | 공칭 임시값 + 조향 백래시 0.002 |

새 프리셋은 YAML 하나를 추가하고 매니페스트에 등록하면 된다.
실차에서 측정한 차량 고유 값은 이 공개 카탈로그에 두지 않는다. 그런 값은 dict 나 `ActuatorParams` 로 실행 시에 넘긴다.

```yaml
id: actuator.my_vehicle
type: actuator
version: 1
schema: actuator_v1
body:
  steer:    {dead_time_s: 0.10, tau_s: 0.05}
  throttle: {dead_time_s: 0.08, tau_s: 0.20}
  brake:    {dead_time_s: 0.06, tau_s: 0.10, dead_zone: 0.02}
```

실측 로그가 있으면 임시값을 반드시 실측으로 교체한다. 프리셋의 임시값으로 얻은 결과는 논문 수치로 쓰지 않는다.

## 5. 사용자 정의 단계

내장 체인으로 표현되지 않는 동작(비선형 맵, 학습 모델, 온도 의존 지연 등)은 `user` 단계로 끼운다.
`user` 는 해당 채널의 내장 체인 출력 뒤에 실행된다. 내장 필드를 모두 0 으로 두면 체인 전체를 대체한다.

### 파이썬 (덕 타이핑)

```python
class Quantizer:
    def __init__(self, q): self.q = q
    def step(self, u, dt):            # 필수: 출력을 돌려준다
        return round(u / self.q) * self.q
    def reset(self): pass             # 선택: 에피소드 시작 시 호출
    def get_state(self): return []    # 선택: 스냅샷에 저장할 float 리스트
    def set_state(self, v): pass      # 선택

plant = VDSimPlant(..., actuator={"steer": {"user": Quantizer(0.001)}})
```

- 객체는 `step(u, dt)` 만 필수이며, 없으면 `TypeError` 가 난다. `reset` 이 없으면 호출이 생략되고, `get_state/set_state` 가 없으면 스냅샷이 이 단계의 내부 상태를 복원하지 못한다.
- 넘긴 객체는 원형(prototype)이며 직접 호출되지 않는다. env 마다 `copy.deepcopy` 한 복제본이 돌기 때문에 상태가 env 간에 공유되지 않는다.
- 호출은 물리 서브스텝마다 일어나며 GIL 을 잡는다. 순수 파이썬이라 무거운 연산을 넣으면 시뮬 속도가 떨어진다.
- `throttle`/`brake` 의 사용자 출력은 [0, 1] 로 다시 포화된다.

### C++ (`IChannelModel`)

```cpp
struct Quantizer : vdsim::IChannelModel {
    double q;
    explicit Quantizer(double q_) : q(q_) {}
    double step(double u, double) override { return std::round(u / q) * q; }
    void reset() override {}
    std::unique_ptr<IChannelModel> clone() const override { return std::make_unique<Quantizer>(*this); }
    // 선택: std::vector<double> get_state() const / void set_state(const std::vector<double>&)
};
params.steer.ch.user = std::make_shared<Quantizer>(0.001);
```

## 6. 종방향 채널의 동작 (직접 제어 경로)

- 직접 제어 입력은 휠별 토크이다. 구동 합과 제동 합을 `drive_ref = max_motor_torque * final_drive_ratio`, `brake_ref = max_brake_torque` 로 정규화해 `throttle`/`brake` 로 액추에이터에 통과시킨다.
- 이후 직전의 0 이 아닌 휠별 분배 비율로 되돌려 나눈다. 초기 분배는 `drive_split_front` 를 따른다.
- 기준값은 `DirectControlSessionOptions.drive_torque_ref/brake_torque_ref` (VDSimPlant 의 같은 이름 인자) 로 덮어쓸 수 있다.
- 종방향 경로는 `dead_time/tau/rate_limit/dead_zone/포화/backlash/user/브레이크 열` 중 하나라도 켜졌을 때만 활성화된다. 꺼져 있으면 기존 경로와 비트 동일하다.
- 주의: 음의 모터 토크(회생)는 액추에이터를 거치지 않고 그대로 통과한다.
- 주의: `throttle`/`brake` 는 단방향 0..1 채널이므로 기준값을 넘는 요청은 포화된다.
- 주의: 같은 틱에 구동과 제동을 동시에 요청하면 두 채널이 각자 처리된다.

## 7. 검증

- `tests/unit/test_actuator.cpp` (gtest): 체인 각 단계, 백래시, 사용자 단계, 스냅샷 왕복.
- `tests/scripts/test_plant_actuator.py` (ctest `plant_actuator`): 기본값 동일성, 데드타임+랙 해석해 일치(1e-9), 궤적과 휠 분배, 사용자 단계 비공유, 프리셋, 스냅샷 재생.
- 알려진 사항: L3 스냅샷 복원 시 psi, Y 에 1 ulp(부동소수점 최소 단위) 수준 불일치가 있다. 액추에이터가 없어도 재현되는 기존 현상이며 이번 변경과 무관하다. 테스트는 1e-9 허용오차를 쓴다.

## 8. 다른 서브시스템 현황 (조사 결과)

액추에이터와 같은 패턴(사용자가 래퍼에서 직접 구현)이 다른 영역에 있는지 저장소 기준으로 조사했다.
저장소 안에서 손으로 짠 대체 구현은 발견되지 않았고, 저장소 밖 래퍼는 이 조사 범위 밖이다.

| 영역 | 내장 모델 | 확장점 | 판정 |
|---|---|---|---|
| 제어기/센서 | 제어 사다리, 캐스케이드(PI 안티윈드업), 운전자 반응시간, 네트워크 지연, 센서 잡음/바이어스/1차 랙 | 캐스케이드 게인 `set_cascade_gains`, 센서 `lag_tau_s` (9절) | 반영됨. 사용자 정의 제어기 훅은 없음 (루프 외부에서 `step()`) |
| 서스펜션 | 스프링·댐퍼·ARB, 토폴로지, 운동학 | 파이썬/`.so` 모듈 (`SuspensionModule`) | 현 상태 유지 (`force()` 가 무상태여야 하는 제약 있음) |
| 타이어 | mf96/linear/MF2002, 이완 길이, LuGre, 카탈로그 | C++ `ITireModel` 만 (파이썬 트램폴린 없음) | 보강 권장: 파이썬 타이어 훅, 열·마모 상태 |
| 구동계 | 엔진 맵, 변속기, 디프, 변속 정책 콜백 | `DrivetrainModule` | EV 모터·배터리·회생 모델 부재 (EV 연구 시 필요) |
| 공력, 노면 | 상수 계수, mu 프로파일, ISO 8608 | 없음 | 현 상태 유지 |

후속 제안은 이 표의 "보강 권장" 항목 기준으로 별도 진행한다.

## 9. 제어기 캐스케이드 게인과 센서 랙

### 캐스케이드 게인 (`CascadeGains`)

`make_sim_session`/`make_vec_session` 세션은 L5~L8 명령을 내부 캐스케이드(속도 PI -> 가속도 PI -> 페달, 순추종, 요레이트 PI)로 낮춘다.
게인은 예전에 코드에 박혀 있었고 이제 세션마다 바꿀 수 있다. 기본값은 이전 하드코딩 값과 같아서 건드리지 않으면 결과가 비트 단위로 같다.

| 필드 | 기본값 | 의미 |
|---|---|---|
| `vx.kp`, `vx.ki`, `vx.i_max`, `vx.ax_clamp` | 0.8, 0.20, 3.0, 3.5 | L6 속도 루프. 출력은 목표 가속도 [m/s^2] |
| `ax.kp`, `ax.ki`, `ax.kd`, `ax.kff`, `ax.i_max` | 0.4, 0.6, 0.0, 0.10, 2.5 | L5 가속도 루프와 페달 전향 보상 |
| `pp_lookahead_min` [m], `pp_lookahead_k` [s] | 1.5, 0.40 | 순추종 전방주시 거리 Ld = max(min, k * vx) |
| `yaw_kp`, `yaw_ki` | 0.15, 0.05 | 요레이트 PI (L6 r_target -> 조향각) |

```python
g = vdsim.CascadeGains()
g.vx.kp = 1.2                      # 속도 루프를 더 공격적으로
sess.set_cascade_gains(g)          # 에피소드 중에도 가능, 적분기는 유지됨
sess.cascade_gains().vx.kp         # 현재 값 확인
vec.set_cascade_gains([g])         # VecSession: 길이 1 = 전 env 동일, 길이 N = env별
```

- 음수·NaN 게인은 `ValueError` 이고 기존 값은 그대로 남는다.
- 값은 `reset()` 과 도메인 랜덤화 뒤에도 유지된다 (차량 파라미터에서 오는 휠베이스·최대 조향각만 다시 읽는다).
- 직접 제어 세션(`VDSimPlant`)은 CmdL1 을 그대로 받아 캐스케이드를 쓰지 않으므로 이 게인은 영향이 없다. 그 경로의 제어기는 호출하는 쪽의 몫이다.

### 센서 랙 (`SensorNoise.lag_tau_s`)

각 센서 그룹(`imu_accel`, `imu_gyro`, `wheel_speed`, `steer`, `gnss_pos`, `gnss_vel`)에 1차 지연 시정수 `lag_tau_s` [s] 가 생겼다.
참값에 1차 저역통과(`y += (1 - exp(-dt/tau)) * (truth - y)`)를 먼저 적용하고 그 위에 바이어스·잡음을 더한다. 0 이면 필터가 없다.

```python
sp = vdsim.SensorParams()
sp.enabled = True
sp.gnss_vel.lag_tau_s = 0.15       # GNSS 속도 대역폭 제한
sess = vdsim.make_sim_session(vp, tp, "L2", sensors=sp)
```

- 첫 샘플은 참값으로 초기화되어 0 에서 올라오는 과도가 없다. `reset()` 에서 다시 초기화된다.
- 필터 상태는 스냅샷에 들어가 복원 후 비트 단위로 재생된다. 센서 스냅샷 레이아웃이 15+1 값만큼 늘었으므로 이전 버전이 만든 스냅샷은 복원할 수 없다.
- 센서가 꺼져 있으면(`enabled=False`) 항등이라 랙도 적용되지 않는다. 전송 지연은 별개의 `sensor_delay_s` 로 계속 쓴다.
- YAML 에서는 `gnss_vel: { noise_std: 0.05, lag_tau_s: 0.15 }` 로 쓴다.
- 검증: `tests/unit/test_sim_session.cpp` (SensorLag*, CascadeGains*), `tests/scripts/test_cascade_sensor.py` (ctest `cascade_sensor`).
