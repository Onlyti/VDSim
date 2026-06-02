# 10. Driver Model — Latency + Gaussian Noise

> **학습 목표.** Human driver 의 reaction time 을 시뮬에 어떻게 ring-buffer 로 표현하는지, Box-Muller 변환이 어떻게 두 uniform 으로 Gaussian 생성하는지 안다. Pure Pursuit + Lc6 + Lc5 cascade 의 외측 wrapping 으로 인간 운전자의 imperfection 을 추가하는 minimal model 을 가진 의의 (autonomous controller 와의 비교 baseline) 를 안다.

## 10.1 왜 Driver model 인가

차량 시뮬레이션의 controller 측은 두 영역:
1. **Autonomous controller** (Pure Pursuit / Stanley / MPC) — perfect, instant.
2. **Human driver** — imperfect (reaction delay, jitter), bounded actuation.

ADAS / autonomous 의 성능 평가 시 인간 driver baseline 이 필요. 또는 driver-in-the-loop simulation 의 surrogate.

VDSim 의 `DriverModel` 은 minimal driver:
- Pure Pursuit + Lc6 + Lc5 의 ideal cascade 위에:
- **Reaction time delay** (ring buffer).
- **Gaussian noise** (steer / throttle / brake 별).

## 10.2 Reaction time delay — ring buffer

인간의 visual-to-motor latency ≈ 150-300 ms (도시 driving 영역).

VDSim 의 구현:
```cpp
struct DriverModel {
    std::vector<double> steer_buffer_;
    int                 steer_idx_ {0};
    double              reaction_time_s {0.150};
};
```

매 step:
```
buf_size  =  round(reaction_time / dt)
steer_buffer_[steer_idx_]  =  pp_steer_now   // 현재 PP 출력 저장
steer_idx_                 =  (steer_idx_ + 1) % buf_size
delayed_steer              =  steer_buffer_[steer_idx_]   // buf_size step 전의 값
```

ring buffer 사이즈가 reaction time / dt 이므로 정확히 N step 의 delay.

low dt → buffer 크기 ↑ → memory 증가. PoC 의 dt = 5 ms, reaction = 150 ms → buffer size = 30. trivial.

### Discrete delay 의 한계

- **dt 의 정수배만 가능**. 150 ms ÷ 5 ms = 30 → OK. 175 ms ÷ 5 ms = 35 → OK. 150.5 ms 같은 비정수는 round 됨.
- **time-varying reaction** — 본 PoC 는 fixed. fatigue / 인지 부하 모델 미반영.

## 10.3 Gaussian noise — Box-Muller

driver 의 steer 와 throttle 에 zero-mean Gaussian noise 추가.

### Box-Muller 변환

두 uniform $u_1, u_2 \in (0, 1)$ 로 한 Gaussian $z \sim N(0, 1)$:

$$
z = \sqrt{-2 \ln u_1} \cdot \cos(2\pi u_2)
$$

derivation:
- 2D Gaussian 의 polar form: r² = −2 log(u1), θ = 2π · u2.
- (r cos θ, r sin θ) 두 독립 Gaussian.
- 본 PoC 는 첫 번째만 사용 (cos).

```cpp
const double u1 = clamp(rand_a, 1e-6, 1 − 1e-6);
const double u2 = rand_b;
const double z  = sqrt(−2 · log(u1)) · cos(2π · u2);

delayed_steer += g_.steer_noise_rms · z;
```

`rand_a`, `rand_b` 가 외부 입력 — controller 의 reproducibility 보장 (같은 seed 면 같은 결과).

### Noise level 의 의미

`steer_noise_rms` = 0.005 rad (default).
σ = 0.005 → 3σ ≈ 0.015 rad ≈ 0.9°.
실인간 driver 의 steer jitter 와 같은 order.

`thr_noise_rms` = 0.02 (default).
throttle 의 ±6 % 변동.

이 값들은 calibration 의 starting point. specific driver 의 measurement 와 fit 가능.

## 10.4 Cascade 통합

```
PurePursuit (Pure Pursuit + Lookahead)
    ↓ raw_steer
Reaction delay (150 ms ring buffer)
    ↓ delayed_steer
+ Gaussian noise
    ↓ noisy_steer (clamped to max_steer)

v_target  →  LongVxController (Lc6 PI)
              ↓ ax_target
            LongAxController (Lc5 PI + FF)
              ↓ throttle, brake
            + Gaussian noise
              ↓ noisy_throttle / noisy_brake

→ CmdL4 to dynamics
```

코드 `core/src/control_converter.cpp` 의 `DriverModel::update`.

### Throttle / brake noise

PoC 는 throttle 과 brake 에 같은 Gaussian z 를 추가 (correlation 1.0). 실제로는 independent 가 더 정확. 하지만 본 PoC 의 minimal model.

## 10.5 Reproducibility — 외부 uniform 입력

```cpp
auto out = drv.update(x, y, yaw, vx, v_target,
                       path_x, path_y, n,
                       dt,
                       rand_a, rand_b);   // 두 uniform [0, 1]
```

`std::mt19937_64` 등으로 외부에서 generate. seed 고정 → deterministic 결과.

`examples/driver_demo.cpp`:
```cpp
std::mt19937_64 rng(42);
std::uniform_real_distribution<double> uni(0, 1);
const double a = uni(rng), b = uni(rng);
const auto out = drv.update(..., a, b);
```

같은 seed 42 → 결과 bit-equal.

## 10.6 검증 (Task 53)

`DriverModel.*` 2 tests:
- `ReactionTimeDelaysSteer` — 100 ms 동안 buffer 가 0 의 steer 출력 후 실제 steer release.
- `NoiseIsBoundedByRMS` — 200 ticks, σ = 0.005, max steer ≤ 0.04 (8σ).

`vdsim_driver_demo` (end-to-end):
- sedan + figure-8 + v_target = 10.
- DriverModel (latency 150 ms, σ_steer = 0.005, σ_thr = 0.02) 가 PP perfect 대비 더 거친 steer 출력.
- 차량 trajectory 는 path 추적 잘 함 (saturation 의 sedan 한계).

## 10.7 Pure Pursuit perfect vs DriverModel — 비교 결과 (figure-8)

| Metric | Pure Pursuit (vdsim_path_tracking) | DriverModel (vdsim_driver_demo) |
|---|---|---|
| Steer | smooth | jittery + 150 ms delay |
| vx tracking | tight | slight overshoot / undershoot |
| Trajectory | path 가까움 | 약간 vibration |

이 두 binary 의 비교가 "autonomous 가 인간 대비 얼마나 깨끗한 driving 가능?" 의 quantification.

## 10.8 한계

| 항목 | 한계 |
|---|---|
| Reaction time | fixed (fatigue, distraction 미반영) |
| Noise 분포 | Gaussian (실제는 fat tail) |
| Throttle/brake correlated | independent 가 정확 |
| Anticipation | 없음 (path 앞쪽 look-ahead 가 lookahead distance 만) |
| Driver intent / mood | 미반영 |
| Force feedback | steering rack torque 입력 받지만 driver 반응 없음 |

본격 driver model — Werling 의 OpenDriver, IPG 의 IPGDriver — 는 30+ parameter.
본 PoC 는 minimal 5 parameter.

## 10.9 사용 패턴

```cpp
vdsim::DriverModel drv;
vdsim::DriverModel::Gains g;
g.wheelbase       = vp.wheelbase;
g.max_steer       = vp.max_steer_angle_wheel;
g.lookahead_min   = 2.0;
g.lookahead_k     = 0.45;
g.reaction_time_s = 0.150;
g.steer_noise_rms = 0.005;
g.thr_noise_rms   = 0.02;
g.vx_kp = 0.6; g.vx_ki = 0.15;
drv.initialize(g);

std::mt19937_64 rng(42);
std::uniform_real_distribution<double> uni(0, 1);

while (...) {
    const double a = uni(rng), b = uni(rng);
    auto out = drv.update(x, y, yaw, vx, v_target,
                           path_x, path_y, n, dt, a, b);
    vdsim::CmdL4 cmd;
    cmd.throttle = out.throttle;
    cmd.brake    = out.brake;
    cmd.steer_angle_wheel = out.steer;
    dyn->step(cmd, contacts, dt);
}
```

## 10.10 참고

- Hess, R.A. & Modjtahedzadeh, A., *A control theoretic model of driver steering behavior*, IEEE Control Systems Magazine, 1990 — classical driver model.
- MacAdam, C.C., *Application of an optimal preview control for simulation of closed-loop automobile driving*, IEEE Trans. SMC, 1981 — preview control.
- IPGDriver Reference Manual (IPG Automotive) — commercial driver model.
- VDSim 의 `examples/driver_demo.cpp` 가 minimal usage.
