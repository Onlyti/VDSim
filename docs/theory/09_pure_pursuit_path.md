# 09. Lc7-PathCurvature (Pure Pursuit) / Lc8-Waypoint

> **학습 목표.** Pure Pursuit 의 geometry 가 어떻게 lookahead 점에서 steer 를 계산하는지 식 단위로 유도한다. lookahead distance Ld 의 vx-dependent scheduling 의 이유 (low-vx 진동 방지, high-vx 부드러움) 를 안다. Pure Pursuit 의 강점 / 약점 (simplicity vs cross-track error in straight) 을 인지하고, Stanley / MPC 대안 의 입장을 명확히 한다.

## 9.1 Pure Pursuit 의 핵심 idea

차량 currently at `(x, y, ψ)`. path 는 waypoint sequence `{(xi, yi)}`. Pure Pursuit:

1. 차량 위치에서 path 위 한 점 (lookahead point) 을 정한다. distance = `Ld`.
2. 그 점을 통과하는 **원호** (rear axle 의 instantaneous center 통과) 가 되도록 front wheel 의 steer 결정.

geometry 가 모든 것을 결정 — feedback 도 없고 optimization 도 없다.

## 9.2 Geometry derivation

차량의 body frame: x_b forward, y_b leftward.
lookahead point 를 body frame 으로 변환 (회전 $R(-\psi)$):

$$
\begin{aligned}
(dx_w, dy_w) &= (x_{\text{target}} - x,\; y_{\text{target}} - y) \\
dx_b &= \cos\psi\, dx_w + \sin\psi\, dy_w \\
dy_b &= -\sin\psi\, dx_w + \cos\psi\, dy_w
\end{aligned}
$$

$dx_b$ 는 lookahead 까지의 forward distance, $dy_b$ 는 lateral offset (좌가 +).

**원의 기하**: rear axle 위치 (= body origin in bicycle model) 를 통과하는 원 + lookahead point 통과. 그 원의 반지름 `R_path` 와 곡률 `κ = 1/R_path`.

이등변삼각형 + 원의 정리 (또는 Pythagoras):

$$
R_{\text{path}} = \frac{L_d^2}{2\, dy_b}, \qquad L_d^2 = dx_b^2 + dy_b^2
$$

따라서 curvature:

$$
\kappa = \frac{2\, dy_b}{L_d^2}
$$

코드 `core/src/control_converter.cpp` 의 `PurePursuitController::update`:
```cpp
const double l2 = dx * dx + dy * dy;
if (l2 < 1e-6) return out;
const double kappa = 2.0 * dy / l2;
const double steer = std::atan(kappa * g_.wheelbase);
```

### Bicycle 모델에서 steer angle

kinematic bicycle:

$$
\kappa = \frac{\tan\delta}{L} \;\Rightarrow\; \delta = \arctan(\kappa L)
$$

여기서 $\delta$ 는 front wheel angle, $L$ 은 wheelbase.
작은 angle 영역: $\delta \approx \kappa L = 2\, dy_b\, L / L_d^2$.

## 9.3 Lookahead distance Ld 의 scheduling

constant Ld 의 문제:
- **너무 작음**: 작은 lateral error 에 큰 steer 반응 → 진동 (특히 정지 근처).
- **너무 큼**: corner 입장에서 일찍 cut, late-apex, 부드럽지만 path 추적 부정확.

VDSim 의 standard scheduling:

$$
L_d = \max(L_{d,\min},\; k\, v_x)
$$

low vx → $L_{d,\min}$ (default 1.5 m) — 정지 근처 진동 방지.
high vx → $k\, v_x$ (default $k = 0.40$ s) — vx 비례, 시야가 더 멀리.

`k = 0.40` 의 의미: 0.4 초 후 위치를 예측.

검증 (Task 32 + figure-8 demo):
- vx = 8 m/s, Ld = max(1.5, 3.2) = 3.2 m.
- figure-8 의 R = 20 corner 에서 sedan max_steer 0.5 rad 에 도달.

## 9.4 Lookahead index search

path 가 dense 한 waypoint sequence 라면:
```
for idx = prev_idx, ... , N−1:
    if distance(path[idx], (x, y)) >= Ld:
        break
return idx
```

`prev_idx` 부터 시작 — O(1) amortized.
`distance >= Ld` 조건 도달 못 하면 path 끝점 사용 (= `N − 1`).

코드 `core/src/control_converter.cpp` 의 lookup 부분.

## 9.5 Pure Pursuit 의 강약점

### 강점

- **Simple**. 한 점 (lookahead) 의 lateral offset 으로 steer 결정. 식 3 줄.
- **Stable**. saturation 발생해도 발진 없음 (geometry 기반).
- **No tuning**. Ld 하나만 결정하면 됨.
- **Real-time**. O(1) compute.

### 약점

- **Straight 에서 cross-track error 가 0 으로 수렴하지 않음**. heading 만 맞추고 lateral offset 은 lookahead 시야 만큼 잔여.
- **Sharp turn 에서 cut**. corner 안쪽으로 잘림.
- **Optimal 이 아님**. MPC 대비 차량 한계 활용 못함.

이 약점들이 MPC / SMPC paper 의 출발점 — receding-horizon optimization 으로 cross-track 0 수렴 + corner 한계 활용.

## 9.6 Stanley vs Pure Pursuit (간단 비교)

| 항목 | Pure Pursuit | Stanley |
|---|---|---|
| 기준점 | rear axle | front axle |
| Cross-track error | lookahead 의존 | 직접 minimize (k_e · cte / vx) |
| Heading error | indirect | 직접 minimize (heading_e) |
| 진동 | low | high gain 시 진동 |
| 표준 사용 | Roborace, FSK | Stanley 우승 차량 (DARPA 2005) |

본 PoC 는 Pure Pursuit. Stanley 추가는 1-day 작업.

## 9.7 Lc8-Waypoint — Path representation

`CmdL8` 의 PathPoint:
```cpp
struct PathPoint {
    double s;              // arc length [m]
    Vec2   xy;             // world coordinate
    double yaw;            // tangent direction [rad]
    double kappa;          // curvature [1/m]
    double v_des;          // desired speed
};
std::vector<PathPoint> path;
double lookahead_distance;
```

본 PoC 의 `vdsim_path_tracking` 은 `path[N]` 을 직접 array 로 받고 Pure Pursuit 호출. Lc8 의 std::variant variant 으로 dispatch 는 lower_to_l4 만 (steer 0 fallback). 본격 dispatch 는 ControlConverter cascade 외부.

### Arc length parameterization

`s` 가 arc length 면 path 의 미분 quantities (yaw, kappa, v_des) 가 자연스럽게 정의.

`kappa` 가 미리 계산되어 있으면 PurePursuit 결과의 sanity check 또는 feed-forward 로 활용 가능.

## 9.8 Implementation 의 한 가지 주의 — Frame

path 가 어느 frame 에 있는가?
- world frame (ENU, X north, Y east) — 본 PoC 의 가정.
- vehicle relative — controller 내부에서 변환.
- Frenet (s, d) — path 진행 방향 + lateral offset.

본 PoC 는 world frame 만. 본격 path tracking (특히 highway driving) 에서는 Frenet frame 이 표준.

## 9.9 검증 (Task 32)

`PurePursuit.*` 3 tests:
- `StraightAheadZeroSteer` — x-axis 직선 path, steer ≈ 0.
- `LeftCircleProducesPositiveSteer` — R = 20 원호, steer > 0, κ > 0.
- `MaxSteerClamped` — tight (R = 1) turn, |steer| ≤ max_steer (saturation).

End-to-end `vdsim_path_tracking`:
- sedan + figure-8 (R = 20) + v_target = 8.
- 결과: vx mean 5.79 ± 0.37 (under-tracking), steer max = 0.50 (saturated), 25 s.
- sedan max_steer 0.5 한계로 corner 에서 saturate. Pure Pursuit 자체는 정상.

## 9.10 한계 / follow-up

| 항목 | 한계 |
|---|---|
| Cross-track error 정확 측정 | lookahead distance 만 측정 (proxy) |
| 차량 한계 활용 | MPC 까지 가야 본격 |
| Frenet frame | 미지원 |
| Path smoothness 보장 | waypoint 의존 (cusp 처리 X) |
| MPC / LQR | Phase 2 |

## 9.11 사용 패턴

```cpp
vdsim::PurePursuitController pp;
vdsim::PurePursuitController::Gains g;
g.wheelbase = vp.wheelbase;
g.max_steer = vp.max_steer_angle_wheel;
g.lookahead_min = 2.0; g.lookahead_k = 0.45;
pp.initialize(g);

std::vector<double> px, py;
make_figure_eight(20.0, 80, px, py);

int prev_idx = 0;
while (...) {
    const auto out = pp.update(x, y, yaw, vx,
                                 px.data(), py.data(), (int)px.size(), prev_idx);
    prev_idx = out.idx;
    steer = out.steer;
    // 그 다음 v_target → ax_target → throttle/brake cascade
}
```

`examples/path_tracking_demo.cpp` 의 main loop.

## 9.12 참고

- Coulter, R.C., *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU Tech Report, 1992 (원본).
- Snider, J., *Automatic Steering Methods for Autonomous Automobile Path Tracking*, CMU Tech Report, 2009 (Pure Pursuit vs Stanley vs MPC).
- Werling, M. *et al.*, *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenét Frame*, ICRA 2010 (Frenet frame motion planning).
