# Task 20 — L1 / L2 격자 SS sweep 정량 비교

| Field | Value |
|---|---|
| Task ID | IM-W5-6 |
| Type | Validation |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

Task 19 의 single-case L1 vs L2 비교 (3 시나리오) 를 격자로 확장. linear region 에서의 일치 + non-linear region 에서의 정량 차이 측정. 동시에 본 task 가 L2 구현의 **체계적 회귀 보호** 역할.

이게 빠지면:
- linear region 에서만 일치 확인 → nonlinear 영역의 sim drift 가 보이지 않는다.
- per-tire lateral transfer 의 정량값 (수백~수천 N) 이 의미 있는 범위에 있는지 확인 불가.
- L2 구현 후속 수정 (Task 21+) 의 회귀를 잡을 baseline 부재.

## 2. 구현 방법

### 2.1 코드 / 데이터

| 위치 | 역할 |
|---|---|
| `docs/figures_src/sweep_l1_vs_l2.py` | (vx, delta) 격자 sweep → 각 cell 마다 동적 scenario YAML 생성 → `vdsim_l1_vs_l2` 호출 → CSV 파싱 → heatmap |
| 임시 scenario YAML | `tempfile.TemporaryDirectory` 안에서 cell 별 생성 |

격자 크기:
- `vx ∈ {5, 10, 15, 20} m/s`
- `delta ∈ {0.02, 0.04, 0.06, 0.08, 0.10} rad`
- 4 × 5 = 20 cells × 2 (L1, L2) = 40 sims, 5 s 각각.

### 2.2 메트릭

| 지표 | 의미 |
|---|---|
| `r_diff_pct = (r_L2 - r_L1) / r_L1 · 100` | L2 가 L1 대비 yaw rate 차이 (linear region 0% 기대) |
| `vy_diff   = vy_L2 - vy_L1` | side-slip velocity 차이 |
| `lat_transfer_f = Fz_FR - Fz_FL` | L2 의 front lateral load transfer |
| `lat_transfer_r = Fz_RR - Fz_RL` | rear 도 같음 |

### 2.3 부수적 발견 + 수정

본 sweep 작성 중에 발견된 **L2 의 `ay_prev_` 정의 버그** 를 함께 수정:
- 기존: `ay_prev_ = k.dvy` (frame velocity 변화율 only)
- 수정: `ay_prev_ = k.ay_body = Fy_total / m` (kinematic body-frame lateral accel = `dvy + vx·r`)

수정 전: max lateral transfer = 37 N (의미 없는 수준).
수정 후: max lateral transfer = 3693 N (분석값과 동일 차원).

이 수정은 본 task 의 sweep 데이터를 신뢰 가능하게 만들기 위한 필수 prerequisite. Task 19 의 7 test 는 모두 단조성 / 부호만 검증해서 ay 값 자체에 둔감했음 — 그래서 이 버그는 격자 sweep 에서만 노출되었다. 87/87 test 수정 후에도 모두 통과.

### 2.4 한계 / 가정

- **5 s 가 SS 도달에 충분한가?** linear region (작은 delta) 에서는 ~1 s 안에 settle. 큰 delta 에서는 tire saturation 이 transient 길게 함 — 5 s 도 충분하지 않을 수 있음. nonlinear cell 의 r_diff 일부는 transient 잔류 가능.
- **격자 해상도** — 4×5 = 20. 더 fine grid 는 별도 task.
- **상수 mu = 1.0** — split-mu / 노면 변화 미평가.
- **CarMaker / 실차 reference 없음** — 본 task 는 L1↔L2 의 self-consistency 만.

## 3. 검증 방법 (근거)

### 3.1 통과 기준

- **자료 신뢰성**: max |lat transfer| 가 ~1000 N 이상 (의미 있는 transfer 발생)
- **Linear region (delta ≤ 0.04, ay ≤ 4 m/s²)**: `|r_diff_pct| ≤ 10 %` (Task 19 의 SmallSteerYawRateMatchesBicycle 와 동일 기준)
- **Mass conservation**: 모든 cell 에서 `Σ Fz ≈ m g` 유지 (Task 19 test 가 보장)
- **Nonlinear region**: large diff 의 정량 측정 (격리 측정만, threshold pass/fail 없음 — 후속 task 의 reference)

### 3.2 한계

- nonlinear region 의 큰 차이 (~89%) 는 어느 모델이 맞는지 단정 못함. L1 의 axle 평균이 outer-wheel saturation 을 놓치고 yaw rate 과대평가 일 수 있고, L2 의 ay 1-step lag 이 SS 도달을 지연 시킬 수도 있음.

## 4. 검증 결과

### 4.1 Yaw rate 차이 heatmap

![Yaw rate diff](figures/sweep_yaw_rate_diff.png)

| vx \ delta | 0.02 | 0.04 | 0.06 | 0.08 | 0.10 |
|---:|---:|---:|---:|---:|---:|
| 5 m/s | ~0% | ~0% | ~0% | ~0% | ~0% |
| 10 m/s | ~0% | -0.2% | -1% | -3% | -5% |
| 15 m/s | -0.5% | -2% | -7% | -15% | -30% |
| 20 m/s | -1.5% | -7% | -25% | -55% | -89% |

해석:
- ay = vx · delta · 1/L · 보정 → vx=5 cell 모두 linear → 일치.
- vx=20 + delta=0.10 cell 의 -89% 는 ay ~ 7.4 m/s² 부근, outer wheel 의 Fz 가 saturate 직전 → L2 가 yaw 모멘트 감소, L1 (axle 평균) 은 그대로 → 큰 갭.
- Task 19 가 보장한 "linear region 일치" 범위가 본 격자에서도 확인됨.

### 4.2 vy 차이 (m/s)

![vy diff](figures/sweep_vy_diff.png)

side-slip velocity (`vy`) 차이 — L1 / L2 의 axle 별 force 분배 차이가 side-slip 에 누적. nonlinear cell 에서 최대 4.8 m/s 차이.

### 4.3 Lateral load transfer (L2 만)

![Lateral load transfer](figures/sweep_lateral_load_transfer.png)

| vx \ delta | 0.02 | 0.04 | 0.06 | 0.08 | 0.10 |
|---:|---:|---:|---:|---:|---:|
| Front (FR-FL) [N] | 25 ~ 50 | 200 | 700 | 1900 | 3700 |
| Rear  (RR-RL) [N] | 20 ~ 40 | 170 | 580 | 1600 | 3100 |

해석:
- Front 가 Rear 보다 ~20% 큼 (roll stiffness 분배 30000:25000 = 6:5 가 그 차이).
- ay → ay·m·h_cg/Tw 로 선형 (transfer 식 정의).
- vx=20, delta=0.10 의 3700 N 이 Fz_static_f ≈ 4086 N 의 90% → 거의 outer-only loading.

### 4.4 데이터 dump

`figures/l1_vs_l2_sweep.csv` 에 20 cells × 9 columns 전체 데이터.

## 5. 판단

- 결과: **pass**
- 근거:
  - Linear region (delta ≤ 0.04, vx ≤ 15) 에서 `|r_diff| ≤ 10 %` 만족.
  - lateral transfer 가 분석값과 동차원 (수천 N) 으로 측정됨 — L2 의 weight transfer 식이 의미 있게 작동.
  - 87/87 test 통과 유지.
  - **부수 수정**: ay_prev 정의 버그 (frame deriv vs body accel) 잡힘 → L2 가 정량 사용 가능 상태.
- 미해결 / Follow-up:
  - **5 s SS 도달 여부** — 큰 delta cell 에 대해 10-20 s sweep 재실행 (별도 task).
  - **격자 해상도 확대** — vx ∈ [1, 30], delta ∈ ±[0, 0.15], 100 cells.
  - **Nonlinear region 의 ground truth** — CarMaker / 실차 비교 (D17 Phase 2).
  - **L1 의 weight transfer 와의 비교** — Task 17 의 longitudinal transfer 와 L2 의 axle-sum 이 동일한지 cross-check (별도 unit test).
