# T30 — Ld4 5-link rear suspension

## 목적

Ld4 framework 의 **4번째 (가장 일반적) suspension type**. 모든 multilink
의 general form — 다른 type 들 (DW, MP, trailing arm) 은 사실 이 5-link 의
특수 케이스 또는 단순화.

## 모델

5 개의 독립적 양단 ball joint rigid link 가 chassis 와 knuckle 을 연결.

- Knuckle 6 DOF
- 각 link: 1 length constraint
- 5 links → 5 constraints
- Net: **1 DOF** (wheel travel)
- 각 link 자체 축 회전: passive DOF (knuckle 운동에 영향 없음)

Gruebler 정합:
```
M = 6·(N − 1) − Σc
  = 6·6 − (5×3·2)        # 6 bodies (5 links + knuckle + chassis), 10 ball joints (3 each)
  ... 계산은 5 passive 빼고 1 active
```

## Hardpoints (`configs/suspensions/5link_rear_sports.yaml`)

5 link, 각 link 마다 chassis-side + knuckle-side ball joint:

```yaml
links:
  upper_fore:  {chassis: [...], knuckle: [...]}
  upper_aft:   {chassis: [...], knuckle: [...]}
  lower_fore:  {chassis: [...], knuckle: [...]}
  lower_aft:   {chassis: [...], knuckle: [...]}
  toe_link:    {chassis: [...], knuckle: [...]}   # 사실상 tie rod 역할
```

## Solve

Knuckle pose = `(x, y, z, axis-angle 3-vec)` = 6 DOF unknown.

```python
def residuals(pose):
    pos, axis_angle = pose[:3], pose[3:6]
    R = rodrigues(axis_angle)
    return [
        |pos + R @ off_knuckle[i] - chassis[i]| - L_link[i]
        for i in range(5)
    ] + [pos[2] - target_wz]      # close the wheel-travel input
# 6 residuals on 6 DOFs → fully determined; least_squares LM
# Continuation across sweep for monotone solution family
```

## 검증

```
Solver success rate     : 17 / 17
Max wheel-z error       : 0.231 μm
Camber gain             : +0.0186 °/mm
Toe gain                : -0.0261 °/mm
Track gain              : +0.0153 mm/mm
Link lengths            : [0.361, 0.361, 0.377, 0.377, 0.345]
```

- camber 0.019°/mm — sports rear 의 매우 tight camber 제어 (5-link 의 장점)
- toe -0.026°/mm — 정상 toe-in bump-steer
- track change 매우 작음 — 5-link 의 또 다른 장점

## 한계

- 부싱 compliance 없음 (Phase 2)
- 5-link 의 anti-lift / anti-squat geometry 분석 미포함 — 종방향 motion 시
  pitch reaction 분석은 추가 작업
- 5개 link 의 ball joint singularity / 경계 조건 처리 robust 화 가능 (현재
  continuation 으로 회피)
