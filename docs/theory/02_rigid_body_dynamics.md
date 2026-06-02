# 02. Rigid-Body Dynamics — Newton-Euler in the Body Frame

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. body frame 에서의 Newton 방정식이 왜 $m(\dot v + \omega \times v)$ 형태가
   되는지 frame 도함수로부터 유도한다.
2. Coriolis 항 $\omega \times v$ 가 어디서 오는지, IMU 가 측정하는 양이
   $\dot v_{\text{body}}$ 가 아닌 이유를 설명한다.
3. planar 차량에서 Euler 회전방정식이 $I_{zz}\dot r = M_z$ 로 줄어드는 과정을
   cross 항 소거로 보인다.
4. "3 종류의 $a_x$" (frame 도함수 / kinematic / IMU) 를 구분하고, weight
   transfer 에 들어가는 것이 어느 것인지 판별한다.

## Prerequisites

- **Chapter 01** — frame 정의, body→world quaternion, 부호 약속.
- **고전역학** — Newton-Euler, 관성 텐서, 각운동량.
- **외부 reference** — Genta §3.2-3.3 (body-fixed Newton-Euler), Rajamani §2 (SAE 부호 주의).

---

## 2.1 동기 — 왜 body frame 에서 푸는가

차량의 질량과 관성 텐서는 body 에 고정된 좌표계에서만 시간 불변이다. body 가
yaw 하면 world 좌표 입장에서 차의 관성이 매 순간 회전한다 — 즉 $I_{\text{world}}(t)$
가 매 step 마다 변한다. inertial frame 에서 풀려면 이 시간 의존성을 매번
계산해야 한다.

body frame 에서 풀면 $I_{\text{body}} = \text{const}$ (보통 대각) 이고, 대신
frame 자체가 회전하는 효과를 **Coriolis 항** 으로 더한다. 그것이
$\omega \times v$ 와 $\omega \times (I\omega)$ 다.

---

## 2.2 가정

본 chapter 의 식은 다음 가정 위에서 유도된다.

| 가정 | 의미 | 깨지는 case |
|---|---|---|
| Sprung body 단일 강체 | sprung + unsprung 을 한 덩어리로 | chapter 06 (unsprung 분리), chapter 13 (multibody) |
| 관성 텐서 대각 | $I_{xy}=I_{xz}=I_{yz}=0$ (대칭 차량) | 비대칭 적재 (out of scope) |
| CG 위치 고정 | fuel slosh / payload shift 무시 | out of scope |
| Planar (p=q=0) for Ld1/Ld2 | roll/pitch DOF 비활성 | chapter 06 (Ld3 가 활성) |

---

## 2.3 위치 / 속도의 frame 변환

$(i, j, k)$ 를 inertial frame 단위 벡터, $(e_1, e_2, e_3)$ 를 body frame 단위
벡터라 하자. 한 점 P 의 inertial 위치 $r$ 와 body 위치 $r_b$:

$$
r = R(t)\, r_b + r_O(t)
$$

여기서 $R(t)$ 는 body→world 회전, $r_O(t)$ 는 body 원점의 world 위치. 미분:

$$
\dot r = R\, \dot r_b + \dot R\, r_b + \dot r_O
$$

강체 운동학의 표준 결과 $\dot R = R\, [\omega_{\text{body}}]_\times$ ($[\omega]_\times$
는 skew matrix) 를 대입하면:

$$
\dot r = R\, (\dot r_b + \omega_{\text{body}} \times r_b) + \dot r_O
$$

P 가 body 위에 고정 ($\dot r_b = 0$) 이고 CG ($r_b = 0$) 이면 $\dot r_{CG} = \dot r_O = v_{\text{world}}$.

---

## 2.4 Body frame 가속도 — Coriolis 항

body frame 속도 $v_{\text{body}} = R^T v_{\text{world}}$. lab frame 에서 본
도함수와 body frame 자체에서 본 도함수는 다르다.

$$
\left.\frac{d}{dt} v_{\text{world}}\right|_{\text{world}}
= R\,(\dot v_{\text{body}} + \omega_{\text{body}} \times v_{\text{body}})
$$

좌변은 inertial 가속도 $a_{\text{world}}$. 양변에 $R^T$:

$$
R^T a_{\text{world}} = \dot v_{\text{body}} + \omega_{\text{body}} \times v_{\text{body}}
$$

![Coriolis term in body frame](figures/02_coriolis.png)

위 그림은 body velocity $v_{\text{body}}$, yaw rate $\omega$, 그리고 그 외적이
만드는 centripetal 성분 $\omega \times v$ 의 관계를 보여준다. 핵심은 **차체에
탑승한 관측자(IMU)가 측정하는 가속도가 $\dot v_{\text{body}} + \omega \times
v_{\text{body}}$ 이지 $\dot v_{\text{body}}$ 가 아니라는 것**이다. 이 구분이
§2.7 의 "3 종류의 $a_x$" 와 weight transfer 정확도의 근원이다.

---

## 2.5 Newton 방정식 (body frame)

Newton $m\, a_{\text{world}} = F_{\text{world}}$ 의 양변에 $R^T$:

$$
m\,(\dot v_{\text{body}} + \omega_{\text{body}} \times v_{\text{body}}) = F_{\text{body}}
$$

planar 차량 ($\omega = (0, 0, r)$):

$$
\omega \times v_{\text{body}} = (0, 0, r) \times (v_x, v_y, v_z) = (-r v_y,\; r v_x,\; 0)
$$

따라서:

$$
\begin{aligned}
m\, \dot v_x - m\, r\, v_y &= F_{x,\text{body}} \\
m\, \dot v_y + m\, r\, v_x &= F_{y,\text{body}} \\
m\, \dot v_z &= F_{z,\text{body}} - m g
\end{aligned}
$$

좌선회 ($r > 0$) 에서 $-r v_y$, $+r v_x$ 항이 centripetal 가속도의 body-frame
표현이다.

---

## 2.6 Euler 회전방정식 (body frame)

각운동량의 body-frame 도함수도 Coriolis 항을 포함한다.

$$
I_{\text{body}}\, \dot\omega_{\text{body}} + \omega_{\text{body}} \times (I_{\text{body}}\, \omega_{\text{body}}) = M_{\text{body}}
$$

$I_{\text{body}} = \operatorname{diag}(I_{xx}, I_{yy}, I_{zz})$ 가정 시:

$$
\begin{aligned}
I_{xx}\, \dot p + (I_{zz} - I_{yy})\, q r &= M_x \\
I_{yy}\, \dot q + (I_{xx} - I_{zz})\, p r &= M_y \\
I_{zz}\, \dot r + (I_{yy} - I_{xx})\, p q &= M_z
\end{aligned}
$$

planar ($p = q = 0$) 이면 cross 항이 모두 0 으로 사라져:

$$
I_{zz}\, \dot r = M_z
$$

roll/pitch 가 활성화되는 Ld3 (chapter 06) 에서는 small-angle linearization 으로
$I_{xx}\ddot\phi$, $I_{yy}\ddot\theta$ 를 별도로 다루며 cross 항은 무시한다. 본격
multibody (chapter 13) 에서 cross 항을 다시 살린다.

---

## 2.7 세 종류의 $a_x$ — 자주 혼동되는 정의

차량 동역학에는 이름이 같은 세 가지 가속도가 있다.

| | 정의 | 용도 |
|---|---|---|
| frame velocity 도함수 | $\dot v_x$ | ODE 의 state derivative |
| kinematic body-x accel | $a_x = \dot v_x - v_y r = F_x/m$ | weight transfer 의 $a_x$ |
| IMU 측정값 | $a_x + g\sin(\text{pitch})$ | 센서 모델 (평지면 (2)와 동일) |

weight transfer 식 (chapter 04-05) 에 들어가는 것은 **(2) kinematic accel
$F_x/m$** 이다. 만약 (1) $\dot v_x$ 를 쓰면 $v_y r$ 성분이 빠져, steady-state
cornering 에서 lateral transfer 가 0 으로 계산되는 오류가 생긴다. 이 함정의
상세 사례는 §2.10 의 implementation note 에 둔다.

---

## 2.8 모든 모델 사다리의 공통 EoM

$$
\begin{aligned}
m\, \dot v_x  &= F_{x,\text{total}} + m\, v_y\, r \\
m\, \dot v_y  &= F_{y,\text{total}} - m\, v_x\, r \\
I_{zz}\, \dot r &= M_{z,\text{total}}
\end{aligned}
$$

World 적분 (yaw-only, chapter 01 §1.6):

$$
\dot x_w = v_x \cos\psi - v_y \sin\psi, \quad
\dot y_w = v_x \sin\psi + v_y \cos\psi, \quad
\dot\psi = r
$$

모델 사다리 사이의 차이는 오직 **$F_{x,\text{total}}, F_{y,\text{total}},
M_{z,\text{total}}$ 을 어떻게 구하는가** 뿐이다. base EoM 은 동일하다.

| 사다리 | 힘/모멘트의 출처 |
|---|---|
| Ld1 | per-axle (front, rear) — bicycle |
| Ld2 | per-tire (4) + weight transfer |
| Ld3 | per-tire + dynamic suspension |
| Ld4 | per-tire + multibody kinematic |
| Ld5 | per-tire + multibody compliant |

---

## 2.9 검증 전략

| 검증 | 식 / 케이스 |
|---|---|
| Coriolis 부호 | 좌선회에서 $-r v_y$, $+r v_x$ 가 centripetal 방향과 일치 |
| planar 축약 | $p=q=0$ 입력 시 Euler 식이 $I_{zz}\dot r = M_z$ 로 일치 |
| kinematic $a_x$ | $F_x/m$ 이 $\dot v_x - v_y r$ 와 일치 |
| 해석 비교 | linear bicycle steady-state yaw rate 가 Genta 식과 부호·크기 일치 (±10 %) |

세부 test 매핑은 §2.11 의 implementation note 참조.

---

## 2.10 한계

| 한계 | 영향 | 다루는 chapter |
|---|---|---|
| 관성 텐서 비대각 | 비대칭 차량의 yaw-roll coupling 누락 | out of scope |
| Euler cross 항 무시 (Ld3) | 큰 roll/pitch rate 에서 오차 | chapter 13 (full multibody) |
| CG 고정 | fuel slosh / payload | out of scope |
| 1-step lag (kinematic $a_x$) | transient 동안 small bias | chapter 11 (수치 적분) |

---

## 2.11 다음 chapter 와의 연결

base EoM 이 정해졌으므로, $F$ 와 $M$ 을 채우는 첫 요소가 **타이어**다.
chapter 03 은 Pacejka Magic Formula 로 slip 으로부터 $F_x, F_y, M_z$ 를 구하는
방법을 다룬다. 그 힘을 body frame 으로 모아 위 EoM 에 넣으면 Ld1 (chapter 04)
의 single-track 모델이 완성된다.

---

## 2.12 참고문헌

- **Genta, G.** *Motor Vehicle Dynamics*, World Scientific, 2014. §3.2 body-fixed reference, §3.3 Newton-Euler.
- **Rajamani, R.** *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. §2 — SAE 좌표계 주의.
- **Featherstone, R.** *Rigid Body Dynamics Algorithms*, Springer, 2008. Ch.2 강체 운동학 — multibody 진입 전 필독.

---

## 2.13 Self-check

<details>
<summary>1. body frame 에서 EoM 을 푸는 핵심 이점 한 가지는?</summary>

관성 텐서 $I_{\text{body}}$ 가 시간 불변 (보통 대각) 이라, 매 step 마다
$I_{\text{world}}(t)$ 를 재계산할 필요가 없다. 대신 frame 회전 효과를 Coriolis
항으로 더한다.
</details>

<details>
<summary>2. IMU 가 측정하는 lateral 가속도는 <code>v̇y</code> 인가 아닌가?</summary>

아니다. $\dot v_y + v_x r$ 을 측정한다 (Coriolis 항 포함). 정상상태 선회에서는
$\dot v_y \approx 0$ 이지만 $v_x r$ 이 남아 centripetal 가속도를 측정한다.
</details>

<details>
<summary>3. planar 차량에서 Euler 식이 <code>Izz·ṙ = Mz</code> 로 줄어드는 이유는?</summary>

$p = q = 0$ 이면 cross 항 $(I_{zz}-I_{yy})qr$, $(I_{xx}-I_{zz})pr$,
$(I_{yy}-I_{xx})pq$ 가 모두 0 이 되어, yaw 식만 $I_{zz}\dot r = M_z$ 로 남는다.
</details>

<details>
<summary>4. weight transfer 식에 <code>v̇x</code> 를 쓰면 어떤 오류가 생기는가?</summary>

steady-state cornering 에서 $\dot v_x \approx 0$ 이지만 실제 longitudinal
가속도 $F_x/m = \dot v_x - v_y r$ 은 $-v_y r$ 만큼 존재. $\dot v_x$ 를 쓰면 이
성분이 빠져 lateral weight transfer 가 비현실적으로 작게 나온다.
</details>

<details>
<summary>5. <code>ω × v_body = (−r·vy, r·vx, 0)</code> 에서 좌선회 시 x 성분이 음인 이유는?</summary>

좌선회 $r>0$, 정상상태에서 $v_y$ 는 약간 음 (차가 안쪽으로 미끄러짐) →
$-r v_y > 0$ 이 아니라 부호는 $v_y$ 에 의존. 핵심은 이 항이 forward 축으로
투영된 centripetal 성분이라는 것 — 순수 $\dot v_x$ 와 분리해야 한다.
</details>

---

## 2.14 VDSim 구현 노트

> **[VDSim impl] § 2.5 — body EoM 코드**
>
> `core/src/bicycle_dynamics.cpp:208-210`:
>
> ```cpp
> d_out.dvx = Fx_total / m + vy * r;
> d_out.dvy = Fy_total / m - vx * r;
> d_out.dr  = Mz_total / Izz;
> ```
>
> §2.8 의 base EoM 그대로. 모든 사다리 (Ld1-Ld3) 가 이 형식을 공유한다.

> **[VDSim impl] § 2.7 — kinematic $a_x$ 의 명시 저장**
>
> `core/src/seven_dof_dynamics.cpp:353-354`:
>
> ```cpp
> d_out.ax_body = Fx_total / m;   // NOT dvx — weight transfer input
> d_out.ay_body = Fy_total / m;
> ```
>
> weight transfer 에 들어가는 것이 (2) kinematic accel 임을 코드에서 명시.
> 초기 구현에서 `ay_prev = dvy` (frame 도함수) 를 저장했다가, 정상상태 선회의
> lateral transfer 가 비현실적으로 작게 (수십 N) 나오는 버그가 있었다. `Fy/m`
> 로 수정 후 예상 차원 (수천 N) 으로 수렴.

> **[VDSim impl] § 2.9 — 검증 test**
>
> | 검증 | test |
> |---|---|
> | linear bicycle steady-state yaw rate vs 해석값 | `tests/integration/test_bicycle_steady_state.cpp` 의 `LeftTurnYawRateMatchesAnalytical` |
> | body EoM 의 부호 / 차원 | 동 파일의 step-steer sweep |
