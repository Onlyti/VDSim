# VDSim Theory Docs — Refactor handoff

**작성 시점**: 2026-06-02
**작성 주체**: scc_controller_imitator workspace 의 다른 agent 가 시범 작업 후 인수인계
**대상**: VDSim repo 의 docs/theory 후속 작업 agent
**상태**: chapter 01, 03 의 시범 재구성 완료. 02 / 04 – 16 + README 의 일관 적용 필요.

---

## 1. 사용자가 제기한 문제

> "글 읽어보고 있는데 흐름이 체계적으로 다듬어지지 않고 개발 흐름에 따라 쓴
> 블로그 같은 느낌이야. 좀 더 전문적으로 교육용 자료가 되었으면 좋겠어."

진단된 *블로그-감* 의 4 가지 sign:

1. **개발 history 가 본문에 박혀 있음** — *"Task 17 의 brake step"*, *"Task 20 의
   ay 정의 버그"*, *"Task 51 의 분석"* 등 chapter 본문이 PoC 개발 sprint 의
   timeline 을 참조. 교육용은 *왜 / 식 / 직관 / 검증 / 한계* 의 학습 순서가
   우선, 개발 history 는 별도 *implementation note* 박스나 부록으로 분리.

2. **Self-referential 일인칭** — *"본 PoC 의 default"*, *"본 PoC 채택"*, *"VDSim
   의 unique claim"* 빈번. 교육용은 *"본 시리즈는 X 를 채택한다"* 의 3 인칭
   reference 양식.

3. **§X.10 사용 패턴 (code) 양식** — 각 chapter 끝에 *"code 사용 패턴"* 절이
   블로그의 *"마지막에 실행 결과"* 양식. 교육용은 chapter 끝의 *worked example*
   1 개 또는 본문 직후 *example* 양식, 또는 별도 박스.

4. **Chapter 간 learning path 가 명시 안 됨** — *§X.Y 다음 챕터 연결* 같은 절이
   일부 chapter 에만 있음 (01, 04 에는 있고 05, 06, 12 에는 없음). prerequisite +
   다음 chapter 가 모든 chapter 의 머리 / 꼬리에 일관 위치해야 함.

---

## 2. 사용자가 확정한 청중 / 구조

질문 응답:

| 항목 | 사용자 답 |
|---|---|
| 청중 | **Hybrid — 교과서 본문 + 사용자 매뉴얼 분리**. 각 chapter 본문 = 교과서, 별도 박스 = VDSim 구현 노트. 둘 다 명시 분리. |
| 시범 chapter 1 | **README** (전체 시리즈 입구) |
| 시범 chapter 2 | **03 Pacejka MF96** (길고 수식 위주 — 포맷 견고함 검증) |

따라서 적용 대상 청중:

- 차량 동역학 / 자율주행 제어를 처음 / 두 번째 배우는 PhD/MS 학습자
- VDSim 의 API / 채택 / 한계를 파악하려는 산업 / 연구 사용자

본문만 따라가도 self-contained 한 교과서. 박스만 추적해도 VDSim 사용자 매뉴얼.

---

## 3. 확정된 표준 sequence (모든 chapter 에 적용)

```
# NN. Chapter Title (English)

## Learning objectives
이 chapter 를 마치면 다음을 할 수 있다.
1. ... (3-5 항목, "할 수 있다" 동사 형식)

## Prerequisites
- Chapter XX (관련 chapter)
- 외부 reference (책 / paper 의 § 단위)

---

## N.1 동기
왜 이 모델 / 알고리즘이 필요한가. 자기-광고 톤 X.

## N.2 가정
| 가정 | 의미 | 깨지는 case |
|---|---|---|
(명시 list, 깨지는 case 와 다루는 chapter 매핑)

---

## N.3 ~ N.k 본문 (유도, 직관, 부호)
모델 자체. 코드 / Task / config 값 / 파일명 inline 금지 — 모두 박스로.

## N.{k+1} 검증 전략
표 형식. 식 / 케이스 단위. test 파일명은 박스로 분리.

## N.{k+2} 한계
| 항목 | 본 모델 | 한계 |
|---|---|---|

## N.{k+3} 다음 chapter 와의 연결
1 단락. 본 chapter 결과가 어느 chapter 에서 어떻게 쓰이나.

## N.{k+4} 참고문헌
README 의 핵심 reference 우선. § 단위로 인용.

---

## N.{k+5} Self-check
<details><summary>Question</summary>
Answer (모델 / 식 / 부호 추적).
</details>

(3-5 문제. <details> 로 답 hidden — GitHub 렌더링 친화)

---

## N.{k+6} VDSim 구현 노트
> **[VDSim impl] § N.X — title**
>
> 코드 / 파일경로 / Task / config 의 박스.
```

### 박스 marker 양식

```
> **[VDSim impl] § N.X — short title**
>
> (코드 또는 표 또는 cross-ref)
```

### `<details>` self-check 양식

```markdown
<details>
<summary>1. 질문문 (간결)</summary>

답변 — 식 / 부호 추적 / 직관. 한 단락.
</details>
```

답 hidden → 학습자가 시도 후 펼쳐 확인. GitHub 렌더링 + Markdown 뷰어 호환.

---

## 4. 시범 chapter 의 before / after

### Chapter 01 — Frames and Sign Conventions

| section | before | after |
|---|---|---|
| 머리 | "학습 목표" 1 문단 blockquote | Learning objectives (4 항목 list) + Prerequisites |
| §1.1 동기 | 비교 표 + `WHEEL_FL = 0` 코드 inline | 비교 표 + 본문만 (코드 박스로) |
| §1.2 (신규) | — | 가정 표 (Flat ground / contact normal / small roll-pitch / rigid sprung) |
| §1.3 네 가지 frame | 4 frame 설명 | 그대로, figure 위치만 조정 |
| §1.4 quaternion | 식 + `yaw_from_quat` 코드 + Euler 함정 | 식 + Tait-Bryan + atan2 추출 (코드 박스로) |
| §1.5 slip 정의 | alpha + kappa + 코드 inline | 식 + 부호 직관 + κ 부호 표 (코드 박스로) |
| §1.6 yaw integration | planar + quaternion ODE | + "yaw 적분의 흔한 실수" 흡수 |
| §1.7 (신규) | — | 검증 전략 표 |
| §1.8 (신규) | — | 한계 표 + 다루는 chapter 매핑 |
| §1.9 다음 chapter | 1 단락 | 그대로 |
| §1.10 참고문헌 | 3 책 | 4 (Sola quaternion paper 추가) |
| §1.11 (신규) | — | Self-check 5 문제 (`<details>`) |
| §1.12 (신규) | — | VDSim 구현 노트 6 박스 |

### Chapter 03 — Pacejka MF96

| section | before | after |
|---|---|---|
| 머리 | 학습 목표 1 줄 blockquote | Learning objectives (5) + Prerequisites |
| §3.2 (신규) | — | 가정 표 (quasi-static, friction ellipse, constant μ, camber input, Mz 모델, load sensitivity linear) |
| 본문 | "VDSim 채택" / 코드 inline / Task 23 결과 inline | 본문 = 모델만. 코드 / Task / config → §3.15 박스 |
| §3.7 γ source | "이전 PoC 에서는 …" / "Ld4 가 들어오면서 바뀜" | `γ` source by model 표 (Ld1/Ld2/Ld3/Ld4) |
| §3.9 relaxation | "이전 PoC 는 quasi-static" | 객관적 "quasi-static 모델은…" + closed-form substep |
| §3.10 검증 전략 (신규) | — | 14 항목 표 |
| §3.11 한계 | 표 그대로 | 그대로 |
| §3.12 (신규) | — | 다음 chapter 연결 |
| §3.14 (신규) | — | Self-check 5 문제 (`<details>`) |
| §3.15 (신규) | — | VDSim 구현 노트 7 박스 |

### README — 시리즈 입구

| section | before | after |
|---|---|---|
| 청중 | "PhD 수준 독자" 한 갈래 | 학습자 + VDSim 사용자 명시 |
| Hybrid 구조 | 없음 | "본문 (교과서)" + "VDSim 구현 노트 박스" 두 trail 명시 + 박스 marker 양식 |
| chapter 표 | 01-13 (14/15/16 누락) | 16 chapter + 한 줄 learning objective |
| reading path | 1 줄 권고 | Path A (textbook) / B (control) / C (sw) / D (multibody) 4 갈래 |
| 부호 약속 | 산문 list | VDSim ↔ Rajamani 비교 표 |
| 약자 표 | TUR/NHCalib/FSK/UKF/HPIPM/SMPC/ERG | 시리즈 무관 약자 drop. MF96/DOF/EoM/ABI/K&C 만 |
| chapter 안 sequence | 미명시 | 표준 sequence 명시 |
| 끝맺음 | "PT / 세일즈 시 인용 가능" | 시리즈 상태 표 + follow-up |

---

## 5. 남은 chapter 14 개의 적용 순서 + 권장 priority

각 chapter 의 길이 / 수식 비중 / 코드 비중을 보고 priority 결정.

### Path 별 그룹화

| Path | chapter | 권장 적용 순서 | 주의 |
|---|---|---|---|
| A — textbook | 02 Rigid-body, 04 Ld1, 05 Ld2, 06 Ld3 | 02 → 04 → 05 → 06 | 04 가 사용자 자주 보는 chapter |
| B — control | 07 Ladder, 08 PID, 09 Pure Pursuit, 10 Driver | 07 → 08 → 09 → 10 | 07 은 짧음 — 빠른 적용 가능 |
| C — sw / numeric | 11 RK4, 12 SW Architecture | 11 → 12 | 12 는 박스 비중 극단적으로 큼 |
| D — multibody / 산업 | 13 Multibody, 14 Hardpoint, 15 Validation, 16 FMI | 13 → 14 → 15 → 16 | 13 은 *roadmap* 성격 — 적용 형식 조정 가능 |

### 권장 시작점

- **chapter 02** — Path A 의 시작. 차량 동역학 textbook 의 *왜 body frame EoM
  인가* 가 핵심. chapter 01 의 figure / 표 패턴 그대로 적용 가능.
- **chapter 04 (Ld1)** — 사용자 자주 보는 chapter. 시범 chapter 01, 03 보다 본문이
  *모델 + analytical solution* 의 비중 큼. 표준 sequence 검증 좋음.
- **chapter 11 (RK4)** — cross-ref 많은 핵심. 다른 chapter 의 *§N.X 의 1-step
  lag* 같은 참조가 본 chapter 와 어긋날 수 있음. 일관 적용 후 cross-ref grep.

---

## 6. 일관 적용 시 검증 항목 (cross-link sanity)

§ 번호가 chapter 마다 추가 절 (가정 / 검증 전략 / 한계 / 다음 chapter / Self-check
/ VDSim 노트) 로 변경되므로 cross-link 깨질 위험.

### 검증 step

1. **§ 번호 cross-ref grep**:
   ```bash
   cd docs/theory
   grep -rn '§[0-9]' . | grep -v "_REFACTOR_HANDOFF"
   ```
   → 각 cross-ref 가 새 § 번호와 일치하는지 확인.

2. **Chapter cross-ref grep**:
   ```bash
   grep -rn 'chapter [0-9]\|Chapter [0-9]\|Ch\.\s*[0-9]' . | grep -v "_REFACTOR_HANDOFF"
   ```
   → 인용된 chapter 가 실제 내용과 일치.

3. **README 의 chapter index ↔ 각 chapter Learning objectives 일치**:
   ```bash
   # README 의 한 줄 objective 와 chapter NN 의 Learning objectives 표
   # 가 일관된 메시지인지 manual check.
   ```

4. **figure 경로 검증**:
   ```bash
   grep -rn 'figures/' . | grep -v "_REFACTOR_HANDOFF"
   # 모든 figure 가 실존하는지 ls 로 확인.
   ```

5. **박스 marker 일관성**:
   ```bash
   grep -rn '\[VDSim impl\]' .
   # 모든 박스가 `> **[VDSim impl] § N.X — title**` 양식인지.
   ```

---

## 7. 시범 단계에서 명시적으로 적용한 결정 / 양식

### 7.1 항상 영문 chapter title

Korean 본문 + 영문 title. 예: `# 03. Tire Model — Pacejka Magic Formula 1996`.
이전 chapter title 의 한국어 part 는 본문 첫 단락에서 자연 사용.

### 7.2 가정 표의 3 column

| 가정 | 의미 | 깨지는 case (chapter 번호 명시) |

특히 *깨지는 case* 칸이 한계 → 다른 chapter 매핑의 single source.

### 7.3 검증 전략 표

본문에 *검증 방법* 자체 (식 / 케이스). test 파일명은 박스로 분리.

```markdown
| 검증 | 식 / 케이스 |
|---|---|
| Linear-region slope | `α = 1e-4` 에서 `Fy/α ≈ −B·C·D·Fz·μ` (±1 %) |
```

### 7.4 박스 표제 양식

```markdown
> **[VDSim impl] § N.X — Default coefficient**
>
> ```yaml
> ...
> ```
>
> (추가 설명 한 단락)
```

`§ N.X — title` 가 본문의 어느 절을 보조하는지 명시.

### 7.5 Self-check 의 답 양식

```markdown
<details>
<summary>1. 질문문 (간결, ?로 끝)</summary>

답변 — 한 단락 또는 짧은 list. 모델 / 식 / 부호 추적.
</details>
```

답을 list 로 쪼개도 OK. 본문에서 학습한 식을 직접 사용해 푸는 *applied*
문제 위주. *암기* 문제 (예: "B 의 단위는?") 는 피한다.

### 7.6 한 줄 메시지 / 박스 분리 원칙

| 본문 (교과서) 에 들어가는 것 | VDSim 구현 노트 박스로 분리 |
|---|---|
| 모델의 식 + 유도 + 직관 | 코드 (`.cpp`, `.hpp`) |
| 부호 약속의 책별 비교 | 파일 경로 / line 번호 |
| 가정 표 | config yaml 의 default 값 |
| 검증 전략 (식 단위) | test 파일명 + 케이스명 |
| 한계 표 | Task 번호 / 개발 history |
| 책 / paper 의 참고문헌 | VDSim 자체 docs cross-ref (예: `상세: chapter 14 §14.8`) |

### 7.7 figure caption 양식

본문에서 figure 를 인용할 때 *Figure 가 무엇을 보여주는가* 1 단락 명시.

```markdown
![Pacejka Fy-alpha + friction ellipse](figures/03_tire.png)

위 그림은 ... load sensitivity ... friction ellipse 의 boundary saturation
이 함께 보인다.
```

---

## 8. 결정 미정 / VDSim agent 가 결정해야 하는 항목

1. **§ 번호 변경 시 다른 chapter 의 cross-ref 자동 업데이트** — 일괄 sed 가능?
   아니면 chapter 마다 manual?
2. **figure 추가 필요한 chapter** — 시범에서 새 figure 발견 X. 본 작업 중
   chapter 02 / 11 등에서 학습 보조 figure 필요 여부 검토.
3. **chapter 13 Multibody Outlook** — roadmap 성격 (M0-M7) 이라 표준 sequence
   적용 시 *가정* / *검증 전략* 등의 절이 어색할 수 있음. roadmap chapter 의
   별도 sequence 정의 또는 표준 sequence 의 일부 절 omit.
4. **chapter 12 SW Architecture** — 박스 (구현 노트) 가 본문 (교과서) 보다 큼.
   "본문 = 일반 SW pattern (factory, variant, FMI), 박스 = VDSim 의 구체 코드"
   분리가 가능한지.
5. **English title 의 chapter README 표 일관성** — 시범에서 영문 title 채택했으나
   README 의 chapter index 표는 한국어 + 영문 mix. 일관 통일 또는 mix 유지 결정.

---

## 9. 참조 — 시범에서 적용된 README 의 표준 sequence

```
0. Learning objectives        (이 chapter 를 마치면 ... 할 수 있다)
0. Prerequisites              (이전 chapter / 외부 reference)
1. 동기                       (왜 이 모델 / 알고리즘 인가)
2. 가정                       (명시적 list)
3. 유도                       (식 + 직관 + 단위 + 부호)
4. 검증 전략                  (식 / 케이스, test 명은 박스)
5. 한계                       (어떤 현상 / 어느 chapter 가 보완)
6. 다음 chapter 와의 연결     (1 단락)
7. 참고문헌                   (README 의 핵심 ref 우선)
8. Self-check                 (3-5 문제, `<details>` hidden 답)
9. VDSim 구현 노트            (박스 list, 본문 § 별 cross-ref)
```

---

## 10. Audit checklist — 작업 완료 후

각 chapter 단위 작업 후 다음 항목 통과 확인:

- [ ] Learning objectives 가 3-5 항목, 동사 *할 수 있다* 형식
- [ ] Prerequisites 가 chapter 번호 + 외부 reference 두 갈래
- [ ] 본문 absolute 없음 — "본 PoC", "VDSim 채택", "이전 PoC 는" 표현 grep 결과 0
- [ ] 본문에 코드 / 파일 경로 / Task 번호 / config 값 / unit test 명 없음
- [ ] 가정 표 의 *깨지는 case* 가 다른 chapter 와 cross-ref 명시
- [ ] 검증 전략이 *식 / 케이스* 형식. test 명은 박스에만
- [ ] 한계 표 의 항목이 본문에서 다 언급된 항목인지
- [ ] 다음 chapter 와의 연결이 1 단락
- [ ] Self-check 3-5 문제, applied 위주, `<details>` 양식
- [ ] VDSim 박스가 본문 § cross-ref 명시 (`§ N.X — title`)
- [ ] figure 가 inline caption 으로 무엇을 보여주는지 명시
- [ ] cross-link grep 통과 (§6 의 5 step)

---

## 11. 사용자 / 작업자 추가 컨텍스트

### 사용자 (Jiwon, Hanyang AILab)

- **PhD 마지막 학년 (2026 H2 졸업 예정)**, self-driving localization & control
- VDSim 은 졸업 후 **창업 시뮬레이션 제품** 으로 발전 예정 (open-core
  positioning: Adams Car × MORAI × CarSim intersection)
- 졸업 thesis 의 main message: *"slip-tolerant path tracking"* — UKF Fz/μ/Cα
  추정 → covariance P → SMPC chance constraint (HPIPM)
- VDSim docs 의 청중은 학계 (강의용) + 산업 (open-core 사용자) 둘 다

### 사용자의 작업 스타일 / 톤

- **Cold, realistic assessment** — praise / hedging / encouragement 표현 X
- **Korean prose + English technical terms** preferred. 예: "UKF 의 covariance
  를 chance constraint 로 넘기는 구조"
- **No emoji, minimal bold** (title / table header 만)
- **Iterative scope refinement** — 한 번에 전체 안 하고 시범 → review → 일관
  적용. 본 handoff 도 그 흐름.

### 환경

- Repo path: `~/git/VDSim`
- Theory docs: `docs/theory/`
- 시범 작업 완료된 파일: `README.md`, `01_frames_and_conventions.md`,
  `03_tire_pacejka_mf96.md`
- AILAB-12 = `10.0.0.20` (Ubuntu 20.04 LTS)

---

## 12. 한 줄 인수인계

> 이 작업의 핵심은 *"교과서 본문 self-contained"* + *"구현 노트 박스 self-contained"*
> 두 trail 의 cleanly 분리. 시범 chapter 01, 03 + README 가 양식 reference.
> 02 / 04-16 의 14 개 chapter 에 §3 의 표준 sequence + §7 의 양식 + §10 의
> audit checklist 를 적용. cross-link 검증은 §6 의 5 step.
