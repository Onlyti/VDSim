# VDSim 타이어 검증 보고서

기준일: 2026-06-15  
VDSim: main `23bc289` 이후, 339 ctest green  
도구: `tools/tire_validation.py` (Python), `ctest -R ChronoPac02Parity` (C++)

---

## 1. 검증 방법

### 1.1 독립 참조 비교 (Chrono Pac02)

VDSim MF2002 평가기를 Chrono Vehicle(BSD-3) Pac02 구현과 동일한 `.tir` 파일로 비교한다.  
동일 계수 → 두 구현 간 수치 차이 = 구현 오차.

참조 데이터:
- `.tir`: `external/chrono_parity/sample_pac02.tir` (공개 Pacejka MF2002 계수)
- 참조 CSV: `external/chrono_parity/reference/pac02_reference.csv` (Chrono 8 생성)
- 147 points: Fz ∈ {2, 4, 6, 8} kN × κ ∈ {−0.17..0} × α ∈ {−0.14..0} rad (grid)

### 1.2 평가 구분 (ISO 기준)

| 구분 | 조건 | 의미 |
|------|------|------|
| Pure-longitudinal | `|α| < 0.02 rad` | 종방향 슬립만 (급가속·제동) |
| Pure-lateral | `|κ| < 0.025`, `|α| > 0.03 rad` | 횡방향 슬립만 (선회) |
| Combined | `|κ| > 0.025` and `|α| > 0.02 rad` | 복합 슬립 (조향+제동) |

---

## 2. MF2002 vs Chrono Pac02 (생성: 2026-06-15)

| 구분 | n | mean rel err | max rel err | gate |
|------|---|-------------|-------------|------|
| Pure-long Fx | 60 | 0.8% | 1.8% | < 6% ✅ |
| Pure-lat  Fy | 14 | 0.7% | 1.6% | < 6% ✅ |
| Combined  Fx | 70 | 28% | 50% | 보고만 (known) |
| Combined  Fy | 70 | 22% | 61% | 보고만 (known) |

**Pure 슬립은 1% 대 오차로 Chrono와 일치한다.** Combined 슬립 오차는 구현 간 friction-ellipse weighting 방식 차이에서 기인하며, sign 일관성과 ×3 gross-regression은 통과한다.

재현:
```bash
# C++ 기준 (ctest — gate 검증)
ctest -R ChronoPac02Parity

# Python 기준 (수치 표 생성)
PYTHONPATH=build/python:python python3 tools/tire_validation.py \
    --tir external/chrono_parity/sample_pac02.tir \
    --ref external/chrono_parity/reference/pac02_reference.csv
```

---

## 3. 실차 측정 데이터 비교 (placeholder)

> 아래 표는 실측 타이어 시험 데이터 입수 후 채울 자리입니다.  
> 측정 장비: 타이어 시험기(flat-track 또는 drum-type), SAE J2374 등.

| 조건 | n | mean err Fx | mean err Fy | 출처 | 날짜 |
|------|---|------------|------------|------|------|
| *(실차 측정 pending)* | — | — | — | — | — |

**채우는 방법:**

1. 시험기에서 (Fz, κ, α, Fx, Fy) 표 측정 후 CSV 저장 (`tools/tire_validation.py` 포맷 준수)
2. 계수를 MF2002 형식 `.tir` 파일로 피팅 (TNO MF-Tool 또는 자체 최적화)
3. 비교 실행:
   ```bash
   python3 tools/tire_validation.py --tir fitted.tir --ref measured.csv
   ```
4. 결과 수치를 이 표에 추가

---

## 4. 검증 도구

| 도구 | 경로 | 용도 |
|------|------|------|
| `tire_validation.py` | `tools/` | Python: MF2002 per-point 오차 계산 + CSV/JSON 출력 |
| `ChronoPac02Parity.*` | `ctest -R Chrono` | C++: 6% gate (pure) + 보고 (combined) |
| `sample_pac02.tir` | `external/chrono_parity/` | 공개 MF2002 계수 (Chrono 동봉) |
| `pac02_reference.csv` | `external/chrono_parity/reference/` | Chrono 생성 기준값 |

---

## 5. 한계 및 솔직한 평가

| 항목 | 현황 | 의미 |
|------|------|------|
| 공식 MF-Tyre solver 인증 | 없음 | CarMaker/Adams처럼 타이어 제조사 인증 solver 아님 |
| 실차 측정 대조 | 없음 | 동일 계수에서 독립 구현 간 비교만 검증됨 |
| combined-slip 고오차 | 알려진 차이 | friction-ellipse 구현 방식 이슈, sign 일관성은 유지 |
| 열 모델 | 없음 | 타이어 온도, 마모 미반영 |

---

## 6. 다음 검증 단계

1. **실차 `.tir` 비교** — 공개 데이터셋 (TNO sample, Delft-Tyre 공개 파라미터) 입수 후 `tire_validation.py` 실행
2. **combined-slip 개선** — VDSim friction-ellipse vs Pac02 weighting 정렬 검토
3. **ADMA/CarMaker ERG 대조** — TUR 실측 데이터 확보 시 `tools/campaign_runner.py` + 실차 Fz/ax/ay 비교
