# CarMaker MF-Tyre/MF-Swift — tire parity (VDSim · Chrono · CarMaker)

상태: **VDSim ↔ CarMaker 직접 비교 동작** (2026-06-16)
목표: 동일 `.tir`을 solver들이 같은 (Fz, κ, α) grid에서 평가 → Fx/Fy/Mz 비교.
배경: `docs/IMPROVEMENT_REPORT.md` P3 (타이어 신뢰성 — "VDSim vs Chrono vs CarMaker").

---

## 결과 (Siemens MF6.2 샘플 .tir, 같은 파일을 VDSim+CarMaker가 읽음)

**동일 슬립 조건에서 기계 정밀도 일치:**

| 구분 | n | mean | max |
|------|---|------|-----|
| Pure-long Fx | 57 | **0.00%** | 0.00% |
| Pure-lat Fy | 8 | **0.09%** | 0.20% |

핵심 (슬립 정의 정렬):
- CarMaker 의 종슬립 κ = (ω·Re − vx)/vx 는 **effective rolling radius Re** (하중 시
  unloaded R 보다 작음) 기준. harness 가 처음엔 unloaded R 로 ω 를 설정 → 저슬립에서
  실제 κ 가 의도값과 크게 달라 종방향 오차가 ~9% 로 부풀려졌음 (횡방향 α 는 반지름
  무관이라 영향 없었음).
- 해결: harness 가 CarMaker 가 **실제로 계산한** 슬립(varinf 6/7)을 출력 → VDSim 에
  그 동일 슬립을 입력 → **두 solver 가 정상상태 순수슬립에서 0% 일치**.
- 즉 VDSim 의 Magic Formula 타이어 == CarMaker MF-Swift (정상상태, 순수슬립), 같은
  계수에서. (손계산 MF6.2 도 VDSim 과 정확히 일치 확인.)

재현:
```bash
cd external/carmaker_parity
CMI=/opt/ipg/carmaker/linux64-12.0.1 ./build.sh
python3 compare_vdsim_carmaker.py \
  /opt/ipg/carmaker/linux64-12.0.1/Data/Tire/Examples/TirePropertyFile/Siemens_car205_60R15.tir
```

상태:
- VDSim ↔ Chrono 2-way: 완료 (`ctest -R ChronoPac02Parity`, pure-slip ~0.8%, 합성 .tir).
- VDSim ↔ CarMaker 2-way: **동작** (위 결과, Siemens MF6.2 .tir).
- 완전한 3-way(동일 .tir): Chrono ref 가 합성 FITTYP=6 파일 기준이라 미정렬 — 아래 참고.

---

## Feasibility — 확인된 사실 (de-risked)

ailab-12 CarMaker 12.0.1 (`/opt/ipg/carmaker/linux64-12.0.1`), floating license.
표준 MF-Tyre/MF-Swift API: `include/mfs_tire_api.h` + `lib/libmfswift_tire_interface.so`
(2212 / API 1.0.13).

- ✅ 라이브러리 로드 + entitlement 블로커 없음 (floating license 로 동작)
- ✅ init 시퀀스 (road=FLAT, contact=SMOOTH, dynamics=STEADY_STATE,
  mf=COMBINED_LOADS, side=LEFT → `tire_init`) — **MF6.x (.tir FITTYP≥61) 파일에서 성공**
- ❌ MF-Swift 2212 는 **MF5.2 (FITTYP=6) 파일을 거부** (`FITTYP not read`). 우리 합성
  `sample_pac02.tir` (FITTYP=6) 및 CarMaker 자체 MF5.2 샘플 모두 거부.
  → 신형 MF6.x 파일 (예: `Siemens_car205_60R15.tir`, FITTYP=70) 필요.

핵심: harness 자체는 정확. legacy MF5.2 파일만 안 됨 → MF6.x 파일로 평가.

---

## 완전한 3-way 를 위한 다음 단계

현재 Chrono reference CSV 는 합성 FITTYP=6 파일에서 생성됨 → MF-Swift 가 그 파일을
못 읽어 동일-입력 3-way 가 안 됨. 정렬 방법:

1. **공용 MF6.x `.tir` 채택** (FITTYP≥61). 단, CarMaker 동봉 Siemens 파일은 TNO
   copyright 이므로 commit 불가 → 공개 TNO/Delft MF6.1 파일 확보 필요.
2. 그 파일로 Chrono reference 재생성 (Chrono 빌드) + VDSim parity 재확인.
3. `tools/tire_validation.py` 에 `--ref2` 추가 → Chrono + CarMaker 동시 비교 테이블.

당장은 VDSim↔CarMaker 2-way (위) 가 실재 CarMaker 검증 evidence 로 충분.

---

## 평가 harness 설계 (구현 완료)

MFS 입력은 직접 (κ, α, Fz) 가 아니라 **wheel-carrier 기구학** (`MFS_INPUT_DATA`:
위치·속도·변환행렬·wheel angle/ω). 따라서 Chrono grid의 각 (Fz, κ, α) 점을
재현하려면:

```
Fz   ← 수직 침투량(노면 z) 조정 → Fz = f(deflection) 역산 (1D solve per point)
κ    ← wheel ω 설정: ω = (κ·vx + vx)/R  (vx 고정)
α    ← lateral velocity 설정: vy = vx·tan(α)
출력 ← mfs_api_tire_get_output → force[3], torque[3] (Fx, Fy, Mz)
```

절차:
1. `external/chrono_parity/reference/pac02_reference.csv` 의 (Fz, κ, α) 읽기
2. 각 점에서 위 기구학으로 `MFS_INPUT_DATA` 구성 → `set_input` → `update` → `get_output`
3. `pac02_carmaker.csv` (Fz, κ, α, Fx, Fy, Mz) 출력
4. `tools/tire_validation.py` 에 `--ref2` 추가 → 3-way 오차 테이블

dynamics mode = STEADY_STATE, MF mode = COMBINED_LOADS, contact = SMOOTH_ROAD,
road = DEFAULT_FLAT 로 steady-state MF만 평가 (rigid-ring 동특성 제외).

---

## 산출물 (완성 시)

```
external/carmaker_parity/
  mfs_init_probe.c          init feasibility probe (현재 단계)
  mfs_grid_eval.c           grid → pac02_carmaker.csv (TODO: init 레시피 후)
  build.sh                  gcc -I include -L lib -lmfswift_tire_interface
  reference/pac02_carmaker.csv   (생성물)
tools/tire_validation.py    --ref2 로 3-way 비교
docs/VALIDATION.md          benchmark #13 을 3-way 로 확장
```

---

## 빌드/실행 메모

```bash
CMI=/opt/ipg/carmaker/linux64-12.0.1
gcc -I $CMI/include mfs_init_probe.c \
    -L $CMI/lib -lmfswift_tire_interface -Wl,-rpath,$CMI/lib -o mfs_init_probe
./mfs_init_probe   # 현재: init 0 (FITTYP validation) — 레시피 디버깅 필요
```

라이선스: CarMaker floating license 가 동작 중이면 별도 entitlement 파일 불필요
(probe 에서 확인). license 서버 다운 시 라이브러리 호출이 막힐 수 있음.
