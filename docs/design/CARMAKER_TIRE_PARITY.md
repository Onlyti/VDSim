# CarMaker MF-Tyre/MF-Swift — 3-way tire parity (VDSim · Chrono · CarMaker)

상태: **feasibility 확인 완료, init 레시피 디버깅 진행 중** (2026-06-16)
목표: 동일 `.tir`을 세 solver가 같은 (Fz, κ, α) grid에서 평가 → Fx/Fy/Mz 3-way 비교.
배경: `docs/IMPROVEMENT_REPORT.md` P3 (타이어 신뢰성 — "VDSim vs Chrono vs CarMaker").

---

## 현재 상태

- VDSim ↔ Chrono 2-way: **완료** (`ctest -R ChronoPac02Parity`, pure-slip ~0.8%).
- CarMaker 축: **인프라 조사 완료, 미완성**.

---

## Feasibility — 확인된 사실 (de-risked)

ailab-12에 CarMaker 12.0.1 설치됨 (`/opt/ipg/carmaker/linux64-12.0.1`),
floating license (IPGLOCK 166.104.167.98).

표준 MF-Tyre/MF-Swift API가 standalone 공유 라이브러리로 제공:
- 헤더: `include/mfs_tire_api.h`, `include/MFS_API.h`
- 라이브러리: `lib/libmfswift_tire_interface.so` (버전 2212 / API 1.0.13)

probe (`external/carmaker_parity/mfs_init_probe.c`) 로 확인:
- ✅ 라이브러리 로드 + 버전 조회 OK
- ✅ `mfs_api_simulation_create` / `mfs_api_tire_create` OK
- ✅ **entitlement(라이선스) 블로커 없음** — `tire_property_file` 로드 성공
- ✅ 섹션 파싱 OK ([UNITS]/[MODEL]/[DIMENSION]/...)
- ❌ `mfs_api_tire_init` 실패: `ERROR - The FITTYP parameter was not read, but is required [parser_validate_read_entries]`

**중요**: 이 에러는 우리 합성 `.tir` 뿐 아니라 CarMaker 자체 샘플
(`Data/Tire/Examples/.../MF_205_60R15_V91.tir`) 으로도 동일하게 발생.
→ `.tir` 파일 문제가 아니라 **init 호출 시퀀스(레시피) 가 불완전**하다는 뜻.

시도했으나 해결 안 됨:
- `[MDI_HEADER]`(FILE_TYPE/VERSION/FORMAT) 추가 — 무해하나 미해결
- `PROPERTY_FILE_FORMAT='MF_05'` + `USE_MODE` 추가 — 미해결
- `initialize_simulation_mode`(레거시 ISWITCH) 제거 — 미해결

---

## 다음 단계 (init 레시피 찾기)

1. **`doc/MFTyreMFSwift_UserManual.pdf` §2.6 (ISWITCH/USE_MODE) 정독** —
   `mfs_api_tire_init` 전 필수 호출 순서/조합 확인.
2. MFS API 사용 예제 탐색 (헤더가 `mfs_api_example_external_road.c` 언급하나
   설치본에 없음 — Siemens 배포본/CarMaker 추가 패키지에 있을 수 있음).
3. 후보 원인:
   - `magic_formula_mode` 설정과 파일 FITTYP 불일치 시 validation 강제
   - init 전 `solver_mode`(internal/external) 또는 inflation/operating-condition
     기본값 설정 누락
   - 파서가 특정 섹션(예: `[OPERATING_CONDITIONS]`, `[INFLATION_PRESSURE_RANGE]`)
     을 요구

---

## 평가 harness 설계 (init 통과 후)

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
