# VDSim 개선안 보고서 — CarMaker 대비 격차 분석 및 우선순위

작성: 2026-06-15  
기준: VDSim v0.5.2 (main `faf3010`), 339 ctest green

---

## 1. 전제 및 범위

ADAS 시나리오·교통 시뮬레이션은 VDSim의 타깃이 아니므로 제외한다. 본 보고서는 다음 두 축에 집중한다.

- **연동성**: 제어 개발 생태계(Simulink, Python, UDP/HIL)와의 연결
- **물리 깊이**: 타이어·브레이크·스티어링·서스펜션·구동계

포지셔닝 전제: VDSim은 "알고리즘 개발용 정확한 차량 sandbox"이다. CarMaker와 동일한 완성도를 목표로 하지 않는다 — 차별점은 open-core, Python-first, 빠른 배치 실험.

---

## 2. 현황 진단

### 2.1 연동성

| 연동 경로 | 현황 | 비고 |
|-----------|------|------|
| FMI 2.0 export | ✅ 구현 (L1/L3), round-trip Δ=0 검증 | Simulink FMU Block으로 직접 사용 가능 |
| Python API (`vdsim_lab.Sim`) | ✅ 완성 | set_input / run_core_dt / state / measurements |
| UDP VDS1 (`vdsim_realtime`) | ✅ 200 Hz, JSON도 추가됨 | HIL/SIL 경계 |
| Simulink S-Function (직접) | 🔴 없음 | FMU 경유로 가능하나 workflow 불편 |
| ROS/ROS2 bridge | 🔴 없음 | 자율주행 스택 연결 필요 시 |
| dSPACE / NI HIL | 🔴 없음 | UDP로 대체 가능하나 인증 없음 |
| MATLAB Engine API | 🔴 없음 | 데이터 교환용 |

**핵심 진단**: FMI 2.0이 이미 있으므로 Simulink 연동은 기술적으로 열려 있다. 문제는 workflow — FMU 빌드 → Simulink import 절차가 문서화되지 않았고, FMI 3.0 지원이 없어 최신 Simulink 버전에서 마찰이 있다.

### 2.2 물리 깊이

| 서브시스템 | CarMaker 수준 | VDSim 현황 | 격차 크기 |
|------------|--------------|------------|-----------|
| 타이어 | 공식 MF-Tyre solver (인증) | 자체 MF96+MF2002 구현 | 중 — 계수 동일하면 결과 유사하나 공신력 없음 |
| ABS/ESC | 내장 hydraulic 모델 | 인터페이스만 | 중 — 알고리즘 개발용으론 interface 충분 |
| 스티어링 | MDPS/EPS + column compliance | ratio + deadtime | 중 |
| 브레이크 | line 압력 → pad μ → fade | proportional + EBD | 소-중 |
| 서스펜션 | bushing compliance, progressive | linear + Ld4 kinematics | 중 |
| 차동기어 | Torsen/visco LSD, e-LSD | open diff뿐 | 소-중 |
| 구동계 | 엔진 열역학, 클러치 slip | torque map + gearbox v2 | 중 |

---

## 3. 우선순위별 개선안

### P1 — Simulink/FMI workflow (1–2개월)

**문제**: FMI 2.0 export는 있으나 Simulink에서 쓰는 절차가 불명확하고 테스트가 없다.

**개선안**:

1. `docs/SIMULINK_GUIDE.md` 작성 — FMU 빌드 → Simulink `FMU Block` 설정 → 데이터 교환 예제
2. `examples/simulink/` — MATLAB script + Simulink 모델 파일(`.slx`) 추가
3. FMI 2.0 round-trip ctest를 Simulink 없이도 검증 가능한 Python co-sim 예제로 확장
4. (선택) FMI 3.0 export — Simulink 2022b 이상에서 권장; 구조는 FMI 2.0과 유사

**기대효과**: 제어 개발팀이 Simulink 컨트롤러를 VDSim plant에 직접 붙일 수 있다 — 가장 큰 진입 장벽 해소.

---

### P2 — Python ↔ Simulink 데이터 교환 (1개월)

**문제**: Simulink에서 Python 함수를 호출하거나, Python에서 Simulink 결과를 읽는 표준 경로가 없다.

**개선안**:

1. `tools/matlab_bridge.py` — MATLAB Engine API(`matlab.engine`)로 workspace 변수 교환
2. `tools/mat_export.py` — `to_csv` 결과를 `.mat` 파일로 변환 (scipy.io.savemat)
3. UDP 경로 예제 — `vdsim_realtime` ↔ Simulink UDP Receive/Send 블록 연결 예제

---

### P3 — 타이어 신뢰성 강화 (1–2개월)

**문제**: 자체 구현 MF solver — CarMaker의 공식 MF-Tyre와 계수 동일할 때 결과 차이가 정량화되지 않았다.

**개선안**:

1. 공개 Pacejka 참조 데이터(예: TNO 샘플 `.tir`)로 VDSim vs Chrono vs CarMaker(공개 결과) 3-way 비교
2. `VALIDATION.md`에 MF2002 절대 오차 표 추가 (Fx/Fy/Mz at key operating points)
3. 실차 측정 데이터 있을 경우 (TUR ADMA 등) — ERG replay → NVDSim 대조 파이프라인 완성

**기대효과**: "공신력" 근거 생성. 논문/보고서에 인용 가능.

---

### P4 — ABS/ESC 인터페이스 완성 (1–2개월)

**문제**: `IBrakeSystem` 인터페이스는 있으나 ABS 로직 내장 모듈이 없다. 사용자가 직접 구현해야 한다.

**개선안**:

1. `SimpleABS` 기본 내장 모듈 — wheel slip λ 기반 on/off ABS (Bosch 구조)
2. `catalog` part 등록 (`brake.simple_abs`)
3. Python 훅 — 사용자가 Python function으로 ABS 로직 주입 가능 (`set_brake_policy`)

**설계 주의**: ABS는 알고리즘 개발의 대상이기도 하므로, 내장 구현을 reference로 제공하되 교체 가능하게 유지.

---

### P5 — EPS/MDPS 스티어링 모델 (2–3개월)

**문제**: 현재 ratio steer는 입력 = 출력이다. 컬럼 마찰(LuGre는 있음)은 있으나 assist torque, 가변 ratio, 컬럼 compliance가 없다.

**개선안**:

1. `EpsSteeringModule` — `ISteeringSystem` 구현
   - assist map: `T_assist = f(T_driver, vx)` (2D 테이블)
   - 가변 ratio: `ratio = g(steer_angle, vx)`
2. catalog part `steer.eps_basic`
3. 검증: 실차 랙힘 vs 추정랙힘 (TUR rack_torque 데이터 활용 가능)

---

### P6 — LSD (Limited Slip Differential) (1개월)

**문제**: open diff뿐. 고마찰 노면에서 과도한 spin, AWD/RWD 성능 차이 표현 불가.

**개선안**:

1. `IDrivetrain`에 `Torsen` / `viscous_lsd` 구현 추가
2. torque bias ratio (TBR) 파라미터로 lockup 정도 제어
3. catalog part `drivetrain.torsen_rwd`

---

### P7 — Bushing compliance (3–4개월, 선택적)

**문제**: Ld4 hardpoint kinematics는 있으나 bushing stiffness(toe/camber compliance)가 없다. 고주파 ride 응답 부정확.

**개선안**:

1. `SuspensionTopology`에 bushing stiffness 필드 추가 (`kx_bushing`, `ky_bushing`, `kz_bushing`)
2. 수직 하중 → bushing 변형 → toe/camber 변화 반영
3. 검증: ISO 8608 PSD 응답 vs Ld3 비교

복잡도가 높아 초기에는 생략 가능. RDE/NVH 타깃이 아니면 우선순위 낮음.

---

## 4. 로드맵 요약

| 우선 | 항목 | 규모 | 선행조건 |
|------|------|------|----------|
| P1 | Simulink/FMI workflow 문서 + 예제 | 소 (1인 2주) | 없음 |
| P2 | Python ↔ MATLAB 데이터 교환 툴 | 소 (1인 1주) | 없음 |
| P3 | 타이어 3-way 정량 검증 | 중 (1인 3주) | 공개 참조 데이터 |
| P4 | ABS SimpleABS + catalog | 중 (1인 2주) | 없음 |
| P5 | EPS/MDPS 스티어링 모듈 | 중 (1인 3주) | rack_torque 데이터 |
| P6 | LSD Torsen/viscous | 소 (1인 2주) | 없음 |
| P7 | Bushing compliance | 대 (1인 5주) | Ld4 완성 ✅ |

---

## 5. 전략 관점

**Simulink 연동은 생태계 진입 조건**이다. 제어 팀이 Simulink로 작업하는 한, VDSim plant를 FMU로 붙일 수 없으면 채택이 안 된다. P1이 가장 높은 ROI.

**물리 깊이 vs 공신력**: 물리를 더 정확히 구현하는 것보다, 현재 구현이 얼마나 정확한지 정량적으로 보여주는 것(P3)이 외부 사용자에게 더 설득력 있다. "구현했다"와 "검증됐다"는 다르다.

**ABS/ESC는 인터페이스 우선**: 사용자가 알고리즘을 VDSim 안에서 개발하고 싶다 → 내장 reference 모듈 + 교체 가능 인터페이스가 핵심. CarMaker처럼 blackbox ABS를 제공하는 게 목표가 아니다.

---

## 6. 하지 않을 것 (명시적 exclusion)

| 항목 | 이유 |
|------|------|
| dSPACE/NI HIL 인증 | 스타트업 단계 — UDP로 충분, 인증은 고객사 OEM 몫 |
| 엔진 열역학 | 동력계 성능이 아닌 차량 동역학이 주 관심 |
| 타이어 thermal | 레이싱/고속 연구 아님 — scope 밖 |
| OpenSCENARIO | ADAS 시나리오 비대상 |
| Autonomous traffic agent | 비대상 |
