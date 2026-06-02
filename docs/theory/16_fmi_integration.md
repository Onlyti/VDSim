# 16. FMI 2.0 Integration — 산업 표준 co-simulation

> **학습 목표.** FMI / FMU 가 무엇이고 차량 dynamics 업계에서 왜 표준인지
> 안다. Model Exchange vs Co-Simulation 의 차이를 안다. VDSim 의 FMU export
> (우리 모델을 산업 도구가 사용) 와 import (외부 모델을 VDSim 이 backend 로
> 사용) 의 양방향 구조와 그 전략적 의미를 안다.

## 16.1 FMI / FMU 란

**FMI (Functional Mock-up Interface)** — 서로 다른 시뮬 도구 사이에 동역학
모델을 주고받는 open standard (Modelica Association, BSD-2). C API + XML
metadata 규약. v2.0 이 현재 대세.

**FMU (Functional Mock-up Unit)** — `.fmu` 확장자, 실제로는 ZIP:
```
mymodel.fmu (ZIP)
├── modelDescription.xml      변수 정의 (이름, VR, 단위, causality)
├── binaries/<platform>/      컴파일된 .so / .dll
└── resources/                룩업 테이블, parameter 등
```

생성·소비 도구: Dymola, OpenModelica, MATLAB/Simulink, CarMaker, CarSim,
dSPACE, ANSYS Twin Builder, Chrono (개발 중) — 거의 모든 시뮬 도구.

## 16.2 두 가지 mode

| Mode | 설명 | 통합기(integrator) 위치 |
|---|---|---|
| **Model Exchange (ME)** | 모델은 `f(x,u,t) → ẋ` 만 제공 | 호스트가 RK4 등으로 적분 |
| **Co-Simulation (CS)** | 모델 안에 자체 integrator | 호스트는 `do_step(t, dt)` 만 호출 |

VDSim 의 FMU 는 **CS** — VDSim 의 RK4 substepping 을 내부에 가짐. 호스트는
입력 set → `fmi2DoStep` → 출력 get 만 하면 됨. 이게 우리 `IVehicleDynamics::step`
패턴과 자연스럽게 맞음.

## 16.3 핵심 C API 흐름

```c
fmi2Component c = fmi2Instantiate(name, fmi2CoSimulation, guid,
                                   resourceLocation, &callbacks, ...);
fmi2SetupExperiment(c, ...);
fmi2EnterInitializationMode(c);    // ← VDSim: YAML 로드 + dynamics init
fmi2ExitInitializationMode(c);

for (t = 0; t < T; t += dt) {
    fmi2SetReal(c, vr_in, n_in, inputs);    // steer, throttle, brake
    fmi2DoStep(c, t, dt, ...);              // VDSim dyn.step()
    fmi2GetReal(c, vr_out, n_out, outputs); // vx, yaw_rate, ay, ...
}
fmi2Terminate(c);
fmi2FreeInstance(c);
```

각 변수는 `valueReference` (정수 ID) 로 식별, `modelDescription.xml` 이
이름↔VR 매핑을 정의.

## 16.4 VDSim FMU export

`fmi_export/vdsim_l2_fmu.cpp`, `vdsim_l3_fmu.cpp`.

C ABI wrapper 가 내부에 VDSim dynamics 인스턴스를 소유:

- `fmi2Instantiate` — VDSimL2/L3 struct 생성, resourceLocation 기록.
- `fmi2EnterInitializationMode` — `<resources>/configs/{vehicle,tire}.yaml`
  로드 → `create_seven_dof()` / `create_fourteen_dof()` → initialize.
  L3 는 `<resources>/kinematics/{front,rear}.csv` 있으면 Ld4 lookup attach.
- `fmi2SetReal` — VR 1-3 (steer/throttle/brake) 캐시.
- `fmi2DoStep` — CmdL4 구성 → `dyn->step(u, contacts, h)`.
- `fmi2GetReal` — VR 10-17 (L2) / 10-43 (L3: + roll/pitch/susp/Fz) 출력.

빌드 (`build_fmu.sh`):

1. `libvdsim_core.a` + yaml-cpp + spdlog 정적 링크하여 .so 컴파일.
2. `-fvisibility=hidden` + `--version-script` 로 `fmi2*` 심볼만 노출.
3. configs YAML 을 resources/ 로 복사.
4. ZIP → `.fmu`.

출력: `vdsim_l2.fmu` (4.1 MB), `vdsim_l3.fmu` (4.6 MB, Ld4 kinematics 포함).

variable map (L2):
```
입력  1 steer_angle_wheel, 2 throttle, 3 brake
출력  10 x, 11 y, 12 yaw, 13 vx, 14 vy, 15 yaw_rate, 16 ax, 17 ay
L3 추가  20 roll, 21 pitch, 30-33 susp_comp, 40-43 Fz
```

## 16.5 VDSim FMU import

`fmi_export/fmu_master.py` — pure-Python ctypes, **fmpy 의존 없음**.

```python
fmu = FMUMaster.load("anything.fmu")
fmu.initialize(t0=0.0)
fmu.set("steer_angle_wheel", 0.05)
fmu.do_step(t=0.0, dt=0.02)
print(fmu.get("vx"))
```

구현:

1. ZIP 해제 → tempdir → `modelDescription.xml` 파싱 (이름→VR 맵).
2. `binaries/<platform>/<modelIdentifier>.so` 를 ctypes 로 로드.
3. FMI 2.0 CS 라이프사이클 함수 시그니처 바인딩.
4. 변수를 **이름**으로 접근 (raw VR 보다 친화적).

어떤 FMI 2.0 CS FMU 든 로드 — 우리 vdsim_l2/l3.fmu, Chrono Vehicle export,
CarMaker / CarSim export, Modelica 생성 FMU 등.

## 16.6 양방향의 전략적 의미

```
        ┌─ export ─→  CarMaker / dSPACE / Simulink 가 VDSim dynamics 사용
VDSim ──┤
        └─ import ─←  Chrono / CarMaker FMU 를 VDSim backend 로 사용
```

이로써 VDSim 은 **"vehicle dynamics backend 를 갈아끼울 수 있는 AV-focused
시뮬레이터"** 가 됨:

| backend | 용도 |
|---|---|
| VDSim native Ld1-Ld3 | 가벼움, 빠름, hardpoint 통합 |
| Chrono Vehicle FMU | full multibody 정확도가 필요할 때 |
| CarMaker FMU | 산업 reference 검증 |

정확도 약점 (Chapter: positioning) 을 FMI import 로 cover, 산업 진입을
FMI export 로 — 둘 다 같은 표준 인터페이스.

## 16.7 Round-trip 검증

`fmi_export/test_roundtrip.py`: native VDSim L2 vs FMU 로 감싼 VDSim L2 를
동일 입력·timestep 으로 100 step 비교:

```
max |Δ vx|       = 0.000e+00
max |Δ vy|       = 0.000e+00
max |Δ yaw_rate| = 0.000e+00
max |Δ ay|       = 0.000e+00
```

수치 정밀도 한계까지 0 차이 — export 경로가 native 와 동일함을 보증.

## 16.8 한계

| 항목 | 현재 |
|---|---|
| Platform | linux64 만 빌드 (win64/darwin 은 cross-compile 필요) |
| FMI version | 2.0 만 (3.0 의 clock/terminal 미지원) |
| Model Exchange | 미지원 (CS 만) |
| GetAndSetFMUstate | false (체크포인트/롤백 미지원) |
| Directional derivative | 미지원 (linearization 불가) |
| 외부 FMU 검증 | 우리 FMU round-trip 만; Chrono/CarMaker 는 패턴상 호환 (실측 검증 대기) |

## 16.9 참고

- FMI 표준: https://fmi-standard.org/  (FMI 2.0 spec, BSD-2 headers).
- FMPy: https://github.com/CATIA-Systems/FMPy (Python reference).
- Modelon FMI Library: https://github.com/modelon-community/fmi-library (C ref).
- 구현: `fmi_export/vdsim_{l2,l3}_fmu.cpp`, `fmu_master.py`, `build_*.sh`.
