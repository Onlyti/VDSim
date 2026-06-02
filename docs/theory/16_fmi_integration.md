# 16. FMI 2.0 Integration (산업 표준 co-simulation)

## Learning objectives

이 chapter 를 마치면 다음을 할 수 있다.

1. FMI / FMU 가 무엇이고 차량 dynamics 업계에서 왜 표준인지 설명한다.
2. Model Exchange vs Co-Simulation 의 차이 (integrator 위치) 를 설명한다.
3. FMU export (모델을 산업 도구가 사용) 와 import (외부 모델을 backend 로
   사용) 의 양방향 구조와 전략적 의미를 설명한다.
4. round-trip 검증이 export 경로의 정확성을 어떻게 보증하는지 안다.

## Prerequisites

- **Chapter 12** — IVehicleDynamics ABI (FMU wrapper 가 감싸는 대상).
- **Chapter 11** — RK4 substepping (CS FMU 의 내부 integrator).
- **외부** — FMI 2.0 spec 개요, C ABI / ctypes 기초.

---

## 16.1 동기 — FMI / FMU

FMI (Functional Mock-up Interface) 는 서로 다른 시뮬 도구 간에 동역학 모델을
교환하는 open standard (Modelica Association, BSD-2). C API + XML metadata,
v2.0 이 대세. FMU (Functional Mock-up Unit) 는 `.fmu` (실제 ZIP):

```
mymodel.fmu (ZIP)
├── modelDescription.xml   변수 정의 (이름, VR, 단위, causality)
├── binaries/<platform>/   컴파일된 .so / .dll
└── resources/             룩업 테이블, parameter
```

Dymola, OpenModelica, Simulink, CarMaker, CarSim, dSPACE, ANSYS Twin Builder,
Chrono 등 거의 모든 도구가 생성·소비한다 — 산업 진입의 공용 포맷이다.

---

## 16.2 두 mode

| Mode | 설명 | integrator 위치 |
|---|---|---|
| Model Exchange (ME) | 모델이 $f(x,u,t)\to\dot x$ 만 제공 | 호스트가 적분 |
| Co-Simulation (CS) | 모델 안에 자체 integrator | 호스트는 `do_step` 만 |

CS 가 자연스러운 선택이다 — 모델이 자체 RK4 substepping (chapter 11) 을 가지고
호스트는 입력 set → `do_step` → 출력 get 만 하면 된다. 이는
`IVehicleDynamics::step` 패턴과 그대로 맞는다.

---

## 16.3 가정 / 범위

| 가정 | 의미 | 깨지는 case |
|---|---|---|
| Co-Simulation only | 자체 integrator | Model Exchange 필요 host |
| FMI 2.0 | clock/terminal 없음 | FMI 3.0 기능 |
| linux64 binary | 단일 platform | win64/darwin |
| No FMUstate | checkpoint/rollback 없음 | rollback 필요 시나리오 |

---

## 16.4 핵심 C API 흐름

```c
fmi2Component c = fmi2Instantiate(name, fmi2CoSimulation, guid,
                                  resourceLocation, &callbacks, ...);
fmi2SetupExperiment(c, ...);
fmi2EnterInitializationMode(c);    // YAML 로드 + dynamics init
fmi2ExitInitializationMode(c);
for (t = 0; t < T; t += dt) {
    fmi2SetReal(c, vr_in, n_in, inputs);    // steer, throttle, brake
    fmi2DoStep(c, t, dt, ...);              // dyn.step()
    fmi2GetReal(c, vr_out, n_out, outputs); // vx, yaw_rate, ay, ...
}
fmi2Terminate(c);
fmi2FreeInstance(c);
```

각 변수는 `valueReference` (정수 ID) 로 식별하고 `modelDescription.xml` 이
이름↔VR 매핑을 정의한다.

---

## 16.5 FMU export

C ABI wrapper 가 내부에 dynamics 인스턴스를 소유한다: instantiate 시 struct
생성, init mode 에서 resources 의 YAML 로드 → factory → initialize (L3 는
kinematics CSV 있으면 Ld4 lookup attach), SetReal 로 steer/throttle/brake
캐시, DoStep 에서 CmdL4 구성 → step, GetReal 로 상태 출력. 빌드는 core static
lib 를 정적 링크하고 visibility hidden + version-script 로 `fmi2*` 심볼만
노출한 뒤 configs 를 resources 로 복사해 ZIP 한다 (상세 §16.10 box).

---

## 16.6 FMU import

pure-Python ctypes master (외부 의존 없음) 가 임의 FMI 2.0 CS FMU 를 로드한다:
ZIP 해제 → `modelDescription.xml` 파싱 (이름→VR) → `binaries/<platform>` 의
`.so` 를 ctypes 로 로드 → CS lifecycle 함수 바인딩 → 변수를 이름으로 접근. 자체
FMU 뿐 아니라 Chrono Vehicle / CarMaker / Modelica 생성 FMU 도 로드한다.

---

## 16.7 양방향의 전략적 의미

```
        ┌─ export ─→  CarMaker / dSPACE / Simulink 가 dynamics 사용
VDSim ──┤
        └─ import ─←  Chrono / CarMaker FMU 를 backend 로 사용
```

이로써 "vehicle dynamics backend 를 갈아끼울 수 있는 AV-focused 시뮬레이터" 가
된다.

| backend | 용도 |
|---|---|
| native Ld1-Ld3 | 가벼움, 빠름, hardpoint 통합 |
| Chrono Vehicle FMU | full multibody 정확도 |
| CarMaker FMU | 산업 reference 검증 |

정확도 약점은 FMI import 로 cover, 산업 진입은 FMI export 로 — 둘 다 같은 표준
인터페이스다.

---

## 16.8 검증 전략

| 검증 | 케이스 |
|---|---|
| Round-trip | native vs FMU-wrapped 동일 입력 100 step, max\|Δ\|=0 |
| Import lifecycle | load → init → do_step → get 값 정상 |
| 외부 FMU | 패턴상 호환 (실측 검증 대기) |

round-trip 의 max\|Δ\|=0 (수치 정밀도 한계) 은 export 경로가 native 와 동일함을
보증한다.

---

## 16.9 한계

| 항목 | 현재 |
|---|---|
| Platform | linux64 만 (win64/darwin cross-compile 필요) |
| FMI version | 2.0 만 (3.0 clock/terminal 미지원) |
| Model Exchange | 미지원 (CS 만) |
| GetAndSetFMUstate | false (체크포인트/롤백 없음) |
| Directional derivative | 미지원 (linearization 불가) |
| 외부 FMU 검증 | 자체 round-trip 만; Chrono/CarMaker 실측 대기 |

---

## 16.10 다음 단계 / 마무리

chapter 01-16 이 VDSim 의 이론 (frame → tire → Ld1-Ld4 → control → integrator
→ architecture → multibody → validation → FMI) 전체를 다뤘다. Phase 2 의 큰
항목은 Ld5 compliance (bushing), full DAE dynamics, MPC dispatch, FMI 3.0 /
multi-platform 이다.

---

## 16.11 참고문헌

- **FMI 표준**: https://fmi-standard.org/ (FMI 2.0 spec, BSD-2 headers).
- **FMPy**: https://github.com/CATIA-Systems/FMPy (Python reference).
- **Modelon FMI Library**: https://github.com/modelon-community/fmi-library (C ref).

---

## 16.12 Self-check

<details>
<summary>1. CS 가 ME 보다 VDSim 에 자연스러운 이유?</summary>

VDSim 이 이미 자체 RK4 substepping integrator 를 가져 `step(dt)` 패턴이다. CS
는 모델이 적분을 소유하므로 `do_step` 으로 그대로 매핑된다. ME 는 호스트에
integrator 를 넘겨야 해 구조가 안 맞는다.
</details>

<details>
<summary>2. valueReference 가 무엇이고 왜 필요한가?</summary>

각 변수의 정수 ID. C API 는 이름 문자열 대신 VR 로 set/get 한다.
modelDescription.xml 이 이름↔VR 매핑을 정의해 호스트가 매핑을 알 수 있다.
</details>

<details>
<summary>3. FMU export 시 fmi2* 심볼만 노출하는 이유?</summary>

내부 (vdsim, yaml-cpp, spdlog) 심볼이 새어 나가면 host 의 다른 FMU/lib 와
충돌할 수 있다. visibility hidden + version-script 로 FMI API 만 노출해 격리.
</details>

<details>
<summary>4. round-trip max|Δ|=0 이 보증하는 것은?</summary>

FMU wrapper 가 native dynamics 와 bit-equal — export 경로가 어떤 계산도
바꾸지 않음을 의미. 산업 도구가 받는 결과가 native 와 동일하다는 신뢰.
</details>

<details>
<summary>5. FMI import 가 정확도 약점을 cover 한다는 의미?</summary>

native Ld1-Ld3 의 fidelity 가 부족한 경우, 같은 인터페이스로 Chrono/CarMaker
의 full multibody FMU 를 backend 로 끼워 정확도를 끌어올릴 수 있다.
</details>

---

## 16.13 VDSim 구현 노트

> **[VDSim impl] § 16.5 — export 코드 + variable map**
>
> `fmi_export/vdsim_l2_fmu.cpp`, `vdsim_l3_fmu.cpp`. init 에서
> `<resources>/configs/{vehicle,tire}.yaml` → `create_seven_dof()` /
> `create_fourteen_dof()`; L3 는 `<resources>/kinematics/{front,rear}.csv` 있으면
> Ld4 lookup attach.
>
> ```
> 입력  1 steer_angle_wheel, 2 throttle, 3 brake
> 출력  10 x, 11 y, 12 yaw, 13 vx, 14 vy, 15 yaw_rate, 16 ax, 17 ay
> L3 추가  20 roll, 21 pitch, 30-33 susp_comp, 40-43 Fz
> ```
> build (`build_fmu.sh`): `libvdsim_core.a` + yaml-cpp + spdlog 정적 링크 →
> `-fvisibility=hidden` + `--version-script` (fmi2* 만) → configs 복사 → ZIP.
> 출력 `vdsim_l2.fmu` (4.1 MB), `vdsim_l3.fmu` (4.6 MB, Ld4 포함).

> **[VDSim impl] § 16.6 — import master**
>
> `fmi_export/fmu_master.py` (ctypes, fmpy 의존 없음):
>
> ```python
> fmu = FMUMaster.load("anything.fmu")
> fmu.initialize(t0=0.0)
> fmu.set("steer_angle_wheel", 0.05)
> fmu.do_step(t=0.0, dt=0.02)
> print(fmu.get("vx"))
> ```

> **[VDSim impl] § 16.8 — round-trip 검증**
>
> `fmi_export/test_roundtrip.py` (native L2 vs FMU-wrapped L2, 100 step):
> ```
> max |Δ vx| = max |Δ vy| = max |Δ yaw_rate| = max |Δ ay| = 0.000e+00
> ```
