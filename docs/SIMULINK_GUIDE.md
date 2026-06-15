# VDSim — Simulink / FMI 연동 가이드

VDSim L2 (7DOF) 차량 동역학을 Simulink co-simulation plant로 사용하는 방법.  
FMI 2.0 Co-Simulation 표준 기반 — Simulink FMU Block, fmpy, 자체 Python co-sim 세 경로 모두 지원.

---

## 개요

```
[Simulink Controller] --steer/throttle/brake--> [VDSim FMU] --x,y,yaw,vx,vy,r,ay--> [Simulink]
```

FMU 입력 (causality=input):

| 변수 | 단위 | 설명 |
|------|------|------|
| `steer_angle_wheel` | rad | 전륜 조향각 |
| `throttle` | 0..1 | 가속 페달 |
| `brake` | 0..1 | 제동 페달 |

FMU 출력 (causality=output):

| 변수 | 단위 | 설명 |
|------|------|------|
| `x_world`, `y_world` | m | 세계 좌표 위치 |
| `yaw` | rad | 요각 |
| `vx`, `vy` | m/s | 차체 좌표 속도 |
| `yaw_rate` | rad/s | 요 속도 |
| `ax_body`, `ay_body` | m/s² | 차체 가속도 |
| `Fz_FL..RR` | N | 각 차륜 수직 하중 (vRef 20..23) |

---

## 1단계: FMU 빌드

```bash
# 전제: cmake 빌드 완료 (build/ 디렉토리 존재)
bash fmi_export/build_fmu.sh
# → build/fmi_export/vdsim_l2.fmu  (~1.8 MB)
```

커스텀 차량/타이어:
```bash
VEH_YAML=configs/parts/body/my_car.yaml \
TIRE_YAML=configs/parts/tire/my_tire.yaml \
bash fmi_export/build_fmu.sh
```

round-trip 검증 (Δ=0 기준):
```bash
PYTHONPATH=build/python python3 fmi_export/test_roundtrip.py
# → PASS
```

---

## 2단계: Python co-sim (Simulink 없이 검증)

Simulink import 전에 FMU 동작을 먼저 확인한다.

```bash
PYTHONPATH=build/python:python python3 examples/simulink/fmu_step_steer.py
# Steps: 1600  |  vmax=21.27 m/s  peak_ay=5.033 m/s²  peak_r=0.2357 rad/s
# PASS
# CSV → results/fmu_step_steer.csv
```

코드 패턴:
```python
from fmu_master import FMUMaster          # fmi_export/fmu_master.py

fmu = FMUMaster.load("build/fmi_export/vdsim_l2.fmu")
fmu.initialize(0.0)

for k in range(N):
    t = k * dt
    fmu.set("steer_angle_wheel", steer(t))
    fmu.set("throttle",          throttle(t))
    fmu.do_step(t, dt)
    st = fmu.get_many("vx", "vy", "yaw_rate", "ay_body")

fmu.free()
```

---

## 3단계: Simulink FMU Block (MATLAB R2021b+)

### 3.1 FMU import

1. Simulink Library Browser → **FMI** → **FMU** 블록 캔버스에 드래그
2. 블록 더블클릭 → **FMU file**: `build/fmi_export/vdsim_l2.fmu` 선택
3. **Co-Simulation** 모드 선택 (Model Exchange 아님)
4. **Sample time**: 제어기 dt 와 동일하게 설정 (예: `0.005`)

### 3.2 입출력 연결

FMU Block 포트 자동 생성:
- 입력 포트 3개: `steer_angle_wheel`, `throttle`, `brake`
- 출력 포트 7개+: `x_world`, `y_world`, `yaw`, `vx`, `vy`, `yaw_rate`, `ay_body`, ...

제어기 → FMU 입력 → FMU 출력 → 제어기 피드백으로 연결.

### 3.3 초기 조건

FMU 초기 속도 설정:
```matlab
% Simulink callback (InitFcn 또는 PreLoadFcn):
set_param('my_model/VDSim_FMU', 'FMUInitializationParameters', ...
    'vx=15.0');  % [m/s] 초기 종방향 속도
```

또는 FMU Block → **Parameter** 탭에서 직접 설정.

### 3.4 실행

- Solver: **fixed-step** (FMU는 variable-step과 호환되지 않음)
- Step size: FMU sample time과 동일하게
- Run → Scope로 `vx`, `ay_body`, `yaw_rate` 확인

---

## 4단계: MATLAB script에서 FMU 직접 구동 (fmpy)

MATLAB 2022b+ 에서 Python fmpy 라이브러리 사용:

```matlab
% fmpy 설치: pip install fmpy
% MATLAB에서:
pe = pyenv('Version', '/usr/bin/python3');

result = py.fmpy.simulate_fmu( ...
    'build/fmi_export/vdsim_l2.fmu', ...
    start_time=0.0, stop_time=8.0, step_size=0.005, ...
    output={'vx','vy','yaw_rate','ay_body'});

t  = double(result{'time'});
vx = double(result{'vx'});
ay = double(result{'ay_body'});
plot(t, ay); xlabel('t [s]'); ylabel('ay [m/s^2]');
```

전체 예제: `examples/simulink/vdsim_step_steer.m`

---

## 5단계: MAT 파일 데이터 교환

Python 배치 결과를 MATLAB으로 가져오기:

```python
# Python 쪽
import scipy.io
sim.to_csv("run.csv")
scipy.io.savemat("run.mat", {
    "t":  [r[0] for r in sim.rows],
    "vx": [r[4] for r in sim.rows],
    "ay": [r[8] for r in sim.rows],
})
```

```matlab
% MATLAB 쪽
data = load('run.mat');
plot(data.t, data.ay);
```

유틸리티 스크립트: `tools/mat_export.py`

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| FMU import 오류 "Wrong platform" | Windows에서 Linux FMU | Linux에서 빌드 또는 Windows용 cross-compile 필요 |
| Δ 발산 (round-trip fail) | 차량/타이어 파라미터 mismatch | 동일 yaml로 FMU 재빌드 |
| Simulink "algebraic loop" 경고 | 직접 피드백 루프 | Unit Delay 삽입 또는 dt 검토 |
| `fmi2GetReal` 심볼 없음 | fmu.so export 누락 | `build_fmu.sh`의 version-script 확인 |

---

## FMI 버전 참고

| 버전 | 지원 여부 | Simulink 최소 버전 |
|------|----------|-------------------|
| FMI 2.0 | ✅ 구현, 검증 완료 | R2021b+ |
| FMI 3.0 | 계획 (P1 roadmap) | R2023b+ |

FMI 3.0은 variable-step, port array, 구조화 변수를 지원한다. VDSim의 FMI 2.0이 stable한 동안 3.0은 선택 업그레이드.

---

## 관련 파일

```
fmi_export/
  build_fmu.sh           FMU 빌드 스크립트 (sedan 기본)
  build_l3_fmu.sh        L3 14DOF FMU 빌드
  modelDescription.xml   FMI 2.0 변수 선언
  vdsim_l2_fmu.cpp       FMI C API 구현 (fmi2DoStep 등)
  fmu_master.py          Python ctypes FMU loader
  test_roundtrip.py      round-trip Δ=0 검증

examples/simulink/
  fmu_step_steer.py      Python co-sim 예제 (Simulink 불필요)
  vdsim_step_steer.m     MATLAB script 예제 (fmpy / FMUMaster)
```
