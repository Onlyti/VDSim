# VDSim

[English](README.md) · **한국어**

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)
![tests](https://img.shields.io/badge/ctest-404%2F404-green.svg)

> **Experimental / pre-release — 양산용 아님.** 근거·한계:
> [VALIDATION.md](docs/VALIDATION.md) (v0.5.1+).
> **검증됨:** analytic + ISO 기동 + L1↔L3 self-consistency + 동일 `.tir` pure-slip
> cross-check (CarMaker &lt;0.1%, Chrono Pac02 ~0.8%) — MF-Tyre 제품 parity 아님.
> **미검증:** full-vehicle 상용 cross-val, 실차 데이터, production sign-off.

![Grip-loss demo](docs/assets/demo_grip_loss.gif)

*결정론적 VDSim plant: 컨트롤러가 사전에 모르는 저-μ patch를 통과하며 타이어가 peak
너머 포화(drift>1) — soft-clamp가 아닌 실제 grip loss.* 재현:
`python examples/demo_grip_loss.py` (`pip install vdsim[plot]` 또는 로컬 wheel).

Open-core 차량동역학 시뮬레이터. VDSim 은 **차량 그 자체**를 담당합니다 — 검증된
L1–L5 동역학 + 실제 Pacejka MF / LuGre / belt-transient 타이어 + 설계검증 가능한
hardpoint 서스펜션 운동학 + 양방향 FMI 2.0. 렌더링·센서는 CARLA 같은 도구에
위임합니다. perception 스택에 없는 "섀시 정확도" 절반을 채웁니다.

문서(이론·리포트): **https://onlyti.github.io/VDSim/** · 모든 실행 모드
(API / batch / FMI): [docs/RUNNING.md](docs/RUNNING.md) · catalog·물리 옵션:
[docs/CATALOG_AND_PHYSICS.md](docs/CATALOG_AND_PHYSICS.md)

## Quickstart (pip wheel)

전제: Python 3.10–3.12. 미리 빌드된 wheel 설치(로컬 빌드는 [소스](#소스-빌드) 참고).

```bash
pip install "./vdsim-*.whl[plot]"
vdsim-quickstart          # cwd 에 run.csv + run.png 생성
```

측정 (clean conda, Python 3.11, Linux x86_64, 2026-06-25): **첫 결과까지 수 초** —
pip install `[plot]` + `vdsim-quickstart` → `run.csv` + `run.png` **~6 s wall-clock**
(cold; lab 네트워크, matplotlib wheel 포함). 재실행 ~1.5 s.

설치 확인:

```bash
python -c "import vdsim; from vdsim_lab import Sim; Sim(vehicle='sedan', level='L2')"
```

스크립트: [`examples/quickstart.py`](examples/quickstart.py) — `from vdsim_lab import Sim, Road` 만 사용.

## 소스 빌드

전제: C++17 컴파일러, CMake ≥ 3.20, Python ≥ 3.10.
- Linux: `g++ ≥ 9` 또는 `clang ≥ 10`
- Windows: Visual Studio 2019+ "C++ 데스크톱 개발"(MSVC)

Python 패키지 (editable / 로컬 wheel 빌드):
```bash
pip install ".[plot]"
python -c "import vdsim; print('ok')"
```

전체 C++ 트리 (tests, real-time runtime, FMI, CARLA bridge):
```bash
cmake -B build -DVDSIM_BUILD_PYTHON=ON          # Linux 는 -G Ninja 추가
cmake --build build --config Release            # Linux 는 -j
ctest --test-dir build -C Release               # 328/328 ; 바이너리는 build/bin/
```

## Python 으로 실험하기 (제어기는 직접 작성)

VDSim 은 **시뮬레이션 seam** 만 제공하고, 루프와 알고리즘은 당신이 소유합니다.
시나리오 파일·네트워크 없이 core 에서 바로 plant 를 만들고(vehicle / tire / level /
road), `set_input(action) → run_core_dt()` 로 구동하며 `state()` / `measurements(id)`
를 읽습니다.

```python
from vdsim_lab import Sim, Road, Sensors

sim = Sim(vehicle="sedan", level="L2", road=Road.iso8608("C"),
          sensors=Sensors().gnss(pos_std=0.3).imu(), v0=12.0,
          sensor_mounts={"gnss": {"type": "gnss", "pos": [1.4, 0, 1.0]}})

while not sim.done(12.0):
    st = sim.state()                                  # ground truth
    gnss = sim.measurements("gnss")                   # noisy, mount pose 기준
    steer, throttle, brake = my_controller(st, gnss)  # <-- 당신의 알고리즘
    sim.set_input(steer=steer, throttle=throttle, brake=brake)
    sim.run_core_dt()                                 # core 한 스텝 전진

sim.to_csv("run.csv")                                 # ground-truth + per-wheel 로그
sim.metrics(["peak_ay", "cte_rms", "lap_time"])       # 스칼라 지표
sim.plot("run.png", signals=("vx", "ay", "r", "xy"))  # 옵션 (matplotlib 필요)
```

`templates/experiment_template.py` 를 복사해 `controller(...)` 만 채우고 클론 트리에서 실행:
```bash
PYTHONPATH=build/python:python python3 templates/experiment_template.py
```
예제 (클론+빌드 필요): `examples/experiment_quickstart.py`,
`examples/experiment_path_follow.py` (pure-pursuit + CTE). pip 사용자는
[`examples/quickstart.py`](examples/quickstart.py) / `vdsim-quickstart` 사용. 전체 레퍼런스(seam, `Sim(...)` 옵션, evidence):
**[docs/EXPERIMENT_API.md](docs/EXPERIMENT_API.md)**. real-time 서버·batch 러너가
쓰는 `set_input → tick` 과 동일한 seam — real-time 은 action 이 UDP 로, 여기선
당신 함수에서 들어옵니다.

## 시각화 — Web GUI

```bash
python3 gui/server.py --port 8100        # Windows: python gui\server.py --port 8100
```
브라우저 `http://localhost:8100`. GUI 가 real-time runtime 을 자동 기동해 렌더링:
3D 뷰(orbit / chase / cockpit), 도로·지형, per-wheel Fz / slip(κ, α) / 타이어 힘
벡터, telemetry HUD. 키보드(↑↓ 가감속, ←→ 조향) 또는 게임패드-휠로 운전 —
force feedback 은 `python tools/wheel_ffb_sdl.py --server <host> --udp-port 8101`.

## 시각화 — 헤드리스 trace 렌더

한 번의 실행을 `.vdtrace` 파일 하나로 기록해 두고, 나중에 GUI·node/npm·브라우저
없이 렌더할 수 있다. 기록은 opt-in 이라 켜지 않으면 동작하지 않는다.

```python
plant.enable_trace("run.vdtrace", seed=0, run_id="demo")   # 켜야만 기록
...                                                        # 평소처럼 step()
path = plant.finalize_trace()
```

- `enable_trace(path, decimation=None, seed=None, run_id=None, producer=None, tags=None)` —
  `step()` 마다 한 샘플을 적분 *이전* 시점에서 취한다. 따라서 pose 는 시각 `t` 의
  상태이고 `u_steer` / `u_fx` 는 `[t, t+control_dt)` 구간에 유지된 명령이다.
  반환값은 실제 적용된 decimation.
- `decimation=None` 이면 기록률이 100 Hz 이상으로 유지되는 최소 N 을 고른다
  (1 kHz 제어 → N=10, 20 Hz 루프는 N=1 로 손실 없음).
- `finalize_trace()` 는 채널을 flush 하고 manifest 를 확정한 뒤 파일을 닫으며,
  기록한 경로를 돌려준다(기록을 켠 적이 없으면 `None`).
- 컨테이너는 zip: `manifest.json` + `channels/*.f64` + `overlays/*.json`.
  이 파일 하나면 렌더가 되므로 재시뮬레이션도, 결과 파일 재파싱도 필요 없다.

시나리오 지식은 실행이 끝난 뒤 **overlay** 로 붙인다. VDSim 은 `kind` / `name`
봉투만 검증하고 내용은 해석하지 않으므로, 새 생산자가 쓴 미지의 overlay 를
현재 렌더러가 무시하고 지나갈 수 있다.

```python
import vdsim_trace
vdsim_trace.attach_overlay(path, {"kind": "path2d", "name": "intended_path",
                                  "xy": [[0.0, 0.0], [4.0, 0.0]]})
vdsim_trace.attach_overlay(path, {"kind": "region", "name": "mu_patch", "mu": 0.35,
                                  "polygon": [[40, -30], [60, -30], [60, 30], [40, 30]]})
```

렌더러는 trace 하나를 입력받아 GIF 와 미리보기 PNG 를 쓴다(`[plot]` extra 의
matplotlib + pillow 사용).

```bash
vdsim-render run.vdtrace --out run.gif --fps 20     # 설치된 wheel
PYTHONPATH=python python3 -m vdsim_render run.vdtrace --out run.gif   # 클론 트리
```

옵션: `--png` 미리보기 경로 · `--stride` 프레임 간격 · `--fps` · `--dpi` ·
`--title` · `--mp4`(`imageio-ffmpeg` 설치 시에만).

BEV 화면에 그려지는 것: 기준 경로(점선, `path2d` overlay 가 있을 때만) · 주행
궤적 · manifest `geometry` 로 크기를 잡은 차체 사각형 · 기록된 조향 명령만큼
돌아간 앞바퀴 · 속도 화살표 · 바퀴별 마찰 이용률 색 · 화면 고정 좌표의 HUD ·
현재 시각 커서가 붙은 `u_steer` / `u_fx` 시계열. 축 범위는 autoscale 이 아니라
`geometry` 에서 계산하므로 프레임마다 스케일이 흔들리지 않는다.

기록 → overlay → 렌더 전 과정 예시:

```bash
PYTHONPATH=build/python:python python3 examples/demo_grip_loss.py \
    --out demo.gif --keep-trace
```

## 설정 — parts catalog & scene (v0.3)

차량 = `configs/parts/` 조합 **blueprint**, 실행 = `fleet[]`가 있는 **scene**
(`configs/scenes/`). 레이아웃·GUI API·drivetrain 관성·LuGre 타이어(opt-in)는
[CATALOG_AND_PHYSICS](docs/CATALOG_AND_PHYSICS.md) 참고.

## 제어 — real-time UDP runtime

`vdsim_realtime` 가 VDSim 의 real-time application 입니다: 같은 core 를 wall clock
으로 돌리며 고정포맷 바이너리 UDP 를 주고받음 — **CMD**(steer / throttle / brake)
입력, **STATE**(pose, 속도, Fz, slip, 타이어 힘, measured 센서 …) 출력. 이게
SIL / HIL / co-sim 경계이고, GUI·외부 제어기는 전부 이것의 클라이언트입니다.

```bash
build/bin/vdsim_realtime --scene=configs/scenes/two_vehicle_race.yaml \
    --cmd-port=7001 --state-port=7002 --rate=200
```
wire 포맷 = canonical VDS1 바이너리 프로토콜 (`cosim/cosim_protocol.hpp`; 계약서는
[docs/vdsim_bridge_interface_requirements.md](docs/vdsim_bridge_interface_requirements.md)).
Python 클라이언트는 `cosim/protocol.py` 로 CMD 인코딩 / STATE 디코딩해 byte-호환.

## 규약

SI 단위 · body frame ISO 8855 RH (X 전방, Y 좌, Z 상) · world frame ENU ·
wheel 인덱스 **FL = 0, FR = 1, RL = 2, RR = 3**.

## 라이선스

Apache-2.0 (core / kinematics / tools / validation / FMI export). FMI 2.0 헤더는
BSD-2-Clause (Modelica Association). 서드파티(Eigen / yaml-cpp / spdlog /
GoogleTest)는 각자 라이선스; CMake FetchContent 로 가져옴.
