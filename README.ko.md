# VDSim

[English](README.md) · **한국어**

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)

Open-core 차량동역학 시뮬레이터. VDSim 은 **차량 그 자체**를 담당합니다 — 검증된
L1–L3 동역학 + 실제 Pacejka 타이어 + 설계검증 가능한 hardpoint 서스펜션 운동학 +
양방향 FMI 2.0. 렌더링·센서는 CARLA 같은 도구에 위임합니다. perception 스택에
없는 "섀시 정확도" 절반을 채웁니다.

> v0.1.0 — experimental / pre-release. analytic + ISO + cross-model
> self-consistency 로 검증 (자세히는 [VALIDATION](docs/VALIDATION.md)); 양산용 아님.

문서(이론·리포트): **https://onlyti.github.io/VDSim/** · 모든 실행 모드
(API / batch / FMI): [docs/RUNNING.md](docs/RUNNING.md)

## 설치

전제: C++17 컴파일러, CMake ≥ 3.20, Python ≥ 3.10.
- Linux: `g++ ≥ 9` 또는 `clang ≥ 10`
- Windows: Visual Studio 2019+ "C++ 데스크톱 개발"(MSVC)

Python 패키지 (Linux + Windows 공통):
```bash
pip install ".[plot]"
python -c "import vdsim; print('ok')"
```

전체 C++ 트리 (tests, real-time runtime, FMI, CARLA bridge):
```bash
cmake -B build -DVDSIM_BUILD_PYTHON=ON          # Linux 는 -G Ninja 추가
cmake --build build --config Release            # Linux 는 -j
ctest --test-dir build -C Release               # 190/190 ; 바이너리는 build/bin/
```

Python 간단 실험:
```python
from vdsim_lab import Experiment, Vehicle, Road, Maneuver, Sensors
res = (Experiment(level="L3").vehicle(Vehicle.preset("sedan"))
       .road(Road.preset("belgian_pave")).maneuver(Maneuver.step_steer(v=20, steer=0.03))
       .sensors(Sensors().gnss().imu()).run(8.0))
res.to_csv("run.csv"); print(res.summary())
```

## 시각화 — Web GUI

```bash
python3 gui/server.py --port 8100        # Windows: python gui\server.py --port 8100
```
브라우저 `http://localhost:8100`. GUI 가 real-time runtime 을 자동 기동해 렌더링:
3D 뷰(orbit / chase / cockpit), 도로·지형, per-wheel Fz / slip(κ, α) / 타이어 힘
벡터, telemetry HUD. 키보드(↑↓ 가감속, ←→ 조향) 또는 게임패드-휠로 운전 —
force feedback 은 `python tools/wheel_ffb_sdl.py --server <host> --udp-port 8101`.

## 제어 — real-time UDP runtime

`vdsim_realtime` 가 VDSim 의 real-time application 입니다: 같은 core 를 wall clock
으로 돌리며 고정포맷 바이너리 UDP 를 주고받음 — **CMD**(steer / throttle / brake)
입력, **STATE**(pose, 속도, Fz, slip, 타이어 힘, measured 센서 …) 출력. 이게
SIL / HIL / co-sim 경계이고, GUI·외부 제어기는 전부 이것의 클라이언트입니다.

```bash
build/bin/vdsim_realtime configs/vehicles/sedan.yaml configs/tires/default_pacejka.yaml \
    --level=L3 --cmd-port=7001 --state-port=7002 --rate=200
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
