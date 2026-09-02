# VDSim

[English](README.md) · **한국어**

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)

> **Experimental / pre-release software — not for production use.**
> What is verified, and what is *not yet*: see [docs/VALIDATION.md](docs/VALIDATION.md).

**Verification scope.** VDSim's tire-force layer is cross-checked against two independent
implementations driven by the same `.tir` parameter file: CarMaker MF-Tyre/MF-Swift
(pure longitudinal 0.00%, pure lateral 0.09%) and Chrono Pac02 (BSD-3, ~0.8% / 0.7%).
This is a pure-slip cross-check between implementations, not product parity.
Full-vehicle behaviour (suspension, transient) and comparison against real-vehicle
measurements are **not yet** covered.

![Grip-loss demo](docs/assets/demo_grip_loss.gif)

*결정론적 VDSim plant: 컨트롤러가 사전에 모르는 저-μ patch를 통과하며 타이어가 peak
너머 포화(drift>1) — soft-clamp가 아닌 실제 grip loss.* 재현:
`python examples/demo_grip_loss.py` (`pip install vdsim[plot]` 또는 로컬 wheel).

Open-core 차량동역학 시뮬레이터. VDSim 은 **차량 그 자체**를 담당합니다 — 구현된
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

- `enable_trace(path, decimation=None, seed=None, run_id=None, producer=None, tags=None, role="plant")` —
  `step()` 마다 한 샘플을 적분 *이전* 시점에서 취한다. 따라서 pose 는 시각 `t` 의
  상태이고 `u_steer` / `u_fx` 는 `[t, t+control_dt)` 구간에 유지된 명령이다.
  반환값은 실제 적용된 decimation.
- `decimation=None` 이면 기록률이 100 Hz 이상으로 유지되는 최소 N 을 고른다
  (1 kHz 제어 → N=10, 20 Hz 루프는 N=1 로 손실 없음).
- `finalize_trace()` 는 채널을 flush 하고 manifest 를 확정한 뒤 파일을 닫으며,
  기록한 경로를 돌려준다(기록을 켠 적이 없으면 `None`).
- 컨테이너는 zip: `manifest.json` + `channels/*.f64` + `overlays/*.json`.
  이 파일 하나면 렌더가 되므로 재시뮬레이션도, 결과 파일 재파싱도 필요 없다.
- manifest 는 `schema_version` `"0.2"` 와 필수 필드 `role` 을 선언한다. `role` 은
  검증 대상인 `"plant"` 또는 최적화·MPC 내부 예측 모델로 쓰인 `"predictor"` 다.
  `VDSimPlant` 은 그 자체가 플랜트이므로 `enable_trace` 의 기본값은 `plant` 이고,
  `vdsim_trace.TraceWriter` 를 직접 만드는 생산자는 `role=` 을 반드시 넘겨야 한다
  — 기본값이 없으므로 예측기 run 이 빠뜨림만으로 플랜트 근거가 되는 일이 없다.
  기존 `0.1` trace 도 그대로 읽힌다. `role` 이 없으면 경고 1회와 함께 `plant` 로
  간주하고, `0.2` 에서 누락되면 에러다. 렌더러는 HUD 에 문자열로만 표시하고
  이 값으로 화면 구성을 바꾸지 않는다.

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
`--title` · `--mp4`(`imageio-ffmpeg` 설치 시에만). 제어기 호라이즌용으로
`--sidecar` / `--view-half` / `--preview-frame` 이 추가된다(아래 참고).

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

### 여러 run 겹쳐 그리기

trace 를 2개 이상 넘기면 하나의 화면에 겹쳐 그린다 — 카메라 하나, 시계 하나, run
마다 다른 색, 주행 경로도 차량과 같은 색:

```bash
vdsim-render base.vdtrace tuned.vdtrace wet.vdtrace --out compare.gif \
    --labels "base,tuned,wet" --alpha 0.5
```

- 정렬 기준은 샘플 인덱스가 아니라 **시간**이다. 프레임 시각에서 각 run 을 보간하므로
  `dt` 가 다르거나 길이가 다른 trace 도 올바르게 겹친다. yaw 는 unwrap 후 보간하므로
  ±π 부근 헤딩이 반대로 도는 일이 없다.
- 먼저 끝난 run 은 마지막 자세를 유지하고 alpha 35 % 로 흐려지며, HUD 에 `(ended)`
  로 표시되고 경로도 더 이상 늘어나지 않는다.
- `--alpha`(차체 채움, 기본 0.55)와 `--path-alpha`(기본 0.9)로 겹친 차량이 얼마나
  비쳐 보일지 조절한다. `--colors` 로 색을, `--labels` 로 범례를 지정한다(기본값은
  manifest 의 `run_id`, 없으면 파일 이름).
- 카메라: `--follow fit`(기본)은 모든 경로를 담는 고정 정사각 창, `--follow 1` 은 해당
  run 을 geometry 기반 창으로 추적한다.
- `--speed` 는 실시간 대비 재생 속도(`--stride` 는 단일 run 전용). 각 run 의 전체 경로가
  옅게 깔리며 `--no-ghost` 로 끌 수 있다.
- 차체 색을 run 식별에 쓰므로 그립은 바퀴로 옮겼다 — utilization 이 0.8 을 넘으면 해당
  타이어 외곽선이 빨간색이 된다. 오른쪽 패널은 모든 run 의 `u_steer` / `u_fx` 와 최대
  utilization 을 공통 시간축에 겹쳐 보여준다.
- 미리보기 PNG 는 임의 시점이 아니라 run 들이 **가장 크게 벌어진** 프레임에서 뽑는다.

전 과정 — 같은 조작을 마찰계수 3단계로 기록해 3개의 trace 를 겹쳐 그리기:

```bash
PYTHONPATH=build/python:python python3 examples/demo_compare_runs.py --out compare.gif
```

### 제어기 호라이즌 — waypoint / target / prediction

"reference" 라는 한 단어로 뭉뚱그려지곤 하는 세 곡선은 서로 다른 것이다. 렌더러는
코드·사이드카 키·범례에서 이름을 구분해서 쓴다.

- **waypoint** — 최종적으로 따라가야 할 전역 경로. 시간 불변, 시나리오당 1개이며
  `path2d` overlay 로 들어와 회색 점선으로 한 번만 그려진다. overlay `name` 은
  `waypoint` / `reference_path` / `ref_path` 를 모두 받는다(앞의 것이 권장,
  나머지는 옛 trace 호환용).
- **target (MPC input)** — *이번* 스텝에 제어기로 들어가는 레퍼런스 호라이즌
  (N+1 점). 시간 가변이며 점 간격이 계획 속도를 따르므로 감속 구간에서 압축된다.
  간격이 보이도록 마커가 있는 파란 선.
- **prediction (MPC output)** — 제어기가 이번 스텝에 풀어낸 호라이즌(N+1 점).
  시간 가변이고, 실패한 스텝에서는 솔버가 돌려준 값 그대로다. 주황색이며 솔버
  status 가 0이 아닌 동안 굵은 빨강으로 바뀐다.

`target` / `prediction` 은 `(K, N+1, 2)` 배열이고 trace 컨테이너의 채널 표는
스칼라·고정폭 행의 화이트리스트라, 컨테이너 안이 아니라 trace 옆의 **사이드카**
`.npz` 로 실어 나른다.

```
run.vdtrace          # trace 본체, 이것만으로도 렌더된다
run.qp.npz           # 선택적 호라이즌 사이드카, 있으면 자동으로 붙는다
```

| 키 | 모양 | 의미 |
| --- | --- | --- |
| `t` | (K,) | 스텝 시각 [s] — 인덱스가 아니라 최근접 시간으로 trace 샘플과 맞춘다 |
| `tgt_XY` | (K, N+1, 2) | target 호라이즌, world 좌표 [m] |
| `pred_XY` | (K, N+1, 2) | prediction 호라이즌, world 좌표 [m] |
| `status` | (K,) | 솔버 status, `0` = 성공 |
| `solve_ms` | (K,) | solve 시간 [ms] — 선택, HUD 에 표시 |
| `tgt_v` | (K, N+1) | target 을 따라가는 계획 속도 [m/s] — 선택 |
| `ego_XY` | (K, 2) | 스텝별 후륜축 위치 — 선택, 화면 범위 산출에 쓰임 |

필수는 앞의 4개뿐이고 사이드카 자체가 선택이다. 없으면 렌더러는 이전과 완전히
동일하게 동작하며 호라이즌 artist 를 아예 만들지 않는다.

- 호라이즌은 **world** 좌표로 저장할 것. ego / Frenet 프레임에서 푸는 제어기는
  역변환을 먼저 적용해야 한다. 검수 기준은 `pred_XY[k, 0]` 이 그 스텝의 차량
  후륜축 위치와 0.1 m 이내로 일치하는지다. 어긋나면 프레임이 틀린 것이다.
- 실패한 스텝은 실패한 그대로 기록할 것. 직전 값으로 덮어쓰지 말고 그 스텝의
  `pred_XY` 를 NaN 이면 NaN 째로 남긴다. 렌더러가 비유한값을 건너뛰며, 덮어쓰면
  영상으로 보여주려던 실패 자체가 사라진다.

```bash
vdsim-render run.vdtrace --out run.gif --view-half 100 --preview-frame first-fail
```

- `--sidecar` 로 `.npz` 를 직접 지정한다. 기본값은 trace 경로의 확장자를 `.qp.npz`
  로 바꾼 경로(`run.vdtrace` -> `run.qp.npz`)가 있으면 그것, 없으면 사이드카
  없음이다. 플래그를 명시하면 파일이 없을 때 조용히 넘어가지 않고 오류가 난다.
- `--view-half` 는 BEV 반창 크기 [m]. 기본 추적창은 축거에서 유도되어(축거 약 6배)
  MPC 호라이즌보다 훨씬 짧으므로, 100 m 짜리 호라이즌은 창을 넓히지 않으면 화면
  밖으로 나간다.
- `--preview-frame util-peak`(기본, 기존 동작 유지)은 마찰 이용률 피크에서 미리보기
  PNG 를 뽑고, `first-fail` 은 첫 실패 스텝에서 뽑는다. 실패가 한 번도 없으면
  안내를 출력하고 util-peak 으로 되돌아간다.
- 실패가 하나라도 있는 run 은 `--preview-frame` 값과 무관하게 GIF 옆에
  `<out>_firstfail.png` 를 함께 쓰고, 실패가 이어지는 동안 BEV 에
  `QP FAIL (status=N)` 배너를 띄운다.
- 실패 시각은 커맨드 패널에 세로 구간으로 표시되며, 연속 실패 스텝은 하나로 병합된다.
  생산자가 `qp_fail` 이라는 이름의 `event` overlay 를 붙였으면 그것을, 없으면
  사이드카의 `status` 배열을 쓴다.

세 옵션 모두 단일 run 전용이라 겹침 모드에서는 무시된다.

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
