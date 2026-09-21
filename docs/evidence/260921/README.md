# [260921]_[Evidence]_[PreflightC1C2WheelImportClosure]

Pre-flight C-1 / C-2 실행 로그. Q19-b(wheel import 폐포 강화) 완결 근거.

## 배경

- 문제: 배포 wheel 에 `vdsim_preset.py` 가 누락된 채 CI verify 를 통과한 사건(Q19 발단).
  - 원인: `CIBW_TEST_COMMAND` 가 `import vdsim` + Sim 2종까지만 확인. 1차 party 모듈 폐포 미검사.
  - 소스트리 안에서 실행하면 누락이 영구히 보이지 않음 — 소스트리 경로가 `sys.path` 를 채움.
- 조치: `tools/wheel_import_smoke.py` 신설(PR #8). `python/CMakeLists.txt` 의 SKBUILD install 목록 단일 소스에서
  모듈명을 읽어, 설치된 wheel 에서 전부 import 하고 각 모듈의 `__file__` 이 체크아웃 밖임을 단언.
- 본 폴더는 그 조치가 **main head 에서** 실효함을 보이는 실행 로그.

## 측정 대상

| 항목 | 값 |
| --- | --- |
| main head | `3bf4e6f` (PR #8 merge commit) |
| build run | [35547994686](https://github.com/Onlyti/VDSim/actions/runs/35547994686) — 5잡 success |
| wheels run (main head) | [35549072933](https://github.com/Onlyti/VDSim/actions/runs/35549072933) — 6잡 success, publish skipped |
| wheel version | `0.7.0.dev134` |

## 파일

| 파일 | 내용 |
| --- | --- |
| `build_jobs.txt` | build 워크플로 잡별 결론 |
| `build_test_counts.txt` | 잡별 ctest 원문(`Total Tests` / `tests passed`) 및 cmake 버전 |
| `wheels_jobs.txt` | wheels 워크플로 잡별 결론 |
| `wheels_artifact_list.txt` | 산출 wheel 9종 파일명 |
| `wheels_import_smoke.txt` | C-1 근거. wheel 9종 각각에서 `all 13 shipped modules import from the installed wheel` |
| `c2_linux.txt` | C-2 linux. docker `python:3.11-slim` clean env |
| `c2_windows.txt` | C-2 windows. win_lab Python 3.12 신규 venv |
| `driver_chain.txt` | 4단계 체인 전체 실행 로그 |
| `c2_windows_harness.sh` | C-2 windows 실행 하네스. 선언된 콘솔 진입점 `vdsim-quickstart` 로 고친 판 |
| `trace_members_windows.txt` | windows C-2 산출 `demo.vdtrace` 의 컨테이너 인벤토리(크기·멤버 14개·sha256). 2026-09-21 win_lab 실측 |

## 판정 기준과 결과

- C-1 = main head wheels run 에서 전 잡 success + test 단계가 import 폐포 스모크를 실행한 로그가 보일 것.
  - 결과: 충족. feature 브랜치 green 은 근거로 쓰지 않음.
- C-2 = 그 run 의 artifact wheel 만으로 clean env 에서 06 quickstart 와 07 데모를 소스트리 없이 재현할 것.
  - 결과: linux·windows 양쪽 충족. 작업 디렉터리에 소스트리 마커 0건.

| 항목 | linux (py3.11) | windows (py3.12) |
| --- | --- | --- |
| 모듈 해석 위치 | `site-packages` | `site-packages` |
| `run.csv` | 275302 B (1602행 × 30열) | 276904 B (1602행 × 30열) |
| `run.png` | 45041 B | 41076 B |
| `demo.vdtrace` | 39718 B (14 members) | 39718 B (14 members) |
| `demo_grip_loss.gif` | 2352713 B (120 frames) | 2386928 B (120 frames) |
| 데모 벽시계 | 16.04 s | 42.90 s |

- 두 OS 의 `demo.vdtrace` 는 크기(39718 B)와 멤버 수(14)가 같다. 근거는 linux 가 `c2_linux.txt` 의
  `trace bytes 39718 members 14`, windows 가 `trace_members_windows.txt` 의 컨테이너 인벤토리다.
- 내용 동일성은 해시로 확인하지 않았다. windows sha256 만 기록돼 있고, linux 산출물은 docker
  컨테이너와 함께 소멸해 대조 대상이 없다. peak slip ratio 가 16.44(linux) / 16.42(windows) 로 다르므로
  수치적으로 동일하지 않다(unknown).
- `run.csv`·GIF 의 바이트 차이도 원인을 분리 측정하지 않았다. 부동소수 텍스트 표기·GIF 인코더 차이가 후보이나,
  위 slip ratio 차이가 있으므로 시뮬 수치 차이가 섞였을 가능성을 배제하지 못한다(unknown).
- 콘솔 진입점 3종(`vdsim-quickstart`, `vdsim-render`, `vdsim-render3d`) 양 OS 모두 설치 확인.

## 한계

- 본 로그는 배포 가능성(import·실행)만 증명함. 수치 정확도 검증은 `docs/VALIDATION.md` 소관.
- `trace_members_windows.txt` 의 sha256 은 windows 단독 값이다. linux 대조본이 없으므로 비교 근거로
  쓰지 말 것 — 재확인하려면 양 OS 에서 C-2 를 다시 돌려 둘 다 떠야 한다.
- 크로스 OS 산출물의 비트 동일성은 주장하지 않는다. 해시 대조 상주 테스트가 없으므로 "byte-identical"·
  "platform independent" 는 이 증거로 뒷받침되지 않는다(unknown).
- 스모크는 install 목록에 **선언된** 모듈만 검사. 선언 자체가 누락된 모듈은 `tests/scripts/test_render_presets.py`
  의 소스트리 폐포 검사가 담당(두 검사가 짝).
