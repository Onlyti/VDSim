# Task 07 — IM-W1 Monorepo skeleton + CI + 첫 commit

| Field | Value |
|---|---|
| Task ID | IM-W1, push.log, LICENSE, CI |
| Type | Impl |
| Date | 2026-05-28 / 2026-05-29 |
| Commit | `d267221` (skeleton+LICENSE) / `4449cf6` (CI) |
| Status | completed |

## 1. 목적

W1-W2 가이드의 monorepo 구조를 이 repo (`/home/ailab-12/git/VDSim`) 에 적용 + 빌드 sanity + GitHub Actions CI. 이 단계 통과 못 하면 모든 후속 작업이 막힌다.

## 2. 구현 방법

### Layout

```
VDSim/
├── core/{include/vdsim, src}        # libvdsim_core C++17
├── python/                          # pybind11 (Phase 2)
├── carla_integration/{plugin,patches}  # W7+
├── third_party/                     # FetchContent
├── tests/{unit,integration,validation/{analytical,carmaker}}
├── scripts/
├── docs/
├── .github/workflows/build.yml
├── CMakeLists.txt
├── LICENSE                          # Apache 2.0
├── README.md
├── .clang-format, .editorconfig, .gitignore
```

### Build stack

| 도구 | 버전 | 위치 |
|---|---|---|
| CMake | 3.16.3 (시스템) — 권장 ≥ 3.20 | `CMakeLists.txt` |
| Ninja | 1.11.1 | — |
| Compiler | g++ 9.4.0, C++17 | — |
| Eigen | 3.4.0 | FetchContent |
| yaml-cpp | 0.8.0 | FetchContent |
| spdlog | 1.13.0 | FetchContent |
| gtest | 1.14.0 | FetchContent |

CMake `cmake_minimum_required` 은 시스템 호환 위해 3.20 → 3.16 으로 낮춤 (FetchContent 는 3.14+).

### CI workflow (`.github/workflows/build.yml`)

| 항목 | 값 |
|---|---|
| Runner | ubuntu-20.04 (CARLA 0.9.16 host OS) |
| Compiler matrix | gcc-9, clang-10 |
| Build type matrix | Debug, Release |
| Step | apt install → cache `_deps` → cmake → build → ctest |
| `fail-fast` | false |

### Push log 정리

`push.log` 는 `master` 가 remote 보다 뒤처져 있을 때의 과거 오류 로그. `git fetch` 결과 `origin/main == local main` (`6a4d34b`) 으로 sync 상태였음 → 파일 삭제.

### License

Apache 2.0, copyright "2026 Jiwon Seok". 사용자 결정: core 는 Apache 2.0, integration/plugin 은 별도 상용.

## 3. 검증 방법 (근거)

W1-W2 가이드의 마일스톤 통과 기준:
1. `cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Debug` 성공
2. `cmake --build build -j` 모든 target 빌드
3. `ctest --output-on-failure` 모든 test pass
4. GitHub 에 push 성공

## 4. 검증 결과

### Build & test

```
[48/48] Linking CXX executable bin/vdsim_unit_tests
100% tests passed, 0 tests failed out of 2
Total Test time (real) = 0.00 sec
```

| 단계 | 결과 |
|---|---|
| Configure (FetchContent 4 deps) | success, ~5 min |
| Build (vdsim_core + unit tests) | 48/48 targets |
| ctest (VersionTest) | 2/2 pass |
| `git push origin main` | success (`6a4d34b..d267221`) |

### Repo 통계 (첫 commit)

| 항목 | 값 |
|---|---|
| 파일 변경 | 21 |
| 추가 줄 | ~520 |
| 디렉토리 | 14 (8 빈 — `.gitkeep`) |

## 5. 판단

- 결과: **pass**
- 근거: 빌드 + test + push 모두 성공. 후속 모든 task 의 기반.
- Follow-up:
  - 시스템 cmake 가 3.16 → CARLA UE4 빌드는 별 문제 없으나 향후 cmake 신기능 사용 시 업그레이드 검토.
  - clang-10 CI 매트릭스 첫 실행은 GitHub 에서 확인 필요 (local 에서는 clang++ 미설치).
  - LICENSE copyright 가 "Jiwon Seok" 개인. 향후 VDSim 법인화 시 양도.
