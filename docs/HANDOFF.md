# VDSim 핸드오프

작성: **2026-06-08** · 브랜치 `feat/v0.4-slope-jump-m5` · **254/254 ctest green**

## 1. 문서 인덱스

| Topic | User / ops | Design spec |
|-------|------------|-------------|
| Catalog + scenes | [`CATALOG_AND_PHYSICS.md`](CATALOG_AND_PHYSICS.md) | [`design/PARTS_CATALOG.md`](design/PARTS_CATALOG.md) |
| **Ld4 multibody M1–M7** | § L4 in catalog doc | **[`design/LD4_MULTIBODY.md`](design/LD4_MULTIBODY.md)** |
| v0.4 slope / jump / loop | [`RUNNING.md`](RUNNING.md) | [`design/V0.4_SLOPE_JUMP_DYNAMICS.md`](design/V0.4_SLOPE_JUMP_DYNAMICS.md) |
| v0.5 terrain + L5 | — | [`design/V0.5_TERRAIN_L5.md`](design/V0.5_TERRAIN_L5.md) |
| LuGre tire | theory ch.19 | [`design/V0.2_TIRE_LUGRE.md`](design/V0.2_TIRE_LUGRE.md) |
| Roadmap | [`ROADMAP.md`](ROADMAP.md) | [`design/TIRE_ROADMAP.md`](design/TIRE_ROADMAP.md) |
| Validation | [`VALIDATION.md`](VALIDATION.md) | — |

## 2. 브랜치 스냅샷

### v0.4 stunt (M0–M5) — shipped

- `create_stunt_dof()` → `Free3DDynamics` (`free_3d_dynamics.cpp`): pos+quat+ω, 4-wheel MF, no rail snap
- `create_legacy_stunt_dof()` → L3+L2 rail / loop CG snap (`Stunt.VerticalLoopCompletesLap`)
- Scenes: `jump_ramp_demo.yaml`, `vertical_loop_demo.yaml`; cosim `stunt:` → `make_ground` (ramp/loop)
- GUI: stunt mesh, **z** / pitch / roll telemetry; `settle_spawn_on_ground`
- LuGre default retune for L5 loop (`sigma0` 9e4, `sigma2` 75)

### Ld4 multibody (M1–M7) — shipped

상세: [`design/LD4_MULTIBODY.md`](design/LD4_MULTIBODY.md)

| M | Runtime | Summary |
|---|---------|---------|
| M1 | L4 | FK attach, `create_fourteen_dof_kinematic()` |
| M2 | — | Topology graph from kin YAML |
| M3 | design | Quasi-static bushing compliance |
| **M4** | **L4 step** | Hard-joint DAE (TA/MP/DW/5-link) + Baumgarte travel |
| M5 | GUI/py | K&C sweep charts |
| M6 | CLI/py | Adams CSV import x-check (5 gains, 5% rtol) |
| M7 | offline | Lumped 1-DOF revolute RNEA (TA/MP/DW/5-link) |

### GUI / cosim (v0.2 carry-over)

- Vehicle Edit: Chassis / Tire / Suspension tabs; FL FR RL RR
- VDS1 protocol **v5** (`throttle_applied`, `brake_applied`)
- L4 in `make_dyn("L4")`, catalog, `l4_sedan_kinematics.yaml`

## 3. 빌드·검증

```bash
cmake --build build -j
cd build && ctest --output-on-failure   # 254/254
python3 gui/server.py                   # http://127.0.0.1:8080
```

## 4. 다음 작업

1. **v0.5 terrain + L5** — hub contact (M0) → heightmap tests → scene/GUI → tag v0.5.0
2. **ISO re-baseline** — flat only (`run_validation.py`)
3. Ld4 v0.6 — shared inertia helpers; full loop dynamics; Featherstone in step (optional)
4. `main` ← PR after review

## 5. 주의

- Hyundai (TUR) `.tir` **confidential** — 커밋 금지
- Default subsystem **숫자 리베이스 금지**
- Wheel: **FL=0, FR=1, RL=2, RR=3** · ISO 8855 RH
- Cosim plant **v5** ↔ rebuilt `vdsim_realtime`
- `configs/.resolve_cache/` — 커밋하지 않음

## 6. Git

브랜치 `feat/v0.4-slope-jump-m5` — **main 미병합**. push/merge는 명시 요청 시.
