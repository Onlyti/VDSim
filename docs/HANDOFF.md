# VDSim 핸드오프

작성: 2026-06-06. v0.3 catalog + drivetrain inertia + LuGre tire 완료.

## 1. 문서 인덱스

| Topic | User guide | Design spec |
|-------|------------|-------------|
| Catalog + scenes + CLI | [`CATALOG_AND_PHYSICS.md`](CATALOG_AND_PHYSICS.md) | [`design/PARTS_CATALOG.md`](design/PARTS_CATALOG.md) |
| GUI catalog API | § GUI in above | `PARTS_CATALOG.md` §8 |
| External packs | § External packs | `PARTS_CATALOG.md` §7 |
| Drivetrain inertia | § Drivetrain | [`design/V0.2_DRIVETRAIN.md`](design/V0.2_DRIVETRAIN.md) |
| LuGre tire | § LuGre | [`design/V0.2_TIRE_LUGRE.md`](design/V0.2_TIRE_LUGRE.md) |
| Low-speed (default tire) | — | [`design/LOW_SPEED_HANDLING.md`](design/LOW_SPEED_HANDLING.md) |
| Run modes | [`RUNNING.md`](RUNNING.md) | [`design/RUNTIME_ARCH.md`](design/RUNTIME_ARCH.md) |
| Validation | [`VALIDATION.md`](VALIDATION.md) | — |

## 2. 구현 상태 (요약)

### v0.3 catalog (M1–M5)
- `configs/catalog/`, `parts/`, `blueprints/`, `scenes/`, `maneuvers/`
- CLI: `vdsim_realtime --scene=` only
- GUI: `/api/catalog`, `/api/scene`, simconfig v3
- `tools/import_part_pack.py` stub
- Legacy `configs/vehicles|tires|scenarios/` removed

### Physics (post–v0.3)
- **Drivetrain:** `engine_rotational_inertia` + open-diff carrier coupling
- **Tire:** `TireParams.lugre` opt-in (default off)

### Tests
- **201/201** ctest green (`cd build && ctest`)

## 3. 다음
- v0.4 스턴트 (`design/V0.4_PLAN.md`)
- ISO re-baseline after drivetrain (`apps/validation/run_validation.py`)

## 4. 주의
- TUR confidential. GUI 시각검증 = 유저.
- Default subsystem behaviour unchanged until catalog parts set non-default physics.
