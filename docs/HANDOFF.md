# VDSim 핸드오프

작성: 2026-06-06. **v0.3 M3** `--scene=` CLI 완료. M4 = GUI catalog API.

## 1. 목표
v0.3 parts catalog + scene-based runtime. `docs/design/PARTS_CATALOG.md`.

## 2. 현재 상태
- **v0.2.4** 태그 = v0.2 마감.
- **M1–M3** 완료: catalog, migrate, `--scene=` only CLI.
- 빌드 `cmake --build build -j`; **ctest 194/194** (M3 `scene_materialize` 추가).
- Flat `vehicle.yaml tire.yaml` argv **제거** (`vdsim_realtime`).

## 3. v0.3 진행
| Phase | 상태 | 내용 |
|-------|------|------|
| M1 | 완료 | CatalogResolver, manifest |
| M2 | 완료 | parts/blueprints/scenes migrate |
| **M3** | **완료** | `--scene=`, `scene_loader.cpp`, `tools/materialize_scene.py` |
| M4 | 대기 | GUI `/api/catalog`, simconfig v3 |
| M5 | 대기 | docs sweep, import_part_pack stub |

## 4. CLI (M3)
```bash
# catalog scene (fleet[] + blueprint) — materialized at startup via Python
build/bin/vdsim_realtime --scene=configs/scenes/two_vehicle_race.yaml \
    --cmd-port=7001 --state-port=7002 --rate=200

# materialized world (vehicles[] paths) — GUI run_config.yaml
build/bin/vdsim_realtime --scene=runs/live/run_config.yaml ...
```
`--scenario=` = deprecated alias. Catalog scene → `tools/materialize_scene.py` (cosim 내부 호출).

## 5. 주의
- GUI 시각검증 = 유저. ctest green 유지.
- TUR tire confidential. ISO 리베이스 금지 until drivetrain.

## 6. 경로
- Cosim: `cosim/realtime_server.cpp`, `cosim/scene_loader.cpp`
- Catalog: `python/catalog/`, `tools/materialize_scene.py`
- GUI: `gui/runner/cosim_bridge.py` (항상 `--scene=`)
