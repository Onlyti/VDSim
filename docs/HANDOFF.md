# VDSim 핸드오프

작성: 2026-06-06. **v0.3 M5** docs sweep + import_part_pack stub 완료.

## 1. 목표
v0.3 parts catalog + scene-based runtime. `docs/design/PARTS_CATALOG.md`.

## 2. 현재 상태
- **M1–M5** 완료.
- **ctest 196/196** (`import_part_pack` 추가).
- CLI: `vdsim_realtime --scene=` only.

## 3. v0.3 진행
| Phase | 상태 | 내용 |
|-------|------|------|
| M1–M4 | 완료 | catalog, migrate, `--scene=`, GUI API |
| **M5** | **완료** | docs sweep, `tools/import_part_pack.py` stub |

## 4. HTTP API (M4)
| Endpoint | Role |
|----------|------|
| `GET /api/catalog` | manifest index; `?type=` `?q=` |
| `GET /api/catalog/parts` | list parts |
| `GET /api/catalog/parts/:id` | part envelope + body |
| `GET /api/catalog/blueprints/:id` | blueprint doc |
| `GET /api/scene/list` | saved scenes (`configs/scenes/`) |
| `GET /api/scene/:name` | scene YAML as JSON |
| `POST /api/scene` | import scene v3 |
| `POST /api/scene/save` | save scene (alias `/api/scenario/save`) |
| `GET /api/simconfig` | export scene **version 3** |
| `POST /api/simconfig` | import scene v3 (**v2 rejected**) |
| `GET/POST /api/runconfig` | materialized world (vehicles[] + gui) |

Legacy shims: `/api/parts/registry`, `/api/suspension/list`, `/api/scenario/list`.

## 5. External packs (M5 stub)
```bash
python3 tools/import_part_pack.py /path/to/pack          # dry-run + collision check
python3 tools/import_part_pack.py /path/to/pack --install
```
Install target: `configs/catalog/packages/<package_id>/`. Id collision with builtin = error.

## 6. 다음 (v0.3 이후)
- Drivetrain inertia (`V0.2_DRIVETRAIN.md`)
- LuGre tire (`V0.2_TIRE_LUGRE.md`)
- v0.4 스턴트 (`V0.4_PLAN.md`)

## 7. 주의
- GUI 시각검증 = 유저. TUR confidential. ISO 리베이스 금지 until drivetrain.

## 8. 경로
- `tools/import_part_pack.py`, `python/catalog/pack_import.py`
- `examples/_catalog_load.py` — catalog preset helper for examples
