# VDSim 핸드오프

작성: 2026-06-06. **v0.3 M4** catalog API + scene v3. M5 = docs sweep.

## 1. 목표
v0.3 parts catalog + scene-based runtime. `docs/design/PARTS_CATALOG.md`.

## 2. 현재 상태
- **M1–M4** 완료. `main` pushed (`ce8b446` + M4 local).
- **ctest 195/195** (`catalog_api` 추가 후).
- CLI: `vdsim_realtime --scene=` only.

## 3. v0.3 진행
| Phase | 상태 | 내용 |
|-------|------|------|
| M1–M3 | 완료 | catalog, migrate, `--scene=` |
| **M4** | **완료** | `/api/catalog`, `/api/scene`, simconfig v3 |
| M5 | 대기 | docs sweep, `tools/import_part_pack` stub |

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

## 5. 주의
- GUI 시각검증 = 유저. TUR confidential. ISO 리베이스 금지 until drivetrain.

## 6. 경로
- `gui/runner/catalog_api.py`, `gui/api/routes.py`
- `python/catalog/resolver.py` (`list_blueprints`)
