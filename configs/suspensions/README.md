# Suspension hardpoint configs

Two config families live here, for two tool generations. Check the top-level key
(`type:` vs `topology:`) to know which is which.

## 3D native solvers — key `type:`  (canonical, use these)

Full 3D hardpoint geometry consumed by the in-process native kinematics
(`create_*_native_kinematics`) and their Python equivalents
(`tools/kinematics/*_3d_solver.py`). Verified by `tests/unit/test_suspension_lookup.cpp`.

| file | `type:` | solver |
|---|---|---|
| `dw_front_sports.yaml` | `double_wishbone` | `create_dw_native_kinematics` / `dw_3d_solver.py` |
| `mp_front_sedan.yaml` | `macpherson` | `create_mp_native_kinematics` / `mp_3d_solver.py` |
| `ta_rear_sedan.yaml` | `trailing_arm` | `create_ta_native_kinematics` / `ta_3d_solver.py` |
| `5link_rear_sports.yaml` | `five_link` | `create_5link_native_kinematics` / `fivelink_3d_solver.py` |

Attach to an L3 vehicle via `attach_front_kinematics` / `attach_rear_kinematics`.

## 2D legacy side-view configs — key `topology:`

Older 2D side-view hardpoint sets used by the Stage-A analyzer
(`tools/kinematics/double_wishbone.py`) and the web suspension editor. They do
**not** drive the 3D native solvers (those throw with a pointer to this note).
Files: `double_wishbone.yaml`, `macpherson.yaml`, `trailing_arm.yaml`,
`multi_link_5.yaml`, plus the dependent-axle stubs `beam_axle.yaml`,
`twist_beam.yaml` (no native solver yet — dependent suspension is future work).
