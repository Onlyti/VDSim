# Component presets (v0.3)

Ride presets for workshop UI. Runtime vehicle assembly uses the **parts catalog**
(`configs/parts/`, `configs/blueprints/`). See `docs/design/PARTS_CATALOG.md`.

| Slot | Fleet field | Catalog |
|------|-------------|---------|
| Assembly | `blueprint` | `configs/blueprints/*.yaml` |
| Tire override | `parts.tire` | `tire.*` part id |
| Front kin (L3) | `parts.front_susp_kin` | `susp.*` → `parts/susp_kinematics/kin/` |
| Rear kin (L3) | `parts.rear_susp_kin` | same |
