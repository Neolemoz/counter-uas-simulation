# RT Runtime Realism Expansion Contract (`rt_runtime_realism_expansion_v1`)

**Phase:** PLAN-RT-F4 / PLAT-RT-F4  
**Authority:** [rt_f4_runtime_realism_expansion_plan.md](../platform/rt_f4_runtime_realism_expansion_plan.md)  
**Supplements:** [rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md) (V2 frozen; F4 is additive)

Explanatory environment and visualization expansion for the RT interactive sandbox. Not replay, sensor, or tracker truth.

---

## 1. Core invariant

| Rule | Detail |
|------|--------|
| Scope | `platform/rt-sandbox-ui/` only |
| Authority | Bridge entity registry remains command truth; terrain affects **display** only |
| Gazebo | Flat world unchanged |
| SA | No SA viewer or replay bundle changes |

---

## 2. Extended terrain profile (additive fields)

Existing `schema: rt_fictional_terrain_v1` fixture may add:

| Field | Rule |
|-------|------|
| `elevation_bands_m` | Optional height levels for band coloring (e.g. 10, 20, 30, 40) |
| `vegetation_markers` | `{ marker_id, position_enu_m, kind, label }` — fictional vegetation cues |
| Additional `ridge_features` | Named polylines for cognition labels |
| Additional `occlusion_markers` | Static explanatory landmarks |

**Display offset:** unchanged from V2 — `display_z = registry_z + sampleTerrainHeight(x,y) * exaggeration`.

---

## 3. Visual layers (F4 defaults)

| Layer | Default | Labeling |
|-------|---------|----------|
| Terrain mesh | on (V2) | Fictional heightmap |
| Ridge overlays | on (V2) | Named ridges + elevation bands when ridges on |
| Contour overlays | **off** | “Explanatory contour — not survey data” |
| Vegetation markers | **off** | “Fictional vegetation cue” |
| Occlusion markers | **off** (V2) | “Occlusion marker (explanatory)” |
| Sensor domes | **off** (V2) | “Nominal dome — not coverage proof” |

---

## 4. Cognition (explanatory only)

| Signal | Rule |
|--------|------|
| Terrain relation | `terrain_m`, `registry_z_m`, `display_agl_m`, nearest ridge, contour level under entity |
| LOS segment | `clear` / `terrain_blocked` / `marker_occluded` — heuristic only |
| Visibility hint | Plain language (e.g. ridge-masked heuristic); never “detected” / “tracked” |
| Sensor dome | Entity count in nominal horizontal range; enhanced context labels |

---

## 5. Camera presets (local only)

V2 presets retained. F4 may add `valleyFloor`, `crestLine` — no bridge commands; no persistence.

Optional viewer-only LOS segment polyline when entity pair selected — not bridge geometry.

---

## 6. Governance

| Item | Rule |
|------|------|
| Banners | Additive `BANNER_REALISM_F4` when contour/vegetation on; frozen T1/T3/T5 strings unchanged |
| Forbidden lexicon | [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) |

---

## 7. Explicit non-goals

- SA replay terrain; federation; auto-import
- Parser/topic changes; tactical authority changes
- Gazebo runtime behavior; sensor truth coupling
- Scrubber sync; Cesium Ion; M3 distributed runtime
- Workflow / capture / handoff changes

---

## Related

- [rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md)
- [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md)
- [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md)
