# RT V2 Terrain Realism Contract (`rt_v2_terrain_realism_v1`)

**Phase:** PLAT-RT-V2 — terrain / visual realism  
**Authority:** [rt_v2_terrain_realism_plan.md](../platform/rt_v2_terrain_realism_plan.md)  
**Supplements:** [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md) (visuals only; V1 authority rules unchanged)

Additive contract for RT-local fictional terrain and environment overlays. Explanatory only — not replay or sensor truth.

---

## 1. Core invariant

| Rule | Detail |
|------|--------|
| Scope | `platform/rt-sandbox-ui/` only |
| Authority | Bridge entity registry remains command truth; terrain affects **display** only |
| Gazebo | Flat world unchanged ([rt_gazebo_visual_fidelity_v1.md](rt_gazebo_visual_fidelity_v1.md)) |
| SA | No SA viewer or replay bundle changes |

---

## 2. Terrain profile `rt_fictional_terrain_v1`

| Field | Rule |
|-------|------|
| `schema` | `rt_fictional_terrain_v1` |
| `grid_enu_m` | Bilinear `heights_m` over ±500 m world bounds |
| `ridge_features` | Named polylines for cognition labels |
| `occlusion_markers` | Static fixture positions (explanatory) |
| `caveat` | Fictional sandbox geometry — not deployed geography |

**Display offset:** `display_z = registry_z + sampleTerrainHeight(x, y) * exaggeration` on Cesium markers when terrain mesh enabled. Registry spawn/move commands use unchanged registry `z`.

---

## 3. Visual layers

| Layer | Toggle default | Notes |
|-------|----------------|-------|
| Terrain mesh | on | Coarse ground polygons from heightmap |
| Ridge overlays | on | Polylines + optional elevation bands |
| Environment markers | off | Tree/occlusion billboards |
| Sensor domes | off | 200 m nominal radius per radar — not coverage proof |

---

## 4. Camera presets (local only)

| Preset | Behavior |
|--------|----------|
| `terrainOverview` | High oblique over bounds |
| `ridgeLine` | Fly along primary ridge polyline |
| `sensorContext` | Fit selected radar + nominal dome |

No bridge commands; no `localStorage` persistence.

---

## 5. Terrain cognition (explanatory)

| Signal | Rule |
|--------|------|
| Terrain relation | `terrain_m`, `registry_z_m`, `display_agl_m`, nearest ridge |
| Occlusion | Heuristic `clear` / `terrain_blocked` / `marker_occluded` — not tracker LOS |
| Sensor dome | Entity count within horizontal nominal range when radar selected |

---

## 6. Governance

| Item | Rule |
|------|------|
| Frozen banners | T1/T3/T5 strings unchanged |
| Additive | `BANNER_TERRAIN` in Cesium panel when terrain layers on |
| Forbidden lexicon | Unchanged ([rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)) |

---

## 7. Explicit non-goals

- SA replay terrain; federation; auto-import
- Parser/topic changes; tactical authority changes
- Scrubber sync to terrain; Cesium Ion
- RT-X1 experimentation (separate wave)

---

## Related

- [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md)
- [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md)
