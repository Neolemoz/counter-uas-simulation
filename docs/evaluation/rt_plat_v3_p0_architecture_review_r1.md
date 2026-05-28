# RT-V3 P0 — Architecture Review R1 (PLAT-RT-V3 P0)

**Phase:** PLAT-RT-V3 P0 — visual layer registry foundations  
**Plan:** [rt_plat_v3_p0_visual_layer_registry_plan.md](../platform/rt_plat_v3_p0_visual_layer_registry_plan.md)  
**Contract:** [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md)  
**Freeze audit:** [rt_plat_v3_p0_freeze_audit.md](rt_plat_v3_p0_freeze_audit.md)

## Verdict

**Pass** — registry isolates toggle policy; existing Cesium sync paths unchanged.

---

## 1. Layer flow

| Layer | Role |
|-------|------|
| `visualLayerRegistry.ts` | Canonical ids, z-order, defaults, performance budget, UI grouping |
| `VisualLayerToggleRail` | Grouped toggles (terrain / sensor / markers) |
| `CesiumRuntimePanel` | Unified `VisualLayerVisibility` state |
| `terrainLayers.ts` | Existing `syncTerrainLayers` — terrain projection from registry defaults |

Bridge entity registry remains command authority. Registry affects display toggles only.

---

## 2. P0 vs P1 boundary

| Item | P0 | P1 |
|------|----|----|
| `visibility_wedge_v3` | In registry; not toggleable | Implement module |
| `horizon_hint_v3` | In registry; not toggleable | Implement module |
| `stacked_los_v3` | In registry; not toggleable | Implement module |
| Performance budget warn | Helper only | UI strip warn |

| Finding ID | Verdict |
|------------|---------|
| V3P0-ARCH-01 | Pass |

---

## 3. Default preservation

Vitest asserts `defaultVisibilityFromRegistry()` matches frozen `DEFAULT_TERRAIN_LAYERS` and panel chrome defaults (bounds, labels, vertical on).

| Finding ID | Verdict |
|------------|---------|
| V3P0-ARCH-02 | Pass |

---

## 4. Module footprint

| Module | Change |
|--------|--------|
| `visualLayerRegistry.ts` | New (~260 LOC) |
| `CesiumRuntimePanel.tsx` | Toggle block replaced by rail |
| `App.tsx` | Unchanged |

| Finding ID | Verdict |
|------------|---------|
| V3P0-ARCH-03 | Pass — no `App.tsx` split required in P0 |

---

## Recommended next

**PLAT-RT-V3 P1** — visibility overlay modules + `BANNER_VISIBILITY_V3` + hub grouping.
