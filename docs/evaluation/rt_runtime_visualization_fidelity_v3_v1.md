# RT Runtime Visualization Fidelity V3 Contract (`rt_runtime_visualization_fidelity_v3_v1`)

**Phase:** PLAN-RT-V3 — runtime visualization fidelity planning  
**Authority:** [rt_v3_runtime_visualization_fidelity_plan.md](../platform/rt_v3_runtime_visualization_fidelity_plan.md)  
**Supplements:** [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md), [rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md), [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md)  
**F5b reference:** [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md) (labels only — not redefined)

Normative contract for the third RT visualization planning wave: unified layer registry, environment readability polish, visibility overlays, grouped sensor/occlusion cognition, and performance budget. **Explanatory only** — not replay, sensor, or command authority.

---

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Scope | `platform/rt-sandbox-ui/` only |
| Authority | Bridge entity registry remains command truth; all V3 layers affect **display** or **cognition** only |
| Gazebo | Flat world unchanged ([rt_gazebo_visual_fidelity_v1.md](rt_gazebo_visual_fidelity_v1.md)) |
| SA | No SA viewer or replay bundle changes |
| New overlays | Default **off** unless noted as V2 carry-forward default-on |
| Telemetry | No new bridge pull channels or subscription types |

**Display offset (unchanged from V2):** `display_z = registry_z + sampleTerrainHeight(x, y) * exaggeration`. Spawn/move commands use registry `z` only.

---

## 1. Unified visual layer registry (`rt_visual_layer_registry_v3`)

### 1.1 Schema

| Field | Rule |
|-------|------|
| `schema` | `rt_visual_layer_registry_v3` |
| `layers` | Ordered array of layer descriptors |
| `performance_budget` | Optional caps (see §6) |

### 1.2 Layer descriptor

| Field | Type | Rule |
|-------|------|------|
| `layer_id` | string | Stable id; snake_case |
| `label` | string | User-facing toggle label |
| `z_order` | int | Lower draws beneath higher (ground → overlays → labels) |
| `default_on` | bool | V3-new layers default **false** |
| `mutual_exclusion_group` | string? | At most one layer per group active when group policy `single` |
| `module_anchor` | string | Implementation file stem (PLAT advisory) |
| `cognition_kind` | enum | `terrain` \| `visibility` \| `sensor` \| `marker` \| `none` |
| `disclaimer` | string | Required when `cognition_kind` ≠ `none` |

### 1.3 Canonical layer map (frozen baseline + V3)

| `layer_id` | Source wave | `default_on` | `module_anchor` |
|------------|-------------|--------------|-----------------|
| `terrain_mesh` | V2 | true | `terrainMeshLayer` |
| `ridge_overlays` | V2 | true | `terrainOverlays` |
| `elevation_bands` | F4 | true when ridges on | `terrainOverlays` |
| `contour_overlays` | F4 | **false** | `terrainContourLayer` |
| `vegetation_markers` | F4 | false | `environmentMarkers` |
| `occlusion_markers` | V2 | false | `environmentMarkers` |
| `sensor_domes` | V2 | false | `sensorDomeLayer` |
| `visibility_wedge_v3` | V3 | **false** | `visibilityWedgeLayer` (PLAT) |
| `horizon_hint_v3` | V3 | **false** | `horizonHintLayer` (PLAT) |
| `stacked_los_v3` | V3 | **false** | `stackedLosPresentation` (PLAT) |
| `entity_markers` | V1 | true | `entityMarkers` |
| `bounds_vertical` | V1 | per V1 toggle | `boundsLayer` |

PLAT-RT-V3 P0 must not change frozen default-on for V1/V2/F4 rows without a new PLAN wave.

---

## 2. Richer environment visuals (readability polish)

Additive presentation rules on frozen fictional terrain — **not** new heightmap sources or Gazebo SDF.

| Topic | V3 rule |
|-------|---------|
| Ridge label stacking | Offset duplicate ridge labels ≥ 12 px screen space; fade when > 3 overlap |
| Elevation band legend | Compact band key when ridges on; max 4 bands visible in legend rail |
| Contour density | When `contour_overlays` on, max 8 major contours in view frustum |
| Contrast | Bounds/markers maintain WCAG-ish contrast on dark globe baseline |
| Fixture | Still `rt_fictional_terrain_v1` — no new schema version in PLAN wave |

---

## 3. Runtime visibility overlays (V3-new, default off)

### 3.1 `visibility_wedge_v3`

| Field | Rule |
|-------|------|
| Geometry | Fan from selected entity or map click; azimuth ± configurable (default ±30°) |
| Length | Clamped to bounds diagonal / 2 |
| Label | **Heuristic visibility wedge — not sensor coverage** |
| Data source | UI-local; optional heuristic using F4 LOS segment when entity selected |

### 3.2 `horizon_hint_v3`

| Field | Rule |
|-------|------|
| Geometry | Great-circle or local horizon line at bounds edge |
| Label | **Fictional horizon cue — not terrain survey** |
| Mutual exclusion | Group `horizon_context` with wedge optional co-display policy `allow_both` |

### 3.3 `stacked_los_v3`

| Field | Rule |
|-------|------|
| Behavior | When F4 LOS segment active, show contour + LOS polyline + wedge with consistent color tokens |
| Label stack | Retain F4 “explanatory contour — not survey data” when contours visible |
| Forbidden lexicon | No “detected”, “tracked”, “cleared”, “neutralized” |

---

## 4. Sensor / occlusion cognition (grouped taxonomy)

### 4.1 Cognition strip groups

| Group id | Includes layers | Strip section title |
|----------|-----------------|---------------------|
| `terrain_context` | mesh, ridges, bands, contours | Terrain (explanatory) |
| `visibility_context` | wedge, horizon, stacked LOS | Visibility (heuristic) |
| `sensor_context` | domes, occlusion markers | Sensor context (nominal) |

### 4.2 Coexistence with F5b

| Label (F5b) | V3 strip behavior |
|-------------|-------------------|
| `truth_attested` | Show only when `enable_fidelity_coupling=true`; never on wedge/dome alone |
| `explanatory` | Default for all V3-new overlays and F4 heuristics |
| `command_authoritative` | Registry pose line only — not on overlay graphics |

When both `display_agl_m` (explanatory) and `sim_agl_m` (truth_attested) present, strip shows dual line per [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md) §2 — V3 does not merge into one unlabeled value.

### 4.3 Dome + occlusion rollup line

Single hub line when any sensor_context layer on:

`Sensor context: {n_domes} nominal domes · {n_occlusion} occlusion markers (explanatory)`

---

## 5. Visual fidelity polish (extends V1)

| Topic | V3 rule |
|-------|---------|
| Marker LOD | Beyond 40 entities in view, reduce label font 1 step; never hide selected entity label |
| Bounds depth cues | Optional corner z tick labels when `bounds_vertical` on (V1) |
| Session accent | Extend [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md) §5 — inactive slot markers at 55% opacity on globe (PLAT) |
| Command ghost | Unchanged V1 labeling `cmd` |

---

## 6. Performance budget (advisory)

Recorded at C2 freeze: production JS bundle ~455 KB (~127 KB gzip). V3 PLAT must respect:

| Cap | Value | Enforcement |
|-----|-------|-------------|
| Max simultaneous overlay layers | 6 | Registry policy warns in cognition strip |
| Max Cesium entities (markers + decor) | 120 | Soft clamp; selected entity exempt |
| Max wedge/LOS polylines per session | 4 | UI-local |
| Build regression | tier0-rt-ui | No new mandatory Ion assets |

---

## 7. Governance

| Item | Rule |
|------|------|
| Banners | Additive `BANNER_VISIBILITY_V3` when any V3 visibility overlay on |
| Frozen banners | T1/T4/F4/F5b/SA2 strings unchanged |
| Forbidden lexicon | [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) |

---

## 8. Explicit non-goals

- Bridge or ROS changes; new telemetry channels
- SA replay terrain; auto-import; federation
- Tactical overlays; operational C2/HITL semantics
- Parser/topic changes; Cesium Ion
- Re-defining F5b coupling or F4 default-off policy

---

## Appendix — Implementation anchors (PLAT advisory)

| Area | Path |
|------|------|
| Cesium layers | `platform/rt-sandbox-ui/src/cesium/` |
| Terrain cognition | `terrainCognition.ts`, `TerrainCognitionStrip.tsx` |
| Fidelity cognition | `fidelity/fidelityCognition.ts`, `FidelityTruthCognitionStrip.tsx` |
| Fixture | `fixtures/rt_visualization/v3_layer_registry_example.json` |

---

## Related

- [rt_cesium_workstation_visualization_v3_v1.md](rt_cesium_workstation_visualization_v3_v1.md)
- [rt_v3_visualization_realism_review_r1.md](rt_v3_visualization_realism_review_r1.md)
