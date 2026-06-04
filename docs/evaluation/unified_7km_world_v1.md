# Unified 7 km RT World V1

**Wave:** Unified 7 km World (Steps 1–6)  
**Status:** frozen  
**Registry ID:** PLAT-RT-UNIFIED-WORLD-7KM1  
**Supersedes (Planning authority):** [rt_large_area_planning_world_v1.md](rt_large_area_planning_world_v1.md) (PLAT-RT-PLAN-LARGE1 dual-world model)  
**Parent freezes:** PLAT-RT-S3 (entity sandbox), PLAT-RT-T3/T5 (Cesium), PLAT-RT-PLAN1, PLAT-RT-PLAN-MC2, PLAT-RT-PLAN-COMPARE1

---

## Architecture summary

RT Sandbox uses a **single unified 7 km world** for runtime authority, Cesium editing, and Planning Mode. The prior dual-world model (±500 m runtime sandbox + selectable 5/10/20 km Planning extents) is retired.

| Layer | Authority | Half-extent |
|-------|-----------|-------------|
| Bridge `WORLD_BOUNDS` | authoritative | ±7000 m x/y, z 0–200 m |
| UI mirror `bounds.ts` | read-only mirror | same |
| Planning `planning_unified_7km` | UI-local metadata only | same |
| Operational rings | display-only | see below |

**No MC execution, parser, topic, or SA coupling.** Planning snapshots remain UI-local export artifacts with provenance `authority: "rt_planning_ui"`.

### Module map

| Concern | Module |
|---------|--------|
| Bridge bounds | `platform/rt-sandbox-bridge/rt_sandbox/governance.py` |
| UI bounds mirror | `platform/rt-sandbox-ui/src/world/bounds.ts` |
| Bounds overlay | `platform/rt-sandbox-ui/src/cesium/boundsLayer.ts`, `visualStyle.ts` |
| Operational rings | `platform/rt-sandbox-ui/src/cesium/operationalRingLayer.ts` |
| Unified Planning world | `platform/rt-sandbox-ui/src/cesium/planningWorld.ts` |
| World-valid guardrails | `platform/rt-sandbox-ui/src/cesium/planningExtentLayer.ts`, `planningCognition.ts` |
| Cesium-primary editing | `AppWorkstationSlots.tsx`, `CesiumRuntimePanel.tsx`, `WorldEditingGrid.tsx` |
| Camera world-fit | `cameraHelpers.ts` (`flyToBounds`), `constants.ts` |
| Snapshot provenance | `planningMcSnapshot.ts` (`world_bounds_m` additive field) |
| Gazebo ground plane | `src/rt_sandbox_gz/worlds/rt_sandbox_flat.sdf` (14000×14000 m) |

---

## World bounds

Authoritative constants (bridge + UI mirror):

```
x: [-7000, 7000] m
y: [-7000, 7000] m
z: [0, 200] m
```

- Spawn/move clamping uses `clampPose()` against `WORLD_BOUNDS`.
- Cesium bounds overlay labels: `±7000m`.
- Default session camera height: `WORLD_FIT_CAMERA_HEIGHT_M` (7000 × 1.45 ≈ 10150 m).

---

## Operational rings (display-only)

Synced with world bounds overlay visibility. **Not** bridge validation, MC semantics, or runtime authority.

| Ring | Radius (m) | Purpose |
|------|------------|---------|
| City | 1000 | Urban core reference |
| Defense | 3000 | Defense-area cognition |
| Warning | 5000 | Warning band |
| Spawn band | 5000–7000 | Attacker spawn band (inner + outer rings) |

Governance copy: `OPERATIONAL_RING_GOVERNANCE_COPY` in `operationalRingLayer.ts`.

---

## Planning model

**Default:** `planning_unified_7km` — Unified 7 km World (±7000 m).

- Extent selector UX removed; Planning Mode operates on the unified world by default.
- Guardrails: **world-valid** vs **world-invalid** (`isInsideWorldBounds`, warning `coordinates_outside_world_bounds`).
- Cognition panel surfaces operational ring radii and unified world summary.
- Camera: **Fit Unified World** → `flyToBounds` (legacy `flyToPlanningExtent` deprecated).

### Legacy compatibility

`planning_5km`, `planning_10km`, `planning_20km` remain **readable** in imported snapshots via `planningExtentById()`. Labels suffixed `(legacy)`. Legacy extent rings render only when legacy metadata is explicitly supplied (import display). **Not** world authority.

### Snapshot metadata (additive)

New exports include optional additive field (schema `rt_planning_mc_snapshot_v1` unchanged):

```typescript
world_bounds_m?: {
  x_min_m: number; x_max_m: number;
  y_min_m: number; y_max_m: number;
  z_min_m: number; z_max_m: number;
};
```

`planning_extent` on new exports uses `planning_unified_7km`.

---

## Migration history

| Step | Wave | Scope |
|------|------|-------|
| 1 | Audit | Dual-world touchpoints; feasibility |
| 2 | Plan | Phased waves 0–4 |
| 3 | Bounds migration | Bridge/UI ±7000, Gazebo ground, maintainer viz, tests |
| 4 | Overlay + editing | Labels, camera, Cesium-primary, operational rings, copy |
| 5 | Metadata collapse | Deprecate legacy extents, remove selector, world-valid guardrails, `world_bounds_m` |
| 6 | Freeze | This document, registry, golden fixtures, validation gate |

Prior ±500 m bounds and 5/10/20 km Planning extent authority are **superseded** by this freeze.

---

## Governance audit (Step 6)

| Check | Verdict |
|-------|---------|
| Dual-world UX authority | **Clear** — single unified world; legacy IDs import-only |
| Planning vs runtime mismatch authority | **Clear** — collapsed to world-valid/invalid |
| Bridge contract changes | **Additive bounds only** — same `WORLD_BOUNDS` shape; values ±7000 (no new endpoints) |
| Parser / topic changes | **None** |
| MC execution changes | **None** |
| SA coupling | **None** |
| Snapshot schema break | **None** — additive `world_bounds_m` only |

**Residual doc debt:** Older frozen docs (e.g. `rt_cesium_runtime_ui_v1.md`, `rt_large_area_planning_world_v1.md`) still describe ±500 m or dual-world Planning. Read through **this document** and PLAT-RT-UNIFIED-WORLD-7KM1 for current authority.

---

## Compatibility path

1. **Imported snapshots** with legacy `planning_extent_id` parse and display; compare panel may warn on extent mismatch across slots.
2. **Stale package advisory** when `planning_extent_id !== planning_unified_7km` — regenerate package from current Planning geometry.
3. **Core SVG grid** remains ~1 km local inset (Cesium-primary editing policy); not a second world authority.
4. **Bridge/tests** accept poses within ±7000; reject beyond.

---

## Limitations

- Fictional terrain fixture (`rt_ridge_terrain_v1.json`) remains ~1 km ridge grid; flat/zero outside.
- Legacy SVG `gridCoords.ts` covers ~1 km inset only.
- Operational rings toggle bundled with bounds visibility (no separate layer registry entry yet).
- Historical evaluation docs may cite ±500 m until individually superseded.

---

## Future work (not authorized by this freeze)

- Tile or extend fictional terrain for full 7 km readability.
- Independent operational-ring layer toggle in visual registry v4+.
- Optional snapshot schema v2 if legacy extent fields are formally removed (requires explicit PLAN wave).
- Governance doc batch supersession for ±500 m references in frozen RT-T3/V1/V2 contracts.

---

## Validation gate

- `python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q`
- `npm test` / `npm run build` in `platform/rt-sandbox-ui`
- `git diff --check`
- Golden fixtures: `planning_layout_compare_golden_v1.json`, `live_smoke_trace_v1.json` use ±7000 bounds and `planning_unified_7km` where applicable.
