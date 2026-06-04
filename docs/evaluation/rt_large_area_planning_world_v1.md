# RT Large-Area Planning World V1

**Wave:** Large-Area Planning World (Steps 1–7)  
**Status:** frozen  
**Parent freezes:** [rt_planning_mode_evolution_v1.md](rt_planning_mode_evolution_v1.md), [rt_planning_terrain_visualization_v1.md](rt_planning_terrain_visualization_v1.md), [planning_mc_integration_v2.md](planning_mc_integration_v2.md)  
**Governance audit:** [rt_planning_large_area_governance_audit_v1.md](rt_planning_large_area_governance_audit_v1.md)

---

## Architecture summary

Large-Area Planning World extends RT Planning Mode with selectable **Planning extents** (5 km / 10 km / 20 km circular map-planner boundaries) that coexist with the fixed ±500 m runtime sandbox.

All surfaces remain **Planning-only**, **UI-local**, and **non-authoritative**:

- Browser React state in `AppWorkstationSlots.tsx`
- Cesium display overlays synced from `CesiumRuntimeView.tsx` when `workspaceMode === "planning"`
- Export artifacts (`planningMcSnapshot`, `planningMcPackage`, `planningMcResultLink`) with provenance `authority: "rt_planning_ui"`

Grid Mode remains default. Runtime entity editing is disabled while planning draw/radar/measure tools capture Cesium clicks.

### Module map

| Concern | Module |
|---------|--------|
| Extent registry | `platform/rt-sandbox-ui/src/cesium/planningWorld.ts` |
| Extent overlay | `platform/rt-sandbox-ui/src/cesium/planningExtentLayer.ts` |
| Cognition / warnings | `platform/rt-sandbox-ui/src/cesium/planningCognition.ts` |
| Measurements | `platform/rt-sandbox-ui/src/cesium/planningMeasurements.ts` |
| Drawing / radar / coverage | `platform/rt-sandbox-ui/src/cesium/planningDrawing.ts`, `planningCoverageAnalysis.ts` |
| Camera fit | `platform/rt-sandbox-ui/src/cesium/cameraHelpers.ts` (`flyToPlanningExtent`) |
| Snapshot / package | `platform/rt-sandbox-ui/src/layout/planningMcSnapshot.ts`, `planningMcPackage.ts` |
| Workstation UI | `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx` |

---

## Planning extent model

Three UI-local presets in `PLANNING_EXTENTS`:

| `planning_extent_id` | `planning_extent_radius_m` | `planning_extent_label` |
|----------------------|----------------------------|-------------------------|
| `planning_5km` | 5_000 | 5 km Planning World |
| `planning_10km` | 10_000 | 10 km Planning World (default) |
| `planning_20km` | 20_000 | 20 km Planning World |

Planning coordinates may lie outside runtime `WORLD_BOUNDS` (±500 m) when inside the selected Planning extent. A UI guardrail distinguishes runtime sandbox from Planning World.

Runtime sandbox bounds are **not** resized by extent selection.

---

## Extent metadata

Exported via `planningExtentMetadata()`:

```typescript
{
  planning_extent_id: PlanningExtentId;
  planning_extent_radius_m: number;
  planning_extent_label: string;
}
```

**Informational only.** Used in:

- Planning toolbar and cognition panel
- Cesium extent ring label
- `PlanningMcSnapshotV1.planning_extent`
- `PlanningMcPackageV1.planning_extent` (copied from snapshot)
- Snapshot id canonical input (`planning_extent_id`)

Governance copy: `PLANNING_EXTENT_GOVERNANCE_COPY` — not runtime authority, bridge bounds, or MC execution authority.

---

## Planning overlays

When Planning Mode is active, Cesium syncs:

| Overlay | Entity prefix | Purpose |
|---------|---------------|---------|
| Extent boundary ring | `rt-planning-extent-*` | Dashed cyan circle + planning-only labels |
| Defense polygon | `rt-planning-defense-area-*` | Draft/completed vertices (from PLAT-RT-PLAN1) |
| Radar sites | `rt-planning-*` | Display-only range rings |
| Coverage cells | planning drawing layer | Heuristic 2D estimate (PLAT-RT-COVERAGE-V2) |
| Measurements | `rt-planning-measurement-*` | Distance line, bearing label, radius ring |

Extent overlay uses scale-aware dash length and width at 20 km for readability.

---

## Cognition panel

`planning-cognition-panel` in Planning Mode displays:

- Selected extent label and radius
- Polygon count (completed)
- Radar site count
- Measurement count (completed pairs)
- Per-extent guidance (compact / standard / wide Planning World)
- Informational warnings (no polygon, no radars, incomplete measurement, draft in progress, coordinates outside extent)

Governance copy: `PLANNING_COGNITION_GOVERNANCE_COPY` — UI-local, non-authoritative.

Warnings do not block actions.

---

## Measurement tools

Planning-only distance, bearing, and radius readouts:

- Tool: `measure_distance` in Planning toolbar
- Two-click distance + cardinal bearing
- Configurable radius display ring (1 km / 3 km / 5 km)
- Coordinate readouts in local X/Y meters

Governance copy: `PLANNING_MEASUREMENT_GOVERNANCE_COPY` — not runtime authority, sensor truth, or MC execution authority.

---

## Governance boundaries

Explicit denials (verified in Step 6 audit):

| Boundary | Status |
|----------|--------|
| Runtime mutation | **None** — no spawn, move, delete, or `apply_scenario` |
| Bridge coupling | **None** — no bridge command dispatch from planning paths |
| MC execution | **None** — export-only snapshots/packages; `mc_preparation` is suggested maintainer hint |
| LOS logic | **None** — no wedge/horizon/stacked LOS imports |
| Sensor logic | **None** — planning radar presets are display ranges; coverage is 2D heuristic |

Acceptable read-only references:

- `WORLD_BOUNDS` for runtime/planning distinction guardrails
- Shared Cesium viewer with mode-gated entity id prefixes

Workstation isolation: `cesiumEntityEditingEnabled = effectiveEditingEnabled && !planningCesiumClickEnabled`.

---

## Limitations

- Coverage is a visual 2D heuristic — no terrain masking, LOS, sensor truth, or PoD
- Planning geometry fingerprint excludes extent; packages flag stale on extent or geometry change
- No persistence of planning state across sessions (browser memory only)
- MC package preview does not create or run jobs
- Result link import is metadata mirror only
- Large-area coordinates outside ±500 m are valid for planning review only, not runtime placement

---

## Future runtime-world considerations

Forbidden without separate PLAN + freeze + governance audit:

- Using Planning extent as bridge validation boundary
- Writing planning coordinates to entity registry or scenario payloads
- Auto MC job creation from Planning package
- Importing Planning snapshots into SA replay as authoritative geometry
- Enabling LOS/sensor truth in Planning coverage overlays
- Expanding runtime `WORLD_BOUNDS` without paired planning guardrail review

---

## Regression coverage review

Recorded at freeze (June 2026):

| Surface | Test file(s) | Status |
|---------|--------------|--------|
| Extent registry | `planningWorld.test.ts` | ✓ 3 tests |
| Extent overlay | `planningExtentLayer.test.ts` | ✓ 4 tests |
| Cognition / warnings | `planningCognition.test.ts` | ✓ 7 tests |
| Measurements | `planningMeasurements.test.ts` | ✓ 6 tests |
| Drawing / coverage | `planningDrawing.test.ts`, `planningCoverageAnalysis.test.ts` | ✓ 12 tests |
| Snapshot extent propagation | `planningMcSnapshot.test.ts` | ✓ 6 tests (incl. 5 km / 20 km) |
| Package extent propagation | `planningMcPackage.test.ts` | ✓ 7 tests |
| Workstation panel | `AppWorkstationSlots.test.tsx` | ✓ 37 tests (cognition, warnings, guardrails) |

**Aggregate:** 676 tests passed (`npm test`), build clean (`npm run build`).

### Extent validation matrix

| Capability | 5 km | 10 km | 20 km |
|------------|------|-------|-------|
| Extent overlay | ✓ | ✓ | ✓ |
| Polygon drawing | ✓ | ✓ | ✓ |
| Radar placement | ✓ | ✓ | ✓ |
| Measurements | ✓ | ✓ | ✓ |
| Cognition summary | ✓ | ✓ | ✓ |
| Camera fit height | 7_250 m | 14_500 m | 29_000 m |
| Snapshot/package metadata | ✓ | ✓ (default) | ✓ |

---

## Governance summary

**Planning-only.** All Large-Area Planning World artifacts are explanatory browser UI state and export JSON.

**UI-local.** No bridge HTTP, no runtime registry writes, no Gazebo/ROS coupling.

**Non-authoritative.** Extent metadata, measurements, coverage, and MC package fields are not runtime, sensor, or MC execution authority.

**No runtime mutation, bridge coupling, MC execution, LOS logic, or sensor logic.**

---

## Freeze verdict

| Dimension | Verdict |
|-----------|---------|
| Planning-only boundary | **Pass — frozen** |
| UI-local isolation | **Pass — frozen** |
| Governance audit (Step 6) | **Pass** |
| Regression coverage | **Pass** |
| Violations | **None** |

**PLAT-RT-PLAN-LARGE1 frozen.** Hold per PAUSE-RT-PLATEAU-V13; no expansion without scoped plan and maintainer pain signal.
