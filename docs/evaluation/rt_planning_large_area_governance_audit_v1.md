# RT Planning Large-Area World — Governance Boundary Audit V1

**Phase:** Large-Area Planning World (Steps 1–6)  
**Wave:** Step 6 — Governance Boundary Audit  
**Status:** audit complete (docs only; no PLAT changes authorized by this audit)

**Prior context:** [rt_planning_mode_evolution_v1.md](rt_planning_mode_evolution_v1.md), [planning_mc_integration_v2.md](planning_mc_integration_v2.md)

---

## Audit scope

Large-Area Planning World artifacts introduced or extended in Steps 1–5:

| Layer | Modules |
|-------|---------|
| Extent registry | `planningWorld.ts` |
| Extent overlay | `planningExtentLayer.ts` |
| Cognition / UX | `planningCognition.ts` |
| Measurements | `planningMeasurements.ts` |
| Drawing / radar | `planningDrawing.ts` |
| Coverage heuristic | `planningCoverageAnalysis.ts` |
| Locations / camera | `planningLocations.ts`, `cameraHelpers.ts` (`flyToPlanningExtent`) |
| Cesium sync | `CesiumRuntimeView.tsx`, `CesiumRuntimePanel.tsx` |
| Snapshot / package / result link | `planningMcSnapshot.ts`, `planningMcPackage.ts`, `planningMcResultLink.ts` |
| Workstation shell | `AppWorkstationSlots.tsx` |

**Out of scope for this audit:** runtime bridge, Gazebo/ROS, MC execution CLI (`rt_layout_mc_execute.py`), SA replay viewer, sensor/LOS runtime modules.

---

## 1. Boundary audit

### Verdict: **Pass** — no authority violations found

All audited artifacts remain **Planning-only**, **UI-local**, and **non-authoritative**.

| Boundary | Finding |
|----------|---------|
| Runtime `WORLD_BOUNDS` | Read-only reference in `planningExtentLayer.ts` (`isInsideRuntimeBounds`) and `planningMeasurements.ts` (`planningMeasurementsPreserveRuntimeBounds`). Used for explanatory guardrails and label placement only; **never written**. |
| Bridge validation | No imports from `@/bridge/*` in planning modules except `AppWorkstationSlots.tsx` type import (`CaptureHandoffRow`) unrelated to planning state. No bridge command dispatch from planning paths. |
| Runtime entities | Planning radar sites are `PlanningRadarSite` records with `rt-planning-*` Cesium entity ids. No `entity_id`, spawn, move, or delete bridge paths. |
| Runtime overlays | Grid-mode sensor domes, defense zones, and tactical overlays are independent. Planning layers use distinct entity id prefixes (`rt-planning-*`, `rt-planning-extent-*`, `rt-planning-measurement-*`). |
| MC execution | Snapshots/packages/result links export JSON only. `mc_preparation` fields are suggested maintainer hints; UI copy states no MC execution. |
| Result execution | `planningMcResultLink.ts` imports metadata only; no filesystem or job runner invocation. |
| Sensor logic | Planning radar presets (`PLANNING_RADAR_PRESETS`) are UI-local display ranges. Coverage uses 2D circle overlap heuristic in `planningCoverageAnalysis.ts`; no runtime sensor dome or fidelity truth coupling. |
| LOS logic | No LOS imports or wedge/horizon modules referenced from planning code paths. |

### Workstation isolation mechanisms

- `workspaceMode === "planning"` gates Planning UI and Cesium planning layer sync.
- `cesiumEntityEditingEnabled = effectiveEditingEnabled && !planningCesiumClickEnabled` prevents runtime entity drag/spawn while planning draw/radar/measure tools are active.
- Grid Mode remains default (`DEFAULT_RUNTIME_WORKSPACE_MODE = "grid"`).

### Governance copy inventory (informational only)

| Constant | Module | Key denial |
|----------|--------|------------|
| `PLANNING_EXTENT_GOVERNANCE_COPY` | `planningWorld.ts` | not runtime authority, bridge bounds, MC execution authority |
| `PLANNING_EXTENT_LAYER_COPY` | `planningExtentLayer.ts` | not runtime bounds, bridge authority, MC execution authority |
| `PLANNING_MEASUREMENT_GOVERNANCE_COPY` | `planningMeasurements.ts` | not runtime authority, sensor truth, MC execution authority |
| `PLANNING_COGNITION_GOVERNANCE_COPY` | `planningCognition.ts` | UI-local, non-authoritative |
| `PLANNING_LOCATION_GOVERNANCE_COPY` | `planningLocations.ts` | camera jump only; no runtime mutation |
| Snapshot provenance | `planningMcSnapshot.ts` | `authority: "rt_planning_ui"` |

---

## 2. Metadata audit

### `planning_extent_id`

| Value | Radius | Label |
|-------|--------|-------|
| `planning_5km` | 5_000 m | 5 km Planning World |
| `planning_10km` | 10_000 m | 10 km Planning World |
| `planning_20km` | 20_000 m | 20 km Planning World |

**Verdict:** Informational UI-local enum only. Default `planning_10km`. Not referenced by bridge, runtime bounds, or parser contracts.

### `planning_extent_radius_m`

Circular Planning World radius in meters. Used for:

- Extent overlay ring geometry
- Cognition summary display
- Camera fit height (`radius × 1.45`)
- Snapshot/package metadata propagation

**Verdict:** Display and export metadata only. Does not resize runtime sandbox (±500 m unchanged).

### `planning_extent_label`

Human-readable Planning World name. Appears in UI, Cesium extent label, snapshot/package JSON.

**Verdict:** Presentation only. `planningExtentMetadata()` returns a stable subset without side effects.

---

## 3. Snapshot / package audit

### Extent propagation

| Artifact | Extent field | Propagation |
|----------|--------------|-------------|
| `PlanningMcSnapshotV1` | `planning_extent` | Full metadata via `planningExtentMetadata()` |
| Snapshot canonical hash | `planning_extent_id` | Included in `planning_snapshot_id` derivation |
| `PlanningMcPackageV1` | `planning_extent` | Copied from snapshot unchanged |
| Package stale detection | extent id compare | UI flags stale when extent or geometry changes |

**Verdict:** Extents propagate correctly through snapshot → package → result link preview.

### Authority semantics

| Field | Authority implied? | Actual role |
|-------|-------------------|-------------|
| `provenance.authority` | `"rt_planning_ui"` | Explicit non-runtime source label |
| `planning_geometry_id` | Planning geometry fingerprint | Polygon + radar geometry only (extent-independent by design) |
| `planning_snapshot_id` | Snapshot identity | Includes extent id in canonical input |
| `mc_preparation.*` | Could be misread as job authority | UI-labeled suggested settings only; no job creation |

**Verdict:** No runtime or MC authority implied when governance copy and provenance fields are read together. Package preview UI includes explicit “no MC execution” disclaimer.

### Regression evidence

Existing tests confirm isolation:

- `planningMcSnapshot.test.ts` — extent preservation, no bridge/MC/runtime
- `planningMcPackage.test.ts` — extent propagation, no `geometry_id`/`layout_id` replacement
- `planningMcResultLink.test.ts` — no execution paths
- Per-module boundary tests in `planningWorld`, `planningExtentLayer`, `planningMeasurements`, `planningCognition`, `planningDrawing`, `planningCoverageAnalysis`

---

## 4. Validation matrix

Capabilities verified at each Planning extent (UI-local; same tool semantics; scale differs):

| Capability | 5 km | 10 km | 20 km |
|------------|------|-------|-------|
| Extent overlay readable | ✓ | ✓ | ✓ (scale-aware dash/width) |
| Polygon drawing | ✓ | ✓ | ✓ |
| Radar placement | ✓ | ✓ | ✓ |
| Measurement tools | ✓ | ✓ | ✓ |
| Cognition summary | ✓ | ✓ | ✓ |
| Camera fit (`flyToPlanningExtent`) | 7_250 m | 14_500 m | 29_000 m |
| Coordinates outside runtime bounds allowed | ✓ (inside extent) | ✓ | ✓ |
| Runtime guardrail when outside ±500 m | ✓ | ✓ | ✓ |

Test coverage:

- `planningCognition.test.ts` — `validateAllPlanningExtents()`, extent switch summary
- `planningExtentLayer.test.ts` — overlay metadata, runtime bounds unchanged
- `planningMcSnapshot.test.ts` / `planningMcPackage.test.ts` — 5 km and 20 km extent metadata
- `AppWorkstationSlots.test.tsx` — cognition panel, warnings, 5 km guardrail, extent switch copy

---

## 5. Governance findings

### Remaining assumptions (acceptable)

1. **Shared Cesium viewer** — Planning overlays render in the same viewer as runtime markers when Planning Mode is active; isolation is by mode flag and entity id prefix, not separate viewer instance.
2. **`WORLD_BOUNDS` as reference** — Planning guardrails assume runtime sandbox remains ±500 m. This is read-only coupling for UX distinction, not authority expansion.
3. **Geometry vs extent fingerprint split** — `planning_geometry_id` excludes extent; extent changes invalidate package via separate stale check. Maintainers must regenerate packages after extent switch.
4. **Heuristic coverage** — Planning radar ranges resemble sensor vocabulary but are explicitly labeled heuristic/visual estimate.

### Future risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| Runtime `WORLD_BOUNDS` expansion without planning guardrail review | Medium | Require paired governance audit if sandbox bounds change |
| Auto MC job creation from Planning package | High | Forbidden without separate PLAN + freeze; current UI is export-only |
| Planning state persistence (localStorage/session) | Medium | Could blur ephemeral vs authoritative; requires explicit schema + provenance if added |
| Label drift (“radar”, “coverage”) toward operational semantics | Medium | Keep planning-only copy in cognition panel and coverage status |
| MC result import treated as layout authority | Medium | Result link is read-only metadata mirror; no auto-apply to runtime |

### Future runtime-expansion considerations

Any wave that connects Planning artifacts to runtime MUST NOT proceed under this audit. Separate governance checklist required for:

- Writing planning coordinates into entity registry or scenario payloads
- Using Planning extent as bridge validation boundary
- Executing MC from Planning package without maintainer-only CLI boundary
- Importing Planning snapshots into SA replay as authoritative geometry
- Enabling LOS/sensor truth in Planning coverage overlays

---

## 6. Audit verdict

| Dimension | Verdict |
|-----------|---------|
| Planning-only boundary | **Pass** |
| UI-local isolation | **Pass** |
| Non-authoritative metadata | **Pass** |
| Snapshot/package extent propagation | **Pass** |
| No bridge/runtime/MC/sensor/LOS coupling | **Pass** |
| Violations requiring code fix | **None** |

**Step 6 complete.** No code changes required.

---

## Regression evidence (Step 6)

Recorded June 2026:

```bash
cd platform/rt-sandbox-ui && npm test    # 676 passed
cd platform/rt-sandbox-ui && npm run build
git diff --check
```

---

## Recommended Step 7 scope

**Freeze + regression closure for Large-Area Planning World**

1. Publish freeze registry entry and short evolution doc update linking Steps 1–6.
2. Golden fixture parity check for planning snapshot/package at 5/10/20 km extents.
3. Full `npm test` + `npm run build` + governance copy lint (planning governance constants present).
4. Explicit “no PLAT expansion” hold statement aligned with PAUSE-RT-PLATEAU-V13 unless maintainer pain signal.
5. Optional: add planning large-area row to `rt_planning_mode_evolution_v1.md` cross-reference.

No runtime, bridge, MC execution, LOS, or sensor changes in Step 7.
