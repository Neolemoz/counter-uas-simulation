# RT Radar Coverage Freeze Audit

Freeze ID: `PLAT-RT-RADAR-COVERAGE1`

Status: **FROZEN**

## Architecture summary

Runtime Coverage Analysis V1 is a **UI-local, read-only** analytics and
visualization wave under `platform/rt-sandbox-ui/src/coverage/`. It adapts frozen
Planning Mode coverage algorithms to live entity mirrors without bridge,
telemetry, or planning module changes.

```
Entity mirror + protectedCenterEntityId + defenseZoneConfig + radarDomeConfig [+ tacticalState]
        ↓
buildRuntimeCoverageInputs() / analyzeRuntimeCoverage()
        ↓
analyzePlanningCoverage()  (unchanged import)
        ↓
┌─────────────────────────────────────────────────────────────┐
│ P1: deriveRuntimeCoverageStatus → RuntimeCoverageStatusStrip│
│ P2: deriveRuntimeCoverageRenderModel → Cesium + SVG overlays│
└─────────────────────────────────────────────────────────────┘
```

Visualization is gated by visual layer registry key `runtime_coverage_cells`
(`default_on: false`). Grid Mode only; Planning Mode unchanged.

## Implemented scope

| Step | Deliverable | Status |
|------|-------------|--------|
| 1 | Step 1 audit (runtime vs planning coverage gap) | complete |
| 2 | Step 2 PLAN (`rt_runtime_coverage_analysis_v1.md`) | complete |
| 3 | P0 `runtimeCoverageAnalysis.ts` | complete |
| 3 | P0 `runtimeCorridorCoverage.ts` | complete |
| 3 | Golden fixture + Vitest | complete |
| 4 | P1 `runtimeCoverageSelectors.ts` | complete |
| 4 | P1 `RuntimeCoverageStatusStrip.tsx` + banner | complete |
| 4 | P1 workstation wiring (Grid Mode) | complete |
| 5 | P2 `runtimeCoverageRenderModel.ts` | complete |
| 5 | P2 `runtimeCoverageLayer.ts` (Cesium) | complete |
| 5 | P2 `RuntimeCoverageSvgOverlay.tsx` (SVG parity) | complete |
| 5 | P2 visual layer registry `runtime_coverage_cells` | complete |
| 6 | Contract docs + freeze audit (this document) | complete |
| 7 | Freeze registry + post-wave closure | complete |

### Files (primary)

**P0 analytics**

- `platform/rt-sandbox-ui/src/coverage/runtimeCoverageAnalysis.ts`
- `platform/rt-sandbox-ui/src/coverage/runtimeCorridorCoverage.ts`
- `platform/rt-sandbox-ui/src/coverage/runtimeCoverageAnalysis.test.ts`
- `platform/rt-sandbox-ui/src/coverage/runtimeCorridorCoverage.test.ts`
- `fixtures/rt_sandbox/runtime_coverage_golden_v1.json`

**P1 status**

- `platform/rt-sandbox-ui/src/coverage/runtimeCoverageSelectors.ts`
- `platform/rt-sandbox-ui/src/coverage/RuntimeCoverageStatusStrip.tsx`
- `platform/rt-sandbox-ui/src/coverage/RuntimeCoverageStatusStrip.test.tsx`
- `platform/rt-sandbox-ui/src/governance/banners.ts` (`BANNER_RUNTIME_COVERAGE`)
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx` (strip wiring)

**P2 visualization**

- `platform/rt-sandbox-ui/src/coverage/runtimeCoverageRenderModel.ts`
- `platform/rt-sandbox-ui/src/coverage/runtimeCoverageLayer.ts`
- `platform/rt-sandbox-ui/src/coverage/RuntimeCoverageSvgOverlay.tsx`
- `platform/rt-sandbox-ui/src/coverage/runtimeCoverageVisualization.test.tsx`
- `platform/rt-sandbox-ui/src/cesium/visualLayerRegistry.ts`
- `platform/rt-sandbox-ui/src/cesium/CesiumRuntimeView.tsx`
- `platform/rt-sandbox-ui/src/components/CesiumRuntimePanel.tsx`
- `platform/rt-sandbox-ui/src/components/WorldEditingGrid.tsx`

**Docs**

- `docs/evaluation/rt_runtime_coverage_analysis_v1.md`
- `docs/evaluation/rt_radar_coverage_freeze_audit.md` (this document)

## Excluded scope

- Bridge / adapter / session manager changes
- Telemetry schema, IPC, or pull payload additions
- ROS / Gazebo / topic / parser changes
- Runtime entity mutation or command dispatch from coverage modules
- Radar placement recommendations or spawn automation
- Tactical assign / engage / autonomy integration
- Engagement or assignment authority semantics
- SA viewer / replay export / capture normalization
- Modifications to `planningCoverageAnalysis.ts`
- Planning Mode UX or overlay changes
- Terrain, LOS, probability of detection, Monte Carlo coupling

## Governance review

| Check | Result | Evidence |
|-------|--------|----------|
| UI-local read-only analytics only | **Pass** | No fetch/command dispatch in `src/coverage/` |
| No bridge changes | **Pass** | `platform/rt-sandbox-bridge/` — no coverage references |
| No telemetry changes | **Pass** | No new telemetry fields or pull consumers |
| No ROS / schema changes | **Pass** | UI-only wave |
| No autonomy | **Pass** | No scheduler or tactical loop coupling |
| No engagement authority | **Pass** | Display-only overlays and metrics |
| No radar placement recommendation | **Pass** | No planning recommendation preset imports |
| No planning-mode modifications | **Pass** | Strip/overlays Grid Mode only; planning paths unchanged |
| Explicit protected center consumption | **Pass** | `protectedCenterEntityId` prop; no selection inference |
| Governance banner present | **Pass** | `BANNER_RUNTIME_COVERAGE` on strip + Cesium chrome |
| Visual layer disclaimer aligned | **Pass** | Registry disclaimer matches banner vocabulary |
| Layer default off | **Pass** | `default_on: false`; `defaultVisibilityFromRegistry()` test |
| `planningCoverageAnalysis.ts` unchanged | **Pass** | `git diff` empty for planning module |
| Fail-closed: rectangle defense | **Pass** | `unsupported_defense_shape` → strip unavailable, no overlay |
| Fail-closed: missing center | **Pass** | `protected_center_unavailable` → strip unavailable, layer cleared |
| Fail-closed: missing corridor | **Pass** | Corridor % null; polylines empty; cells still render |
| No runtime command dispatch | **Pass** | Coverage modules are derive/render only |

## Regression audit

| Area | Result |
|------|--------|
| Planning Mode coverage workflows | Unchanged — `analyzePlanningCoverage` used only via existing planning paths + read-only runtime import |
| `planningCoverageAnalysis.ts` | No modifications in this wave |
| Visual layer defaults | `showRuntimeCoverageCells: false` in registry and tests |
| Rectangle defense | Analytics and overlays fail closed with guidance copy |
| Missing corridor | Non-blocking; corridor metrics omitted |
| Bridge contracts | No diffs |
| Tactical command paths | Coverage modules do not import command dispatch helpers |

## Validation record

Step 6 (2026-06-07):

| Command | Result |
|---------|--------|
| `npm test coverage` | **41 passed** (5 files) |
| `npm run build` | **Pass** |
| `git diff --check` | **Pass** |

Test coverage breakdown:

- P0 analytics + golden fixture: 12 tests
- P0 corridor polylines: 2 tests
- Planning module regression (unchanged): 9 tests
- P1 status strip + selectors: 10 tests
- P2 visualization + registry + SVG parity: 8 tests

## Known limitations (V1)

- Single global `radarDomeConfig.detectionM` for all runtime radars.
- Circle defense shape only; rectangle disables analytics.
- 2D horizontal heuristic — not sensor truth or detection probability.
- Grid-sampled warning disc; approximate percentages.
- Protected center from UI cache; may diverge from backend until pull-sync wave.
- Blind spot and corridor overlays are display-only cognition aids.
- Grid Mode only; Planning Mode has separate frozen coverage workflow.

## Deferred (out of wave scope)

- Pull-sync hydration of `protected_center_entity_id` from session state.
- Per-radar detection range fields (requires bridge wave).
- Rectangle defense adapter.
- Per-radar detection range fields (requires bridge wave).

## Freeze verdict

**FROZEN** — P0–P2 complete; registry entry in `freeze_registry_r1.md`.
No further PLAT implementation authorized without a new scoped plan.
