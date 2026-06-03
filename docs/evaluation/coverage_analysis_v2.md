# Coverage Analysis V2

Status: frozen

## Scope

Coverage Analysis V2 extends RT Sandbox Planning Mode with richer UI-local
planning analytics and display-only advisory overlays.

Included:

- Pure planning coverage analysis module.
- Backward-compatible coverage percent, covered area, uncovered area, and blind
  spot hint outputs.
- Per-sampled-cell `coverageCount` and `coveringRadarIds`.
- Overlap and redundancy metrics.
- Blind Spot V2 advisory summary.
- Advisory radar recommendation.
- Cesium display-only blind spot overlay and recommended radar marker.
- Planning Mode analytics and advisory UI copy.

Excluded:

- Runtime state mutation.
- Bridge command or API changes.
- Monte Carlo integration.
- LOS logic.
- Sensor logic.
- Terrain coupling.
- Telemetry, schema, capture, parser, or replay contract changes.
- Runtime entity creation for advisory visualization.

## Architecture Summary

Coverage Analysis V2 keeps analytics inside the RT Sandbox UI. The analysis
entry point is `planningCoverageAnalysis.ts`, which consumes the completed
Planning Mode defense polygon and UI-local Planning radar sites.

The relevant UI path is:

`AppWorkstationSlots -> PlanningModePanel -> CesiumRuntimePanel -> CesiumRuntimeView`

The relevant Cesium drawing path is:

`syncPlanningDefenseAreaLayer -> analyzePlanningCoverage -> Planning-only Cesium entities`

Planning Mode remains browser-local. Advisory overlays use Planning entity IDs
under the existing Planning defense-area prefix and do not create runtime
entities.

## Analytics Model

`PlanningCoverageAnalysis` contains:

- `estimate`: the backward-compatible `PlanningCoverageEstimate`.
- `sampledCells`: all sampled cells inside the completed polygon.
- `coveredCells`: sampled cells with at least one covering radar.
- `uncoveredCells`: sampled cells with no covering radar.
- `overlapCells`: sampled cells with two or more covering radars.
- `overlapCellCount`.
- `overlapAreaM2`.
- `overlapPercent`.
- `redundancyPercent`.
- `blindSpotV2`.
- `radarRecommendation`.

Each analyzed cell adds:

- `coverageCount`.
- `coveringRadarIds`.

These fields are UI-local analysis data only. They are not runtime schema,
bridge payload, telemetry, MC input, sensor state, or LOS state.

## Coverage Calculation

Coverage continues to use the existing Planning Mode heuristic:

- Sample a deterministic grid across the completed defense polygon bounds.
- Keep only cells whose centers are inside the polygon.
- Treat a cell as covered when at least one Planning radar detection range
  contains the cell center.
- Estimate covered and uncovered area from the sampled cell counts and polygon
  area.

The legacy `estimatePlanningCoverage` output is preserved by delegating to the
new analysis layer and returning `analysis.estimate`.

## Overlap Calculation

Overlap is derived from sampled cells where:

`coverageCount >= 2`

`overlapCellCount` is the number of those cells.

`overlapAreaM2` is:

`overlapCellCount * sampledCellAreaM2`

`overlapPercent` is:

`overlapAreaM2 / totalPolygonAreaM2 * 100`

Overlap does not double-count coverage percent. Coverage remains boolean per
sampled cell for the legacy estimate.

## Redundancy Calculation

Redundancy is the percent of covered area that is covered by two or more
Planning radars:

`redundancyPercent = overlapAreaM2 / estimatedCoveredAreaM2 * 100`

When no area is covered, redundancy is `0`.

## Blind Spot Analysis

Blind Spot V2 uses uncovered sampled cells only.

The analysis derives:

- uncovered area summary.
- uncovered percent.
- major uncovered sectors.
- farthest uncovered cluster hints.

Sectors are deterministic quadrants relative to the polygon bounds center:

- NE
- NW
- SE
- SW

The advisory summary identifies the largest uncovered sector by uncovered area.
Tie-breaking is deterministic by sector order.

## Recommendation Analysis

The radar recommendation is advisory and deterministic. It uses:

- uncovered cells.
- major uncovered sector summary.
- configured Planning radar presets.
- existing Planning radar locations.
- uncovered percent.

The recommendation includes:

- recommended radar preset.
- approximate placement point.
- reason text.

No recommendation is emitted when sampled cells are fully covered or no radar
presets are available.

## Visualization Behavior

Cesium Planning Mode renders display-only advisory visualization when Planning
coverage and blind spot display are enabled:

- advisory blind spot sector polygons.
- advisory recommended radar placement marker.
- advisory recommended radar range ring.

The overlays remain Planning-only and use Planning entity IDs. They do not use
runtime entity IDs and do not call runtime or bridge mutation paths.

## Governance Audit

Confirmed boundaries:

- UI-local only.
- Planning-only.
- No runtime mutation.
- No bridge changes.
- No MC integration.
- No LOS logic.
- No sensor logic.
- No terrain coupling.
- No telemetry changes.
- No schema changes.
- No parser, capture, or replay contract changes.

The UI labels analytics and advisory outputs as planning heuristics. These
outputs are presentation and analysis aids only; they are not runtime authority.

## Limitations

- Coverage remains grid-sampled and approximate.
- Radar ranges remain simple 2D circles from Planning radar presets.
- Blind spot sectors are quadrant summaries, not geometric clustering.
- Recommendation placement is approximate and heuristic.
- Terrain visualization does not affect analytics.
- LOS and sensor effects are intentionally excluded.
- No browser screenshot evidence is stored in this freeze note.

## Future Work

Future work must remain additive and separately scoped:

- Higher-resolution analysis controls with clear performance bounds.
- More explicit overlap visualization.
- Alternative deterministic blind spot clustering.
- Recommendation comparison across multiple candidate placements.
- Screenshot-based visual regression checks for advisory overlays.
- Optional export/import of planning analytics only after separate governance
  review.

Future work must not couple Planning coverage analytics to runtime state,
bridge commands, MC, LOS, sensors, terrain, telemetry, schemas, capture,
parser contracts, or replay authority without a separate governed plan.

## Validation

Commands run at freeze:

- `npm test`
- `npm run build`
- `git diff --check`
- `git status`

Validation result at freeze: pass.

Build note: Vite still emits the existing chunk-size warning.
