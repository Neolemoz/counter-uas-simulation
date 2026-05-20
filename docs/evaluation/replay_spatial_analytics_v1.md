# Replay Spatial Analytics (`replay_spatial_analytics_v1`)

Normative semantics for replay-side spatial concentration overlays in the SA-R0 viewer. **Explanatory replay concentration only** — not operational prediction, validated probability, or deployment confidence.

See also: [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md), [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md).

## Grid model

| Field | Type | Notes |
|-------|------|-------|
| `origin_enu_m` | `[x, y]` | Fictional ENU origin (meters) |
| `spacing_m` | number | Cell size (default 100 m, max grid 40×30) |
| `size` | `[cols, rows]` | Column-major flat arrays: index = `row * cols + col` |

All coordinates use the bundle `georef_display` fictional frame.

## Layer kinds

| Layer key | Meaning | Wording in UI |
|-----------|---------|---------------|
| `ambiguity_density` | Narrative ambiguity windows + ambiguity-category events binned | "Replay ambiguity concentration" |
| `los_degraded` | LOS segments `terrain_blocked` / `partially_occluded` midpoints | "LOS degradation concentration" |
| `first_detection` | First detection marker / event positions | "First-detection replay points" |
| `intercept_outcome` | Terminal intercept track samples by replay outcome label | "Intercept outcome replay regions" |
| `topology_sensitivity` | Cells where member outcome metric differs from baseline | "Topology sensitivity concentration" |
| `replay_event_clusters` | Grid peaks above P75 count | "Replay event concentration zones" |

Each layer includes a `caveat` string forbidding operational interpretation.

## Aggregation rules (deterministic)

1. **Ambiguity** — increment cell for each ambiguity window span (sample at window midpoint) and ambiguity narrative event (track sample at `line_index` if available).
2. **LOS** — increment cell at segment polyline midpoint when status is degraded.
3. **First detection** — append `[x, y]` ENU point per member; optional +1 neighbor blur in aggregate builder only.
4. **Intercept outcome** — bucket from `comprehension.at_a_glance` cards (replay labels only); increment at interceptor terminal track sample.
5. **Topology sensitivity** — compare `first_detection_t` (or duration span) vs baseline member; increment cell at threat ingress sample when delta exceeds threshold (≥2 line indices).
6. **Clusters** — cells with count ≥ P75 of non-zero cells; emit `centroids_enu_m` + descriptive `labels` (no ML).

## Placement

- **Sweep manifest:** `spatial_aggregate.layers` (cross-member).
- **Bundle (optional):** `spatial_analytics` with same shape for single-member view.

## Governance

**Do:** Use "concentration", "replay-local", "explanatory observation".

**Don't:** Use "probability", "kill zone", "WEZ", "readiness", "validated effectiveness".

## Consumer

`platform/sa-r0-viewer/src/replay/spatial/spatialAnalytics.ts` and `src/cesium/spatialGridLayer.ts`.
