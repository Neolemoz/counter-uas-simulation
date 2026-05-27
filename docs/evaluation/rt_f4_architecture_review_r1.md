# RT-F4 — Architecture Review R1 (PLAN-RT-F4)

**Phase:** PLAN-RT-F4  
**Contract:** [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md)

## Summary

F4 extends the frozen V2 terrain stack inside `platform/rt-sandbox-ui/` only. No new bridge modules, telemetry channels, or cross-package imports.

## Layer flow

```
rt_ridge_terrain_v1.json (extended)
  → rtFictionalTerrain.ts (contours, bands, vegetation lists)
  → terrainLayers.ts (visibility flags)
  → terrainMeshLayer | terrainOverlays | terrainContourLayer | environmentMarkers | sensorDomeLayer
  → CesiumRuntimeView + TerrainCognitionStrip + RuntimeCognitionHub
```

## Boundaries

| Boundary | Status |
|----------|--------|
| Bridge registry authority | Preserved — flat spawn z |
| Gazebo adapter | Unchanged |
| SA viewer | No imports |
| Experiment workbench | Additive `terrain_context` fields only |

## Risks

| Risk | Mitigation |
|------|------------|
| Contour misread as survey | Default off; explanatory labels |
| LOS read as tracker output | Heuristic naming; governance lexicon |
| Scope creep to Gazebo truth | Explicit non-goals in contract |

**Recommendation:** Proceed to PLAT-RT-F4 within `rt-sandbox-ui/` only.
