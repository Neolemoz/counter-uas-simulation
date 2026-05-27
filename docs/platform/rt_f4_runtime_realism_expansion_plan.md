# RT-F4 — Runtime Realism Expansion (PLAN-RT-F4)

**Phase:** PLAN-RT-F4 — UI/runtime visualization realism (docs only)  
**Prerequisite:** PLAT-RT-V2, PLAT-RT-F2, PLAT-RT-F3 frozen  
**Contract:** [rt_runtime_realism_expansion_v1.md](../evaluation/rt_runtime_realism_expansion_v1.md)  
**Supplements:** [rt_v2_terrain_realism_v1.md](../evaluation/rt_v2_terrain_realism_v1.md)

## Goal

Extend RT sandbox **environment realism** in `platform/rt-sandbox-ui/` — richer fictional terrain visuals, explanatory cognition (LOS / visibility hints), Cesium polish, workstation integration — without bridge, SA viewer, tactical authority, parser, or Gazebo behavior changes.

## Architecture

Additive layers on frozen V2 `rt_fictional_terrain_v1` fixture → contour/band/vegetation overlays → cognition strips + hub lines. Registry and Gazebo remain flat.

## Allowed (PLAT advisory)

| Item | Location |
|------|----------|
| Fixture extension | `fixtures/rt_ridge_terrain_v1.json` |
| Contour + band layers | `terrainContourLayer.ts`, `terrainOverlays.ts` |
| Vegetation vs occlusion markers | `environmentMarkers.ts` |
| Sensor dome context | `sensorDomeLayer.ts` |
| Cognition | `terrainCognition.ts`, `TerrainCognitionStrip.tsx` |
| Workstation | `CesiumRuntimePanel`, `RuntimeCognitionHub`, `BackgroundDiagnostics` |
| Experiment snapshot fields | `experimentStore.ts` `terrain_context` |

## Forbidden

- `platform/rt-sandbox-bridge/` protocol or telemetry channel changes
- `platform/sa-r0-viewer/` changes
- Parser/topic/schema changes; tactical authority changes
- Gazebo SDF terrain / physics; Cesium Ion
- M3 distributed runtime; SA auto-import; workflow changes

## PLAT-RT-F4 scope

See [rt_roadmap_plat_rt_f4_v1.md](../evaluation/rt_roadmap_plat_rt_f4_v1.md).

## Stop line

PLAN-RT-F4 frozen. Do not start PLAT-RT-F4 without implementation plan + PLAT governance review + freeze audit.

## Related

- [rt_f4_governance_review_r1.md](../evaluation/rt_f4_governance_review_r1.md)
- [rt_f4_realism_review_r1.md](../evaluation/rt_f4_realism_review_r1.md)
- [rt_f4_freeze_audit.md](../evaluation/rt_f4_freeze_audit.md)
