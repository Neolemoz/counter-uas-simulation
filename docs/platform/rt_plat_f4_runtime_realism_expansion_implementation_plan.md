# RT-F4 — Runtime Realism Expansion (PLAT-RT-F4)

**Phase:** PLAT-RT-F4 — environment visualization + cognition  
**Prerequisite:** PLAN-RT-F4 frozen; PLAT-RT-V2 frozen  
**Authority:** [rt_runtime_realism_expansion_v1.md](../evaluation/rt_runtime_realism_expansion_v1.md)

## Goal

Deliver richer fictional terrain visuals, explanatory LOS/visibility cognition, Cesium polish, and workstation integration in `platform/rt-sandbox-ui/` only.

## Delivered

| Item | Location |
|------|----------|
| Extended fixture | `fixtures/rt_ridge_terrain_v1.json` |
| Contour generation + layer | `rtFictionalTerrain.ts`, `terrainContourLayer.ts` |
| Elevation bands on ridges | `terrainOverlays.ts` |
| Vegetation + occlusion markers | `environmentMarkers.ts` |
| Sensor dome context | `sensorDomeLayer.ts` |
| LOS segment (viewer-only) | `losSegmentLayer.ts` |
| Cognition + hub | `terrainCognition.ts`, `TerrainCognitionStrip.tsx`, `RuntimeCognitionHub.tsx` |
| Camera presets | `cameraHelpers.ts` (`valleyFloor`, `crestLine`) |
| Workstation wiring | `App.tsx`, `CesiumRuntimePanel.tsx`, `BackgroundDiagnostics.tsx` |
| Experiment terrain_context | `experimentStore.ts`, `experimentSchema.ts` |
| Governance banner | `BANNER_REALISM_F4` |

## Forbidden

- `platform/rt-sandbox-bridge/` changes
- `platform/sa-r0-viewer/` changes
- Parser/topic changes; tactical authority changes
- Gazebo behavior changes

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_annex_pack.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-F4 frozen. Next advisory: **M3 optional polish** or **F5 advanced runtime experiments** — new wave audit required.

## Related

- [rt_plat_f4_governance_review_r1.md](../evaluation/rt_plat_f4_governance_review_r1.md)
- [rt_plat_f4_realism_review_r1.md](../evaluation/rt_plat_f4_realism_review_r1.md)
- [rt_plat_f4_freeze_audit.md](../evaluation/rt_plat_f4_freeze_audit.md)
