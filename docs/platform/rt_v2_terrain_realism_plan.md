# RT-V2 — Terrain / Realism (PLAT-RT-V2)

**Phase:** PLAT-RT-V2 — terrain and visual realism (RT UI only)  
**Prerequisite:** PLAT-RT-V1, PLAT-RT-T3/T5, PLAT-RT-G6, PLAT-RT-SA3 frozen  
**Authority:** [rt_v2_terrain_realism_v1.md](../evaluation/rt_v2_terrain_realism_v1.md)

## Goal

Increase **runtime environment realism** in `platform/rt-sandbox-ui/` via fictional terrain, ridge overlays, environment markers, sensor domes, and explanatory terrain cognition — without bridge, SA viewer, parser, or tactical authority changes.

## Architecture

UI-local `rt_fictional_terrain_v1` fixture → height sampling → Cesium mesh/overlays + SVG contour → cognition strips. Gazebo remains flat per G6 policy.

## Allowed

| Item | Location |
|------|----------|
| `src/cesium/rtFictionalTerrain.ts` | Height sampling, display offset |
| `src/cesium/fixtures/rt_ridge_terrain_v1.json` | Committed terrain profile |
| `terrainMeshLayer.ts`, `terrainOverlays.ts`, `environmentMarkers.ts`, `sensorDomeLayer.ts` | Visual layers |
| `terrainCognition.ts`, `TerrainCognitionStrip.tsx` | Explanatory cognition |
| `CesiumRuntimeView.tsx`, `CesiumRuntimePanel.tsx`, `WorldEditingGrid.tsx` | Wiring + toggles |
| Additive `BANNER_TERRAIN` in `banners.ts` | Panel-only banner |

## Forbidden

- `platform/rt-sandbox-bridge/` changes
- `platform/sa-r0-viewer/` changes
- Parser/topic/schema changes; new telemetry channels
- Tactical authority or assign/engage UI changes
- Cesium Ion; Gazebo SDF terrain physics
- Changes to frozen T1 banner strings (additive terrain banner only)

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-V2 frozen. Do not start **RT-X1** (experimentation frontier) without explicit new wave audit.

## Related

- [rt_v2_governance_review_r1.md](../evaluation/rt_v2_governance_review_r1.md)
- [rt_v2_terrain_realism_review_r1.md](../evaluation/rt_v2_terrain_realism_review_r1.md)
- [rt_v2_freeze_audit.md](../evaluation/rt_v2_freeze_audit.md)
