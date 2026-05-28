# RT-V3 P0 — Visual Layer Registry Foundations (PLAT-RT-V3 P0)

**Phase:** PLAT-RT-V3 P0 — visual layer registry + contract tests + grouped toggles  
**Prerequisite:** PLAN-RT-V3 frozen — [rt_v3_freeze_audit.md](../evaluation/rt_v3_freeze_audit.md); PLAT-RT-V1/V2/F4 frozen  
**Authority:** [rt_runtime_visualization_fidelity_v3_v1.md](../evaluation/rt_runtime_visualization_fidelity_v3_v1.md)

## Goal

Deliver canonical `rt_visual_layer_registry_v3` in `platform/rt-sandbox-ui/`, Vitest contract tests, and grouped layer toggles wired to existing Cesium modules — without bridge changes, SA viewer changes, V3 overlay geometry, or workstation layout redesign.

## Delivered (P0)

| Item | Location |
|------|----------|
| Registry module | `platform/rt-sandbox-ui/src/cesium/visualLayerRegistry.ts` |
| Bundled fixture | `platform/rt-sandbox-ui/src/cesium/fixtures/v3_layer_registry_v3.json` |
| Reference fixture | `fixtures/rt_visualization/v3_layer_registry_example.json` |
| Contract tests | `platform/rt-sandbox-ui/src/cesium/visualLayerRegistry.test.ts` |
| Grouped toggle rail | `platform/rt-sandbox-ui/src/components/VisualLayerToggleRail.tsx` |
| Panel wiring | `platform/rt-sandbox-ui/src/components/CesiumRuntimePanel.tsx` |
| Default alias | `platform/rt-sandbox-ui/src/cesium/terrainLayers.ts` → registry defaults |

## Forbidden (unchanged)

- `platform/rt-sandbox-bridge/` changes  
- `platform/sa-r0-viewer/` changes  
- `visibilityWedgeLayer`, `horizonHintLayer`, `stackedLosPresentation` — P1  
- `BANNER_VISIBILITY_V3`, cognition hub block reorder, cognition rail — P1/P2  
- Re-opening frozen V1/V2/F4 default-on without new PLAN wave  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-V3 P0 frozen. Do not start **P1** without governance review + freeze audit.

## Related

- [rt_roadmap_plat_rt_v3_v1.md](../evaluation/rt_roadmap_plat_rt_v3_v1.md)
- [rt_plat_v3_p0_freeze_audit.md](../evaluation/rt_plat_v3_p0_freeze_audit.md)
- [rt_plat_v3_p0_governance_review_r1.md](../evaluation/rt_plat_v3_p0_governance_review_r1.md)
