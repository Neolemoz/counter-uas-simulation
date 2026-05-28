# RT-V3 P1 — Visibility Overlay Foundations (PLAT-RT-V3 P1)

**Phase:** PLAT-RT-V3 P1 — visibility overlays + cognition grouping  
**Prerequisite:** PLAT-RT-V3 P0 frozen — [rt_plat_v3_p0_freeze_audit.md](../evaluation/rt_plat_v3_p0_freeze_audit.md)  
**Authority:** [rt_runtime_visualization_fidelity_v3_v1.md](../evaluation/rt_runtime_visualization_fidelity_v3_v1.md)

## Goal

Deliver V3 visibility overlay modules, registry-driven toggles, grouped cognition hub blocks, `BANNER_VISIBILITY_V3`, and warn-only performance budget UI — without bridge/SA changes or P2 layout work.

## Delivered (P1)

| Item | Location |
|------|----------|
| Visibility wedge layer | `visibilityWedgeLayer.ts` |
| Horizon hint layer | `horizonHintLayer.ts` |
| Stacked LOS orchestration | `stackedLosPresentation.ts` |
| Visibility cognition | `visibilityCognition.ts`, `VisibilityCognitionStrip.tsx` |
| Grouped hub blocks | `RuntimeCognitionHub.tsx` |
| Banner | `BANNER_VISIBILITY_V3` in `banners.ts` |
| App-level visibility state | `App.tsx`, `CesiumRuntimePanel.tsx`, `CesiumRuntimeView.tsx` |
| Registry P1 keys | fixtures + `visualLayerRegistry.ts` |
| Tests | `*.test.ts` (overlay + registry + governance) |

## Forbidden (unchanged)

- `platform/rt-sandbox-bridge/` changes  
- `platform/sa-r0-viewer/` changes  
- Cognition rail column / inactive-slot dimming — P2  
- Performance budget enforcement (warn-only in P1)  

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Stop line

PLAT-RT-V3 P1 frozen. Do not start **P2** without governance review + freeze audit.

## Related

- [rt_roadmap_plat_rt_v3_v1.md](../evaluation/rt_roadmap_plat_rt_v3_v1.md)
- [rt_plat_v3_p1_freeze_audit.md](../evaluation/rt_plat_v3_p1_freeze_audit.md)
