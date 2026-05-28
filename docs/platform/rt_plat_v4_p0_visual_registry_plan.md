# RT-V4 P0 - Visual Registry + Density Foundations (PLAT-RT-V4 P0)

**Phase:** PLAT-RT-V4 P0 - visual registry, density controls, compare cognition foundations
**Prerequisite:** PLAN-RT-V4 frozen - [rt_v4_freeze_audit.md](../evaluation/rt_v4_freeze_audit.md)
**Authority:** [rt_visualization_fidelity_v4.md](../evaluation/rt_visualization_fidelity_v4.md), [rt_visual_density_management_v1.md](../evaluation/rt_visual_density_management_v1.md), [rt_visual_multi_session_cognition_v1.md](../evaluation/rt_visual_multi_session_cognition_v1.md)

## Goal

Deliver the V4 P0 foundations in `platform/rt-sandbox-ui/`: additive visual registry extensions, density-control grouping, warn-only budget indicators, and local multi-session comparison cognition. This phase is display/cognition only and does not implement P1 visibility or terrain overlays.

## Delivered

| Item | Location |
|------|----------|
| Registry v4 extensions | `platform/rt-sandbox-ui/src/cesium/visualLayerRegistry.ts` |
| Density grouping and warn-only summary | `VisualLayerToggleRail.tsx`, `RuntimeCognitionHub.tsx` |
| Session comparison primitives | `sessionComparisonCognition.ts`, `SessionComparisonCognitionStrip.tsx` |
| P0 tests | `visualLayerRegistry.test.ts`, `sessionComparisonCognition.test.ts` |
| Workstation cohesion | `CesiumRuntimePanel.tsx`, `RuntimeCognitionHub.tsx`, `App.tsx` |

## Architecture

```text
V3 visual registry baseline
  + V4 P0 display-only registry rows
  + density_context / comparison_context grouped toggles
  + warn-only density summary
  + selected/comparison/background session visual rules
  -> RT UI cognition surfaces only
```

| V4 P0 layer | Default | Role |
|-------------|---------|------|
| `density_warnings_v4` | on | Warn-only density indicator |
| `layer_budget_summary_v4` | on | Explanatory layer budget summary |
| `session_contrast_v4` | on | Selected/background visual contrast rule |
| `comparison_ghosts_v4` | off | Compare-only visual primitive, no commands |

## Boundary guarantees

- No `platform/rt-sandbox-bridge/` changes
- No `platform/sa-r0-viewer/` changes
- No bridge endpoints, subcommands, telemetry, parser, topic, or schema changes
- No browser->ROS authority
- No import, auto-import, corpus write, federation, or distributed runtime
- No P1 overlays: no visibility corridor, occlusion confidence bands, or new Cesium geometry
- Command target remains selected RT session under existing lock semantics

## Validation

```bash
cd platform/rt-sandbox-ui && npm test
cd platform/rt-sandbox-ui && npm run build
scripts/ci_eval.sh tier0-rt-ui
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

Bridge pytest retains the known two SA string-scan failures documented in prior freeze audits; no bridge or SA files changed in P0.

## Stop line

PLAT-RT-V4 P0 frozen. Do not start **P1** without a scoped P1 plan, visualization realism review, governance review, validation, and freeze audit.

## Related

- [rt_plat_v4_p0_architecture_review_r1.md](../evaluation/rt_plat_v4_p0_architecture_review_r1.md)
- [rt_plat_v4_p0_governance_review_r1.md](../evaluation/rt_plat_v4_p0_governance_review_r1.md)
- [rt_plat_v4_p0_visualization_realism_review_r1.md](../evaluation/rt_plat_v4_p0_visualization_realism_review_r1.md)
- [rt_plat_v4_p0_freeze_audit.md](../evaluation/rt_plat_v4_p0_freeze_audit.md)
