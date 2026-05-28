# RT - PLAT-RT-V4 Implementation Roadmap v1

**Phase:** PLAN-RT-V4 frozen -> **PLAT-RT-V4 P0-P1 frozen**; P2 future
**Prerequisite:** [rt_v4_freeze_audit.md](rt_v4_freeze_audit.md) (PLAN-RT-V4 docs frozen)
**Contracts:** [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md), [rt_visual_density_management_v1.md](rt_visual_density_management_v1.md), [rt_visual_multi_session_cognition_v1.md](rt_visual_multi_session_cognition_v1.md)

This roadmap tracks PLAT-RT-V4 implementation. P0 and P1 are frozen; P2 remains future and is not authorized by P1.

## P0 - Density policy foundations

**Prerequisite:** PLAN-RT-V4 frozen; V4 governance review accepted for P0.

| Item | Future location | Status |
|------|-----------------|--------|
| Density policy helper | `platform/rt-sandbox-ui/src/cesium/visualLayerRegistry.ts` | **Delivered** |
| Layer budget summary | `VisualLayerToggleRail.tsx`, `RuntimeCognitionHub.tsx` | **Delivered** |
| Legend grouping for density | Existing visualization rail | **Delivered** |
| Session compare primitives | `sessionComparisonCognition.ts`, `SessionComparisonCognitionStrip.tsx` | **Delivered** |
| Vitest policy checks | RT UI tests | **Delivered** |
| Governance check | `rt_plat_v4_p0_*` docs | **Delivered** |

**Frozen:** [rt_plat_v4_p0_freeze_audit.md](rt_plat_v4_p0_freeze_audit.md) - **PLAT-RT-V4 P0 frozen**.

**Risk:** Low-Med. The phase stayed policy-first and avoided new visual geometry.

## P1 - Advanced visibility and terrain cognition

**Prerequisite:** P0 frozen.

| Item | Future location | Status |
|------|-----------------|--------|
| Visibility corridor visual | `visibilityOverlayV4.ts`, `CesiumRuntimeView.tsx` | **Delivered** |
| Occlusion confidence bands | `visibilityOverlayV4.ts`, `visibilityCognition.ts` | **Delivered** |
| Terrain relation labels | `visibilityOverlayV4.ts`, `RuntimeCognitionHub.tsx` | **Delivered** |
| Default-off toggle wiring | `visualLayerRegistry.ts`, `VisualLayerToggleRail.tsx` | **Delivered** |
| Realism review | `rt_plat_v4_p1_visualization_realism_review_r1.md` | **Delivered** |

**Frozen:** [rt_plat_v4_p1_freeze_audit.md](rt_plat_v4_p1_freeze_audit.md) - **PLAT-RT-V4 P1 frozen**.

**Risk:** Med. Misread risk is the main cost; every new visual remains explanatory.

## P2 - Multi-session comparison visuals

**Prerequisite:** P1 frozen; M3 session isolation re-check.

| Item | Future location | Status |
|------|-----------------|--------|
| Session comparison strip | Existing workstation session area | Planned |
| Ghosted comparison geometry | Existing Cesium panel | Planned, default off |
| Background diagnostic cohesion row | Existing background diagnostics | Planned |
| Session compare legend | Existing cognition hub/rail | Planned |
| Isolation tests | RT UI tests + bridge deny-path tests | Required |

**Risk:** Med. The phase must not introduce cross-session command affordances or distributed runtime assumptions.

## Explicit out of scope

- Bridge protocol or subcommand changes
- ROS/Gazebo runtime changes
- SA viewer changes
- Auto-import, corpus writes, or federation
- Browser->ROS authority
- Tactical redesign or operational readiness scoring
- Distributed multi-bridge

## Validation (future PLAT)

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

Each PLAT phase requires its own plan, governance review, visualization realism review when visuals change, freeze audit, and freeze registry row.

## Stop line

PLAT-RT-V4 P1 is frozen. Do not start **P2** without a scoped P2 plan, governance review, visualization realism review, validation, and freeze audit.

## Related

- [rt_roadmap_next_frontiers_v9.md](rt_roadmap_next_frontiers_v9.md)
- [rt_v4_governance_review_r1.md](rt_v4_governance_review_r1.md)
