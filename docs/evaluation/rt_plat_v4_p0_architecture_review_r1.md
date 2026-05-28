# RT-V4 P0 - Architecture Review R1

**Phase:** PLAT-RT-V4 P0 - visual registry and density foundations
**Plan:** [rt_plat_v4_p0_visual_registry_plan.md](../platform/rt_plat_v4_p0_visual_registry_plan.md)
**Freeze audit:** [rt_plat_v4_p0_freeze_audit.md](rt_plat_v4_p0_freeze_audit.md)

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Layering integrity | **Pass** - UI-local display/cognition only |
| Registry design | **Pass** - additive V4 rows on frozen V3 baseline |
| Density policy | **Pass** - warn-only summaries, no enforcement |
| Multi-session cognition | **Pass** - selected/comparison/background roles only |
| P1 separation | **Pass** - no new overlay geometry |

**Recommendation:** Freeze **PLAT-RT-V4 P0**. Next advisory step is **PLAT-RT-V4 P1**, not started here.

## 1. Code architecture

| Area | Change |
|------|--------|
| `visualLayerRegistry.ts` | Adds `rt_visual_layer_registry_v4`, density/comparison groups, V4 P0 display-only rows |
| `VisualLayerToggleRail.tsx` | Groups density and comparison toggles; shows warn-only budget tone |
| `RuntimeCognitionHub.tsx` | Adds density and session-compare cognition blocks |
| `sessionComparisonCognition.ts` | Derives selected/comparison/background visual roles |
| `SessionComparisonCognitionStrip.tsx` | Renders explanatory compare-only surface |
| `CesiumRuntimePanel.tsx` | Aligns session chrome with compare cognition |

## 2. Non-goal audit

| Non-goal | Result |
|----------|--------|
| Bridge/runtime changes | **None** |
| SA viewer changes | **None** |
| Tactical redesign | **None** |
| Import logic | **None** |
| Browser authority | **None** |
| P1 overlays | **None** |
| X3 | **None** |

## 3. Risk assessment

| Risk | Mitigation |
|------|------------|
| Density warnings read as enforcement | Copy says warn-only/advisory; no commands are invoked |
| Comparison rows read as commandable | Only selected session has `commandable=true`; UI labels say no commands |
| V4 registry breaks V3 defaults | Tests preserve V1/V2/F4/V3 defaults and P1 default-off layers |

## Architecture verdict

**Pass.** PLAT-RT-V4 P0 is a narrow UI-local foundation wave and preserves the frozen runtime architecture.
