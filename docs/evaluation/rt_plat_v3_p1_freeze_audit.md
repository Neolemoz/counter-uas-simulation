# RT-V3 P1 — Freeze Audit (PLAT-RT-V3 P1)

**Phase:** PLAT-RT-V3 P1 — visibility overlay foundations  
**Status:** frozen

**Plan:** [rt_plat_v3_p1_visibility_overlays_plan.md](../platform/rt_plat_v3_p1_visibility_overlays_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Visibility wedge layer | `visibilityWedgeLayer.ts` |
| 2 | Horizon hint layer | `horizonHintLayer.ts` |
| 3 | Stacked LOS orchestration | `stackedLosPresentation.ts` |
| 4 | Registry P1 toggle keys | fixtures + `visualLayerRegistry.ts` |
| 5 | App-level `layerVisibility` | `App.tsx`, `CesiumRuntimePanel.tsx`, `CesiumRuntimeView.tsx` |
| 6 | `BANNER_VISIBILITY_V3` | `banners.ts` |
| 7 | Visibility cognition strip | `VisibilityCognitionStrip.tsx`, `visibilityCognition.ts` |
| 8 | Grouped hub blocks | `RuntimeCognitionHub.tsx` |
| 9 | Performance budget warn (advisory) | `performanceBudgetAdvisory` |
| 10 | Tests | overlay + registry + governance Vitest |
| 11 | Reviews + registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/` or `platform/sa-r0-viewer/`.

---

## P1 summary

PLAT-RT-V3 P1 adds three default-off visibility overlay modules (wedge, horizon, stacked LOS) driven by the frozen P0 registry, a visibility toggle group in the Cesium panel, additive `BANNER_VISIBILITY_V3`, grouped collapsible cognition hub blocks (terrain / visibility / fidelity / sensor / authority), and warn-only overlay budget messaging. F4 legacy LOS behavior is preserved when stacked LOS is off.

---

## Boundary guarantees

- Bridge entity registry remains command authority  
- Overlays and hub text are explanatory only  
- No SA viewer, auto-import, or federation scope  
- No performance budget enforcement (warn-only)  
- P2 layout (cognition rail column, inactive-slot dimming) not started  

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| `test_rt_sandbox_bridge.py` | 151 pass; 2 pre-existing `templateGuards.ts` string-scan failures |
| `platform/rt-sandbox-ui` Vitest | 273 pass (69 files) |
| `npm run build` | pass (~466 KB JS, ~130 KB gzip) |
| `scripts/ci_eval.sh tier0-rt-ui` | pass |

---

## Recommended next (advisory)

**PLAT-RT-V3 P2** per [rt_roadmap_plat_rt_v3_v1.md](rt_roadmap_plat_rt_v3_v1.md):

1. Cognition rail column in `RuntimeWorkstationShell`  
2. Compact background diagnostic row  
3. Inactive-slot marker dimming (55% opacity)  
4. Optional session-scoped toggle memory  

**Alternate:** checkpoint manual overlay UX review before P2.

---

## Stop line

PLAT-RT-V3 P1 frozen. **PLAT-RT-V3 P2** not authorized until explicit P2 scope + governance review.
