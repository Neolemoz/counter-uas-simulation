# RT — PLAT-RT-V3 Implementation Roadmap v1

**Phase:** PLAN-RT-V3 frozen → **PLAT-RT-V3 complete** (P0–P2 frozen)  
**Prerequisite:** [rt_v3_freeze_audit.md](rt_v3_freeze_audit.md) (PLAN-RT-V3 docs frozen)  
**Contracts:** [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md), [rt_cesium_workstation_visualization_v3_v1.md](rt_cesium_workstation_visualization_v3_v1.md)

---

## P0 — Layer registry + contract tests

**Prerequisite:** PLAN-RT-V3 frozen; [rt_v3_governance_review_r1.md](rt_v3_governance_review_r1.md) closed for P0

| Item | Location | Status |
|------|----------|--------|
| `visualLayerRegistry.ts` | `platform/rt-sandbox-ui/src/cesium/` | **Done** |
| Load `rt_visual_layer_registry_v3` fixture | `fixtures/rt_visualization/v3_layer_registry_example.json` | **Done** |
| Toggle wiring to existing layers | `VisualLayerToggleRail`, `CesiumRuntimePanel` | **Done** |
| Vitest registry contract | `visualLayerRegistry.test.ts` | **Done** |
| Governance: frozen defaults unchanged | [rt_plat_v3_p0_governance_review_r1.md](rt_plat_v3_p0_governance_review_r1.md) | **Done** |
| `tier0-rt-ui` | CI | **Done** |

**PLAT plan:** [rt_plat_v3_p0_visual_layer_registry_plan.md](../platform/rt_plat_v3_p0_visual_layer_registry_plan.md)

**Freeze:** [rt_plat_v3_p0_freeze_audit.md](rt_plat_v3_p0_freeze_audit.md) — **PLAT-RT-V3 P0 frozen**

---

## P1 — Visibility overlays + cognition grouping

**Prerequisite:** P0 frozen

| Item | Location | Status |
|------|----------|--------|
| `visibilityWedgeLayer.ts` | `src/cesium/` | **Done** |
| `horizonHintLayer.ts` | `src/cesium/` | **Done** |
| `stackedLosPresentation.ts` | `src/cesium/` | **Done** |
| Grouped hub blocks | `RuntimeCognitionHub.tsx` | **Done** |
| `BANNER_VISIBILITY_V3` | `banners.ts` | **Done** |
| Visibility toggle group | `VisualLayerToggleRail` + registry P1 keys | **Done** |
| Vitest overlay + banner tests | `src/cesium/*.test.ts` | **Done** |
| Performance budget warn | `visibilityCognition.ts` | **Done** |

**PLAT plan:** [rt_plat_v3_p1_visibility_overlays_plan.md](../platform/rt_plat_v3_p1_visibility_overlays_plan.md)

**Freeze:** [rt_plat_v3_p1_freeze_audit.md](rt_plat_v3_p1_freeze_audit.md) — **PLAT-RT-V3 P1 frozen**

No new bridge commands. No F5b coupling changes.

---

## P2 — Workstation layout + multi-session chrome + diagnostics

**Prerequisite:** P1 frozen

| Item | Location | Status |
|------|----------|--------|
| Cognition rail column | `RuntimeWorkstationShell.tsx`, `App.tsx` | **Done** |
| Compact background diagnostic row | `BackgroundDiagnosticsCompact.tsx` | **Done** |
| Inactive-slot marker dimming | `entityMarkers.ts`, `SessionTabBar.tsx` | **Done** |
| Session-scoped toggle memory | `sessionLayerVisibilityStore.ts` | **Done** |
| Registry budget summary | `visualLayerRegistry.ts`, `VisualLayerToggleRail.tsx` | **Done** |
| `tier0-rt-ui` + build size check | CI | **Done** |

**PLAT plan:** [rt_plat_v3_p2_workstation_layout_plan.md](../platform/rt_plat_v3_p2_workstation_layout_plan.md)

**Freeze:** [rt_plat_v3_p2_freeze_audit.md](rt_plat_v3_p2_freeze_audit.md) — **PLAT-RT-V3 complete**

---

## Regression matrix (all PLAT phases)

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## Stop line

Do not start PLAT-RT-V3 without:

1. PLAN-RT-V3 frozen  
2. Per-phase PLAT plan in `docs/platform/`  
3. Governance review + freeze audit per phase  
4. No change to frozen V1/V2/F4 default-on without new PLAN wave  
