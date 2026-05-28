# RT-V3 — Architecture Review R1

**Phase:** PLAN-RT-V3 — runtime visualization fidelity (read-only)  
**Plan:** [rt_v3_runtime_visualization_fidelity_plan.md](../platform/rt_v3_runtime_visualization_fidelity_plan.md)  
**Contracts:** [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md), [rt_cesium_workstation_visualization_v3_v1.md](rt_cesium_workstation_visualization_v3_v1.md)  
**Freeze audit:** [rt_v3_freeze_audit.md](rt_v3_freeze_audit.md)  
**Baseline:** [rt_c2_platform_consolidation_review_r1.md](rt_c2_platform_consolidation_review_r1.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Layering integrity | **Pass** — display-only extensions on frozen pull path |
| Contract completeness | **Pass** — layer registry + workstation annex defined |
| Overlap with frozen waves | **Pass** — additive; no authority path changes |
| Concentration risk | **Pass-with-conditions** — `App.tsx` / `cesium/` integration |
| PLAT readiness | **Conditional** — P0 registry before overlays |

**Recommendation:** Freeze **PLAN-RT-V3** (docs only). **Advisory next:** **PLAT-RT-V3 P0**.

---

## 1. Layering model

```text
Browser (rt-sandbox-ui)
    → GET /v1/telemetry/pull (unchanged)
    → UI derive (telemetry, terrain sample, F5b optional)
    → Cesium display + cognition strips (V3 registry governs toggles)
    → Bridge commands (unchanged) ← registry authority
```

| Check | Result |
|-------|--------|
| No new HTTP endpoints | **Pass** |
| No new bridge commands in PLAN | **Pass** |
| Display-only overlays | **Pass** |
| One globe / selected session | **Pass** |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| V3-ARCH-01 | Pass | Pull → derive → display chain unchanged |
| V3-ARCH-02 | Pass | V3 layers cannot mutate registry |

---

## 2. Overlap matrix

| Frozen wave | V3 relationship | Assessment |
|-------------|-----------------|------------|
| V1 markers/bounds/camera | Extended LOD + session dimming | **Intentional** |
| V2 terrain mesh/ridges | Registry entries + label stacking | **Intentional** |
| F4 contours/LOS/vegetation | Registry + stacked_los_v3 presentation | **Intentional** — F4 defaults preserved |
| F5b fidelity truth | Separate `fidelity_block` in hub | **Intentional** — no merge |
| M3 poll/diagnostics | Compact diagnostic row | **Intentional** |
| T3/T5 Cesium edit | Layer rail coexists with edit surface | **Pass-with-conditions** — layout density |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| V3-ARCH-03 | Pass | No duplicate authority paths |
| V3-ARCH-04 | Pass-with-conditions | Workstation density increases — mentor/demo still target |

---

## 3. Module concentration

| Module | Scale (C2 baseline) | V3 touch |
|--------|---------------------|----------|
| `App.tsx` | ~725 LOC | Grouped hub blocks, diagnostic row |
| `src/cesium/` | ~29 files | New layer modules P1 |
| `CesiumRuntimePanel.tsx` | ~533 LOC | Layer rail + banners |
| `RuntimeCognitionHub.tsx` | moderate | Block ordering |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| V3-ARCH-05 | Pass-with-conditions | Defer `App.tsx` split to P2 or optional maintenance |
| V3-ARCH-06 | Pass | P0 registry isolates toggle policy before overlay code |

---

## 4. V3 architecture summary

PLAN-RT-V3 introduces:

1. **`rt_visual_layer_registry_v3`** — canonical layer ids, z-order, defaults, performance budget  
2. **Visibility overlay pack** — wedge, horizon, stacked LOS (default off)  
3. **Grouped cognition taxonomy** — terrain / visibility / sensor / fidelity blocks  
4. **Workstation layout annex** — cognition rail, compact background diagnostics, multi-session chrome rules  

All surfaces remain **explanatory** except F5b-labeled truth lines when coupling is enabled.

---

## 5. PLAT phase advisory

| Phase | Risk | Dependency |
|-------|------|------------|
| P0 | Low | Registry + fixture + vitest |
| P1 | Med | New Cesium entities; performance budget |
| P2 | Med | `App.tsx` wiring; layout regression |

**Stop line:** No P1 without P0 freeze.

---

## Related

- [rt_v3_governance_review_r1.md](rt_v3_governance_review_r1.md)
- [rt_roadmap_plat_rt_v3_v1.md](rt_roadmap_plat_rt_v3_v1.md)
