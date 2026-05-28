# RT-V3 — Visualization Realism Review R1

**Phase:** PLAN-RT-V3 — visualization fidelity planning  
**Plan:** [rt_v3_runtime_visualization_fidelity_plan.md](../platform/rt_v3_runtime_visualization_fidelity_plan.md)  
**Contracts:** [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md), [rt_cesium_workstation_visualization_v3_v1.md](rt_cesium_workstation_visualization_v3_v1.md)  
**Baseline:** [rt_f4_realism_review_r1.md](rt_f4_realism_review_r1.md)

## Purpose

Assess **misread risk** for V3 visualization features — whether reviewers, mentors, or maintainers could confuse heuristic overlays with survey data, sensor coverage, or operational truth.

---

## Misread risk assessment

| Feature | Risk | Mitigation |
|---------|------|------------|
| `visibility_wedge_v3` | **Med** — fan shape implies sensor footprint | Default **off**; label “heuristic visibility wedge — not sensor coverage”; `BANNER_VISIBILITY_V3` |
| `horizon_hint_v3` | **Low–Med** — horizon line implies survey | Default off; “fictional horizon cue — not terrain survey” |
| `stacked_los_v3` | **Med** — stacked contour + LOS reads as GIS | Retain F4 contour disclaimer; default off |
| Ridge label stacking | **Low** | Density limits; fictional ridge names only |
| Grouped sensor block | **Med** — dome count implies coverage | Retain V2 “nominal — not coverage proof” |
| Inactive session dimming | **Low** | Cosmetic; tabs still show slot state |
| Compact diagnostic row | **Low** | Pull age only — no health scoring |
| F5b dual readout on globe card | **Med** | Mandatory `explanatory` + `truth_attested` badges per F5b contract |
| Performance budget warnings | **Low** | Maintainer-facing; not ops readiness |

---

## Comparison to V2 and F4

| Wave | Established |
|------|-------------|
| V2 | Fictional terrain, occlusion markers, nominal domes |
| F4 | Contours default off, LOS heuristic, vegetation cues |
| V3 | **Integration polish** — registry, wedge/horizon, grouped cognition, workstation layout |

V3 does not introduce new heightmap authority or sensor truth channels. It **coordinates** frozen layers for mentor/demo readability.

---

## Default policy review

| Layer class | Default | Verdict |
|-------------|---------|---------|
| V3-new overlays | off | **Approve** |
| V2 terrain mesh / ridges | on (frozen) | **No change in PLAN** |
| F4 contours | off (frozen) | **No change in PLAN** |

---

## Lexicon audit

Contract scan: no use of “detected”, “tracked”, “neutralized”, “readiness”, “operational picture”, or “cleared” as factual claims on V3 overlays.

| Check | Result |
|-------|--------|
| Forbidden lexicon | **Pass** |
| Banner text proposed | **Pass** — conservative |

---

## Realism verdict

**Pass-with-conditions — approve PLAN freeze.**

Conditions for PLAT:

1. Enforce default-off on all V3-new `layer_id` rows.  
2. Ship `BANNER_VISIBILITY_V3` before enabling wedge in demo fixtures.  
3. Vitest: overlay toggle does not hide F5b truth badges when coupling on.  

**Recommendation:** Proceed to **PLAT-RT-V3 P0** after PLAN freeze with realism review cited in PLAT governance packet.

---

## Related

- [rt_v3_governance_review_r1.md](rt_v3_governance_review_r1.md)
- [rt_v3_freeze_audit.md](rt_v3_freeze_audit.md)
