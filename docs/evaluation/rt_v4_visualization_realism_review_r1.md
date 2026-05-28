# RT-V4 - Visualization Realism Review R1

**Phase:** PLAN-RT-V4 - visualization fidelity planning  
**Plan:** [rt_v4_visualization_fidelity_plan.md](../platform/rt_v4_visualization_fidelity_plan.md)  
**Contracts:** [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md), [rt_visual_density_management_v1.md](rt_visual_density_management_v1.md), [rt_visual_multi_session_cognition_v1.md](rt_visual_multi_session_cognition_v1.md)  
**Baseline:** [rt_v3_visualization_realism_review_r1.md](rt_v3_visualization_realism_review_r1.md), [rt_f4_realism_review_r1.md](rt_f4_realism_review_r1.md)

## Purpose

Assess whether V4 visualization planning could cause reviewers to mistake richer visual cues for sensor truth, command authority, SA import readiness, or operational status.

## Misread risk assessment

| Feature | Risk | Mitigation |
|---------|------|------------|
| Visibility corridors | **Med** - corridor shape can imply coverage | Default off; label as heuristic sandbox cue |
| Occlusion confidence bands | **Med** - confidence can imply probability | Qualitative only; no numeric probability or readiness score |
| Terrain relation labels | **Low-Med** - may imply surveyed terrain | Fictional/explanatory label preserved |
| Session comparison ghosts | **Med** - can imply commandable background session | Muted/dashed, `comparison` label, no command affordance |
| Density summaries | **Low** - can hide detail | Label as summarized explanatory labels |
| Background diagnostic cohesion | **Low-Med** - stale chips can read as health | Avoid health/readiness language |
| F5b coexistence | **Med** | Preserve `truth_attested` and `explanatory` label separation |

## Default policy

| V4 feature class | Default |
|------------------|---------|
| New visibility/terrain overlays | Off |
| Comparison globe overlays | Off |
| Density summaries | On only as display simplification |
| Background diagnostic cohesion | Display-only; no actions |

## Required PLAT conditions

Future PLAT-RT-V4 must:

1. Keep V4-new visual geometry default off until reviewed.
2. Use explanatory labels on every terrain/visibility cognition surface.
3. Preserve selected-session command isolation in comparison views.
4. Avoid `readiness`, `health`, `coverage proof`, `operational picture`, and `auto-sync` language.
5. Preserve F5b label discipline when truth-coupling fields are visible.

## Realism verdict

**Pass-with-conditions - approve PLAN freeze.**

The PLAN artifacts are safe to freeze because they define realism and misread controls without implementing visual surfaces. Future PLAT phases carry the residual misread risk.

## Related

- [rt_v4_governance_review_r1.md](rt_v4_governance_review_r1.md)
- [rt_v4_freeze_audit.md](rt_v4_freeze_audit.md)
