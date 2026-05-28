# RT-V4 P0 - Visualization Realism Review R1

**Phase:** PLAT-RT-V4 P0
**Plan:** [rt_plat_v4_p0_visual_registry_plan.md](../platform/rt_plat_v4_p0_visual_registry_plan.md)
**Baseline:** [rt_v4_visualization_realism_review_r1.md](rt_v4_visualization_realism_review_r1.md)

## Purpose

Assess misread risk for P0 density controls and session comparison primitives. P0 intentionally avoids P1 terrain/visibility overlays.

## Misread risk

| Feature | Risk | Mitigation |
|---------|------|------------|
| Density warnings | Low-Med | Labeled warn-only/advisory; no automatic enforcement |
| Budget summary | Low | Explanatory layer count only |
| Session contrast | Low | Selected session remains commandable; background rows are visual |
| Comparison ghosts toggle | Med | Default off; label says explanatory only and no cross-session commands |

## P1 separation

P0 does not implement:

- visibility corridors
- occlusion confidence bands
- terrain relation overlays
- new Cesium geometry
- new sensor coverage visuals

## Realism verdict

**Pass-with-conditions.** P0 is safe to freeze because it introduces display policy and compare cognition only. P1 must repeat realism review before adding any new visual overlays.
