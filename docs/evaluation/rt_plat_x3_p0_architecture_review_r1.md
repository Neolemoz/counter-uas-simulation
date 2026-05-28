# RT-X3 P0 — Architecture Review R1 (PLAT-RT-X3 P0)

**Phase:** PLAT-RT-X3 P0 — workbench v3 shell  
**Plan:** [rt_plat_x3_p0_workbench_v3_shell_plan.md](../platform/rt_plat_x3_p0_workbench_v3_shell_plan.md)

## Summary

P0 layers v3 ergonomics inside the frozen v2 workstation grid without relocating authority controls. `ExperimentWorkbenchV3Shell` stacks governance banner, program context, manifest roster, and explicit secondary picker above the existing cohort navigator / unified review / report dock columns. Compare ergonomics are additive: mode coach strip, `formatCompareStatus` on multi-manifest rows, and drill-down buttons that only update `primary_manifest_ref` / `secondary_manifest_ref` in UI-local state.

## Concentration

New logic lives in dedicated modules (~15 files). `ExperimentWorkbenchV2Shell` remains the integration point; `ExperimentWorkbenchPanel` unchanged. Packet `sections[]` preview is UI-only; export path unchanged.

## Risks (accepted)

- Tag filter matches `experiment_class` when cohort `tags[]` clicked (manifest refs lack per-row tags).
- `review_session_id` persisted in same localStorage key as v2 state (additive fields).

## Recommendation

Freeze **PLAT-RT-X3 P0**. Proceed to **PLAT-RT-X3 P1** only with contamination review for F6/F7 adjacency.
