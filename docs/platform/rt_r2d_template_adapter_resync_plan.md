# RT-R2d — Template Adapter Resync Policy (PLAT-RT-R2d)

**Phase:** PLAT-RT-R2d — P1 workflow continuity closure  
**Prerequisite:** PLAT-RT-R1b frozen; PLAT-RT-S6 frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Close R1-AUTH-04: explicit adapter resync policy after template/workflow registry mutations.

## Allowed

- Module: `template_resync.py` — `run_template_adapter_resync`
- Hook `apply_runtime_template`, `advance_workflow` (`apply_template`, `reset_world`)
- Refactor `adapter_resync` subcommand through shared helper
- Audit: `template_resync_requested`, `template_resync_completed`, `template_resync_skipped`, `template_resync_stale`
- Contract: [rt_template_resync_policy_v1.md](../evaluation/rt_template_resync_policy_v1.md)
- Additive tests

## Forbidden

- Telemetry UI, Cesium, SA integration, distributed/autonomous runtime
- New bridge commands
- Blocking template apply on resync stale (registry remains authoritative)
- Auto-resync on `reload_workflow` / `reset_workflow` (no registry mutation)
- Session manager decomposition (R3a)

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

Do not start R2e (capture pose cognition) or expansion waves until PLAT-RT-R2d is frozen.

## Related

- [rt_r2d_governance_review_r1.md](../evaluation/rt_r2d_governance_review_r1.md)
- [rt_r2d_freeze_audit.md](../evaluation/rt_r2d_freeze_audit.md)
