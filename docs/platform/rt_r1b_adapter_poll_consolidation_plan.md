# RT-R1b — Adapter Poll & Telemetry Path Consolidation (PLAT-RT-R1b)

**Phase:** PLAT-RT-R1b — P1 consolidation closure  
**Prerequisite:** PLAT-RT-R1a frozen; PLAT-RT-S2–S6, PLAT-RT-G2–G5 frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Reduce duplication and unify runtime adapter polling + telemetry flow without feature expansion. Closes roadmap items **RT-R2a**, **RT-R2b**, and partial **RT-R2c** under the **PLAT-RT-R1b** wave label.

## Allowed

- Module: `adapter_poll.py` — unified `run_adapter_poll_tick`
- Module: `time_utils.py` — shared `parse_utc`, `poll_age_seconds`, `is_poll_stale`
- `sync_audit_event()` helper in `adapter_sync.py`
- Session manager poll routing simplification (`skip_adapter_poll`, `_apply_adapter_poll_result`)
- Contract: [rt_poll_sync_semantics_v1.md](../evaluation/rt_poll_sync_semantics_v1.md)
- Cross-links in [rt_adapter_feedback_v1.md](../evaluation/rt_adapter_feedback_v1.md), [rt_adapter_telemetry_v1.md](../evaluation/rt_adapter_telemetry_v1.md)
- Additive tests

## Forbidden

- Telemetry UI, Cesium, SA integration, distributed/autonomous runtime
- New bridge commands
- `apply_runtime_template` resync policy (R2d)
- Session manager module decomposition (R3a)
- Removing `TelemetryBuffer`
- Authority label / audit `command_type` renames
- Parser/topic/schema changes

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

Do not start R2d (template resync), telemetry UI, Cesium, or SA bridge until PLAT-RT-R1b is frozen.

## Related

- [rt_r1b_governance_review_r1.md](../evaluation/rt_r1b_governance_review_r1.md)
- [rt_r1b_freeze_audit.md](../evaluation/rt_r1b_freeze_audit.md)
