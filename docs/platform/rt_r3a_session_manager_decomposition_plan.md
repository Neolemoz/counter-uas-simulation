# RT-R3a — Session Manager Decomposition (PLAT-RT-R3a)

**Phase:** PLAT-RT-R3a — P2 maintenance / architecture hardening (R1-DEBT-01)  
**Prerequisite:** PLAN-RT-R2f frozen; PLAT-RT-R1a–R2e frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Reduce `session_manager.py` concentration and improve maintainability without changing runtime behavior. Closes roadmap item **RT-R3a** / finding **R1-DEBT-01**.

## Allowed

| Item | Notes |
|------|-------|
| Module split | `session_record.py`, `session_response.py`, `session_teardown.py`, `session_adapter_results.py`, `session_runtime_commands.py`, `session_entity_handlers.py`, `session_telemetry_coordinator.py`, `session_workflow_handlers.py`, `session_capture_handler.py`, `session_lifecycle_handlers.py` |
| Teardown unification | `clear_adapter_mirrors`, composed discard/capture/auto-cleanup paths |
| Ownership contract | [rt_session_manager_ownership_v1.md](../evaluation/rt_session_manager_ownership_v1.md) |
| Facade preservation | `BridgeSessionManager`, `handle_command`, `pull_telemetry`, `GovernanceConfig` re-export |
| Comments / naming cleanup | Internal module organization only |

## Forbidden

- Telemetry UI, Cesium, SA integration, replay auto-ingestion
- Multi-session, distributed/autonomous runtime, Gazebo expansion
- New bridge commands; governance constant renames; authority label changes
- Behavior changes unless required for correctness (none expected)
- R3b/R3c/R3d scope (lifecycle docs, governance lint, revision hints)

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

Do not start R3b (lifecycle docs), telemetry UI, Cesium, or SA bridge until PLAT-RT-R3a is frozen.

## Related

- [rt_r3a_governance_review_r1.md](../evaluation/rt_r3a_governance_review_r1.md)
- [rt_r3a_freeze_audit.md](../evaluation/rt_r3a_freeze_audit.md)
