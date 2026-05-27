# RT-R3b — Lifecycle Documentation Hardening (PLAT-RT-R3b)

**Phase:** PLAT-RT-R3b — P2 maintenance / lifecycle hardening (R1-LIFE-02)  
**Prerequisite:** PLAT-RT-R3a frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Harden lifecycle semantics and eliminate ambiguity around bridge/runtime transitions. Closes roadmap item **RT-R3b** / finding **R1-LIFE-02**. **`bridge_disconnected` session state is docs + tests only** — not implemented in handlers.

## Allowed

| Item | Notes |
|------|-------|
| Contract | [rt_lifecycle_transitions_v1.md](../evaluation/rt_lifecycle_transitions_v1.md) — implementation truth table |
| Lifecycle doc updates | `rt_session_lifecycle_v1.md`, sync/boundary/governance cross-links |
| Audit vocabulary | Lifecycle audit table (§3.1) |
| `transition_rules()` helper | Read-only export from `lifecycle.py` |
| Additive transition tests | `can_transition` matrix + failure/cleanup integration |

## Forbidden

- Implement `bridge_disconnected` reconnect timer or HTTP disconnect detection
- Telemetry UI, Cesium, SA integration, Gazebo expansion
- New bridge commands; wiring `bridge_disconnected_reconnect_timeout` to `GovernanceConfig`
- R3c/R3d scope

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

Do not start R3c (governance lint), telemetry UI, Cesium, or SA bridge until PLAT-RT-R3b is frozen.

## Related

- [rt_r3b_governance_review_r1.md](../evaluation/rt_r3b_governance_review_r1.md)
- [rt_r3b_freeze_audit.md](../evaluation/rt_r3b_freeze_audit.md)
