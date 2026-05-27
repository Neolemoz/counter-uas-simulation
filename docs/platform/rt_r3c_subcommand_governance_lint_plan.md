# RT-R3c — Runtime Subcommand Governance Lint (PLAT-RT-R3c)

**Phase:** PLAT-RT-R3c — P2 maintenance / governance lint (R1-GOV-04)  
**Prerequisite:** PLAT-RT-R3b frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Add governance enforcement so runtime subcommand definitions stay synchronized with implementation. Closes roadmap item **RT-R3c** / finding **R1-GOV-04**.

## Allowed

| Item | Notes |
|------|-------|
| Contract | [rt_runtime_subcommand_registry_v1.md](../evaluation/rt_runtime_subcommand_registry_v1.md) |
| Lint module | `runtime_subcommand_governance.py` — AST handler discovery + drift checks |
| CLI | `scripts/rt/lint_rt_runtime_subcommands.py --check` |
| CI | `scripts/ci_eval.sh tier0` integration |
| Governance constants | `RUNTIME_SUBCOMMANDS_RESERVED`, `RUNTIME_SUBCOMMAND_AUDIT_EXCEPTIONS` in `governance.py` |
| Additive lint test | `test_runtime_subcommand_governance_lint_passes` |
| Doc cross-links | bridge, governance, audit, ownership, poll semantics |

## Forbidden

- Runtime behavior changes (handler logic, lifecycle gating)
- New subcommands or bridge commands
- Telemetry UI, Cesium, SA integration, Gazebo expansion
- R3d scope (revision hint policy)

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
scripts/ci_eval.sh tier0
```

## Stop line

Do not start R3d (revision hint policy), telemetry UI, Cesium, or SA bridge until PLAT-RT-R3c is frozen.

## Related

- [rt_r3c_governance_review_r1.md](../evaluation/rt_r3c_governance_review_r1.md)
- [rt_r3c_freeze_audit.md](../evaluation/rt_r3c_freeze_audit.md)
