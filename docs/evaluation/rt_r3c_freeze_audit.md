# RT-R3c — Runtime Subcommand Governance Lint Freeze Audit (PLAT-RT-R3c)

## Scope

- [rt_r3c_subcommand_governance_lint_plan.md](../platform/rt_r3c_subcommand_governance_lint_plan.md)
- [rt_runtime_subcommand_registry_v1.md](rt_runtime_subcommand_registry_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/governance.py` — `RUNTIME_SUBCOMMANDS_RESERVED`, `RUNTIME_SUBCOMMAND_AUDIT_EXCEPTIONS`
- `platform/rt-sandbox-bridge/rt_sandbox/runtime_subcommand_governance.py`
- `scripts/rt/lint_rt_runtime_subcommands.py`
- `scripts/ci_eval.sh` — `tier0` lint gate
- Additive test in `test_rt_sandbox_bridge.py`
- [rt_r3c_governance_review_r1.md](rt_r3c_governance_review_r1.md)

Not in scope: runtime behavior changes, new subcommands, telemetry UI, Cesium, SA integration, R3d.

Prerequisite: PLAT-RT-R3b frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-R3c.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| No new bridge commands | Pass |
| No runtime behavior change | Pass — lint/docs/constants only |
| RT↔SA export boundary unchanged | Pass |
| P2 roadmap closure (R1-GOV-04) | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `rt_runtime_subcommand_registry_v1.md` | Yes |
| 2 | `runtime_subcommand_governance.py` | Yes |
| 3 | `scripts/rt/lint_rt_runtime_subcommands.py` | Yes |
| 4 | `tier0` CI integration | Yes |
| 5 | Governance constants in `governance.py` | Yes |
| 6 | Bridge/governance/audit/ownership doc updates | Yes |
| 7 | Governance review R1 | Yes |
| 8 | Freeze audit (this document) | Yes |
| 9 | Additive lint test | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
# lint_rt_runtime_subcommands OK (7 subcommands)

python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
# 119 passed

scripts/ci_eval.sh tier0
# tier0: OK
```

119 passed at freeze time (118 prior + 1 R3c governance lint test).

## Lint coverage summary

| Category | Count | Details |
|----------|-------|---------|
| Active subcommands | 7 | `adapter_attach`, `adapter_detach`, `adapter_health`, `adapter_poll_feedback`, `adapter_poll_telemetry`, `adapter_resync`, `mock_inject_drift` |
| Reserved forbidden | 2 | `reload_world_config`, `set_clock_pause` |
| Audit exceptions | 1 | `adapter_resync` → `sync_update` |
| Lint checks | 5 | allow-list ⊆ handlers; handlers ⊆ allow-list; reserved ∩ allow-list = ∅; audit classification; audit exception keys valid |

## Remaining P2 items

| Wave | Finding | Objective |
|------|---------|-----------|
| **RT-R3d** | R1-SYNC-05 | World revision hint policy — document when hint may diverge from `world.revision` |

## Stop Line

PLAT-RT-R3c frozen. Next documented P2 action: **RT-R3d** revision hint policy — not telemetry UI, Cesium, or SA bridge implementation.
