# RT-R3a — Session Manager Decomposition Freeze Audit (PLAT-RT-R3a)

## Scope

- [rt_r3a_session_manager_decomposition_plan.md](../platform/rt_r3a_session_manager_decomposition_plan.md)
- [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/session_manager.py` (facade)
- New modules: `session_record.py`, `session_response.py`, `session_teardown.py`, `session_adapter_results.py`, `session_runtime_commands.py`, `session_entity_handlers.py`, `session_telemetry_coordinator.py`, `session_workflow_handlers.py`, `session_capture_handler.py`, `session_lifecycle_handlers.py`
- [rt_r3a_governance_review_r1.md](rt_r3a_governance_review_r1.md)

Not in scope: telemetry UI, Cesium, SA integration, R3b/R3c/R3d, behavior changes.

Prerequisite: PLAN-RT-R2f frozen; PLAT-RT-R1a–R2e frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-R3a.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Command failure semantics preserved | Pass |
| P2 roadmap closure (R1-DEBT-01) | Pass |
| No new bridge commands | Pass |
| R1a–R2e authority/audit semantics preserved | Pass |
| RT↔SA export boundary unchanged | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `session_record.py` | Yes |
| 2 | `session_response.py` | Yes |
| 3 | `session_teardown.py` — unified cleanup | Yes |
| 4 | `session_adapter_results.py` | Yes |
| 5 | `session_runtime_commands.py` | Yes |
| 6 | `session_entity_handlers.py` | Yes |
| 7 | `session_telemetry_coordinator.py` | Yes |
| 8 | `session_workflow_handlers.py` | Yes |
| 9 | `session_capture_handler.py` | Yes |
| 10 | `session_lifecycle_handlers.py` | Yes |
| 11 | `session_manager.py` facade (~433 lines) | Yes |
| 12 | `rt_session_manager_ownership_v1.md` | Yes |
| 13 | Governance review R1 | Yes |
| 14 | Freeze audit (this document) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

99 passed at freeze time (no new tests — behavior frozen).

## Decomposition summary

| Metric | Before | After |
|--------|--------|-------|
| `session_manager.py` lines | ~2042 | ~433 |
| Handler modules | 0 (monolith) | 10 internal modules |
| Duplicated full teardown paths | 3+ | 1 (`session_teardown.py`) |
| Duplicated adapter mirror clear | 5+ | 1 (`clear_adapter_mirrors`) |

Cleanup paths normalized: capture, discard, auto-cleanup (STOPPED vs FAILED/RUNTIME_CRASHED) share composed helpers while preserving partial-teardown semantics for failed/crashed sessions.

## Remaining P2 items

| Wave | Finding | Objective |
|------|---------|-----------|
| **RT-R3b** | R1-LIFE-02 | Lifecycle doc hardening — **closed by PLAT-RT-R3b** |
| **RT-R3c** | R1-GOV-04 | Subcommand governance lint — CI check `RUNTIME_SUBCOMMANDS` ⊆ handlers |
| **RT-R3d** | R1-SYNC-05 | World revision hint policy documentation |

Expansion waves (telemetry UI, Cesium, SA import implementation) remain forbidden until explicit new wave audit.

## Stop Line

PLAT-RT-R3a frozen. Successor: **PLAT-RT-R3b** (frozen). Next P2 action: **RT-R3c** governance lint.
