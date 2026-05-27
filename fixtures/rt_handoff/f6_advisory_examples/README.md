# F6 Advisory Derivation Examples

Reference inputs for PLAT-RT-F6 P0 golden tests. Each file documents expected `rt_sa_workflow_advisory_status_v1` output when derive runs against the described staging signals.

**Not authoritative staging** — documentation fixtures only.

| File | Scenario | Expected `advisory_state` | Full expected |
|------|----------|---------------------------|---------------|
| `capture_ready_normalized.json` | Normalized capture, review not started | `capture_ready` |
| `export_handoff_ready_pre_approve.json` | Export event `handoff_ready` but not `handoff_reviewed` | `capture_ready` (not advisory `handoff_ready`) |
| `review_complete.json` | `handoff_reviewed` present | `review_complete` |
| `approval_ready.json` | Review complete, preconditions pass, not approved | `approval_ready` |
| `handoff_ready_post_approve.json` | Approved + conversion manifest | `handoff_ready` |
| `import_ready_prepared.json` | `handoff_import_prepared`, pipeline logs present | `import_ready` |
| `blocked_rejected.json` | `handoff_rejected` | `blocked` |
| `blocked_deferred.json` | `handoff_import_deferred` | `blocked` |
| `f5_eligible_import_not_ready.json` | F5 experiment eligible but capture not prepared | F5 warn; advisory below `import_ready` |
| `sa2_ready_maps_approval_ready.json` | SA2 `workflow_phase: ready` pre-approve | `approval_ready` |
| `committed_terminal.json` | `handoff_import_committed` | terminal — not an advisory rung |

See [rt_sa_workflow_automation_v1.md](../../docs/evaluation/rt_sa_workflow_automation_v1.md) for derivation rules.

Full golden outputs: `expected/<fixture-stem>.expected.json` (used by vitest + pytest).
