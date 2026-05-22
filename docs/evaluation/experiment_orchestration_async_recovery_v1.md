# Experiment Orchestration Async Recovery (`experiment_orchestration_async_recovery_v1`)

**Phase:** PLAT-SA-I3 — reviewer-facing recovery semantics  
**Authority:** [AGENTS.md](../../AGENTS.md); extends [experiment_orchestration_async_model_v1.md](experiment_orchestration_async_model_v1.md) §6

Recovery is **explanatory** and **offline-first**. CLI/workers remain authoritative; the viewer mirrors recovery artifacts only.

---

## Retry lineage semantics

A **retry chain** is an ordered, append-only sequence:

1. `queue_claim_token_v1` (open → closed)
2. `worker_execution_record_v1` entries (attempt 1, 2, … via `retry_parent_ref`)
3. `async_lineage[]` status events (`failed` → `retrying` → terminal or `quarantined`)

Rules:

- Each retry attempt references its parent worker record when applicable.
- `retrying` without a prior `failed` lineage event is an audit finding (`retry_lineage_gap`).
- Lineage events are never deleted or rewritten in place.

---

## Deterministic recovery lifecycle

| Stage | Async status | Reviewer action |
|-------|--------------|-----------------|
| Detect fault | `failed` | Inspect worker record + partial execution audit |
| Bounded retry | `retrying` | New worker attempt; new claim if required |
| Integrity hold | `quarantined` | Review recovery report; clear via scoped CLI wave only |
| Supersede snapshot | `superseded` | Read-only; successor snapshot owns forward reconcile |

Recovery principle (from I2): **reconcile forward** — new snapshot, new report, additive lineage — never mutate historical queue snapshots or audit reports.

---

## Failed execution immutability

Failed worker records and lineage entries remain auditable indefinitely. Supersession marks prior snapshots `superseded` but does not delete worker refs or claims.

---

## Quarantine review flow

1. Async integrity or recovery audit sets or documents `quarantined`.
2. `promote_experiment_manifest.py` blocks forward I1 transitions while quarantined.
3. Reviewer reads `orchestration_recovery_report_v1` and batch audit summary.
4. Clearance requires explicit CLI status change after human review — no browser-triggered clear.

---

## Related contracts

- [experiment_orchestration_replay_reconciliation_v1.md](experiment_orchestration_replay_reconciliation_v1.md)
- [experiment_orchestration_async_recovery_governance_v1.md](experiment_orchestration_async_recovery_governance_v1.md)
- [orchestration_recovery_report_v1.md](orchestration_recovery_report_v1.md)

*End of experiment orchestration async recovery v1.*
