# Experiment Orchestration Async Recovery Governance (`experiment_orchestration_async_recovery_governance_v1`)

**Phase:** PLAT-SA-I3 — governance-safe recovery semantics  
**Authority:** [AGENTS.md](../../AGENTS.md)

---

## Rules

| Rule | Enforcement |
|------|-------------|
| Recovery is explanatory | All I3 artifacts include `governance.notice`; no parser contract |
| Supersession additive-only | Audits error on in-place snapshot/report mutation; `superseded` read-only |
| Failed executions auditable | Worker records + lineage never deleted |
| Quarantined artifacts reviewable | Viewer shows hold; I2 promote guard unchanged |
| Lineage immutable | `async_lineage` append-only; `retry_lineage_gap` audit on violations |

---

## Audit dimensions (recovery plane)

| Dimension | Detection | Severity (strict) |
|-----------|-----------|-------------------|
| Retry lineage gap | `retrying` without prior `failed` in lineage | Error |
| Superseded claim | Open claim on `superseded` async | Error |
| Replay replacement gap | Success path without replay fingerprint or quarantine | Error |
| Fingerprint pair mismatch | Worker execution vs async stored mismatch | Error |
| Recovery continuity | Combined async + replay + partial execution | Error |

I2 `audit_orchestration_async_integrity.py --strict` and I1 `audit_orchestration_integrity.py --strict` remain required separately.

---

## Reconciliation rules (reviewer narrative)

1. **Supersede, don't patch** — from [experiment_orchestration_async_governance_v1.md](experiment_orchestration_async_governance_v1.md).
2. **Quarantine before promote** — unchanged I2 guard.
3. **Orphan claims** — CLI close/revoke with audit entry; no worker self-heal.
4. **Stale replay** — hold or downgrade via explicit CLI wave; regen never automatic from viewer.

---

## CI registration

- `audit_orchestration_recovery.py --strict`
- Platform integrity: `orchestration_recovery_integrity`
- tier0-sa-r0 includes recovery audit + recovery pytest

*End of experiment orchestration async recovery governance v1.*
