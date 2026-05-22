# Experiment Orchestration Async Governance (`experiment_orchestration_async_governance_v1`)

**Phase:** PLAN-SA-I2 — CI constraints and async integrity model (docs only)  
**Authority:** [AGENTS.md](../../AGENTS.md); [sa_i2_async_orchestration_plan.md](../platform/sa_i2_async_orchestration_plan.md)

CI orchestration boundaries and **future** async integrity audit dimensions. Implements nothing in PLAN-SA-I2.

---

## 1. CI orchestration boundaries (I2.5)

Aligns with frozen H3/I1 and `experiment_orchestration.py` `runtime_capture` gating.

| Rule | Detail |
|------|--------|
| Optional/offline CI capture | Default CI path remains synthetic / dry-run; live Gazebo capture is never default |
| Explicit flags only | `runtime_capture` steps require `--allow-runtime-capture` on `run_experiment_queue.py` |
| No automatic runtime authority escalation | `tier0-sa-r0` must not implicitly enable live capture or async workers |
| Bounded execution environments | Planning limits: job timeout, max concurrent claims per tenant, resource caps documented per manifest policy (constants in PLAT-SA-I2+ only) |
| Single-tenant queue per job | CI job receives one manifest + one queue snapshot; no implicit multi-tenant fan-out |

### CI async worker lane (future, not authorized)

When PLAT-SA-I2+ adds CI workers:

- Separate opt-in flag (e.g. `--allow-async-worker`) — never default
- Worker identity recorded in `worker_execution_record_v1`
- Same fingerprint and audit rules as local CLI execution
- CI artifacts remain under `fixtures/` or CI export paths — not parser contracts

### Federation deferral

Multi-corpus / multi-tenant queues ([sa_platform_governance_review_r1.md](sa_platform_governance_review_r1.md) corpus-scale risk) remain **out of scope** until a dedicated federation wave with registry discipline.

---

## 2. Async integrity and audit model (I2.6)

Extends I1 [replay_sa_orchestration_integrity.py](../../scripts/evaluation/replay_sa_orchestration_integrity.py) dimensions for **future** `audit_orchestration_async_integrity.py` (PLAT-SA-I2+).

### Audit dimensions

| Dimension | Detection rule | Severity (strict) |
|-----------|----------------|-------------------|
| Orphan worker detection | `queue_claim_token_v1` exists without matching queue terminal state or manifest ref | Error |
| Stale replay detection | Bundle SHA-256 ≠ ops-recorded replay fingerprint at `replay_generated` | Error |
| Partial execution recovery | Step `running` in queue/report without terminal report closure | Error |
| Replay continuity auditing | I1 `verify_replay_outputs` + worker ref present when async claim used | Error if claim without bundle |
| Queue reconciliation | Active claim on `superseded` snapshot | Error |
| Duplicate claim | Two open claims same `(snapshot_hash, manifest_id)` | Error |
| Viewer mirror drift | Async integrity flags stale vs CLI audit output | Warning |

### Reconciliation rules

1. **Supersede, don't patch:** New queue snapshot supersedes prior; prior marked `superseded` (read-only).
2. **Quarantine before promote:** `quarantined` blocks `promote_experiment_manifest.py` forward transitions until CLI audit clears.
3. **Orphan claims:** CLI revokes or closes claim with audit entry; no worker self-heal.
4. **Stale replay:** Downgrade ops to `pending` or hold `quarantined`; regen only via explicit CLI wave.

### Proposed `orchestration_async_integrity_report_v1` (schema sketch)

```json
{
  "artifact_type": "orchestration_async_integrity_report_v1",
  "schema_version": "1",
  "generated_at": "ISO-8601 UTC",
  "issues": [{ "kind": "orphan_claim", "manifest_id": "...", "message": "..." }],
  "warnings": [],
  "governance": {
    "notice": "Explanatory audit only. Not parser-visible authority.",
    "anti_claims": ["not operational readiness"]
  }
}
```

Sync to viewer via future extension of `sync_orchestration_mirrors.py` — not in PLAN-SA-I2.

---

## 3. Relationship to I1 integrity audit

| I1 check (today) | Async extension (future) |
|------------------|--------------------------|
| Orphan queue without manifest | Orphan claim without queue |
| `manifest_fingerprint` stale | `execution_fingerprint` stale |
| Missing ops sidecar | Missing claim token when async enabled |
| Authoring handoff symmetry | Worker provenance ↔ authoring pack refs |
| Viewer mirror index | Async integrity report mirror |

I1 `audit_orchestration_integrity.py --strict` remains required and unchanged in PLAN-SA-I2.

---

## 4. Manual governance review checklist

Before PLAT-SA-I2 implementation:

- [ ] Boundary matrix all Pass ([sa_i2_async_orchestration_governance_review_r1.md](sa_i2_async_orchestration_governance_review_r1.md))
- [ ] No doc conflates mirrors with authority
- [ ] No parser/schema change proposals in async specs
- [ ] Federation explicitly deferred
- [ ] `governance_lint_sa.py` clean on new markdown

---

## Related

- [experiment_orchestration_async_model_v1.md](experiment_orchestration_async_model_v1.md)
- [experiment_orchestration_async_safety_v1.md](experiment_orchestration_async_safety_v1.md)
- [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md)
- [sa_i1_orchestration_operations_freeze_audit.md](sa_i1_orchestration_operations_freeze_audit.md)

*End of experiment orchestration async governance v1.*
