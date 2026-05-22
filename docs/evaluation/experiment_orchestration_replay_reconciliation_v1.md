# Experiment Orchestration Replay Reconciliation (`experiment_orchestration_replay_reconciliation_v1`)

**Phase:** PLAT-SA-I3 — replay replacement and fingerprint reconciliation  
**Authority:** [AGENTS.md](../../AGENTS.md)

Replay reconciliation is **explanatory**. Bundle `index.json` hashes are compared for reviewer cognition; they are not parser contracts.

---

## Replay reconciliation flow

```
frozen manifest + queue snapshot
  → worker execution record(s)
  → execution_fingerprint (canonical over manifest + queue + report steps)
  → replay bundle index.json
  → replay_fingerprint (stored on async sidecar)
```

When a replay is regenerated after failure:

1. Store new `replay_fingerprint` on the async sidecar (additive update to sidecar fields only).
2. Append `async_lineage` event documenting replacement.
3. Prior bundle remains addressable via worker `retry_lineage` / refs — not deleted.

---

## Superseded replay handling

When a queue snapshot is superseded:

- Prior async sidecar may be marked `superseded` (read-only).
- Open claims on superseded snapshots are audit errors (`queue_reconciliation`).
- Replay fingerprints on superseded chains are historical evidence only.

---

## Fingerprint reconciliation

| Fingerprint | Scope | Reconciliation check |
|-------------|-------|----------------------|
| `execution_fingerprint` | Manifest + queue + report step hashes | Re-hash frozen inputs; must match stored or document terminal reason |
| `replay_fingerprint` | Bundle `index.json` SHA-256 | `verify_replay_reproducibility` vs stored async field |

Per worker attempt: execution fingerprint on the record must align with async sidecar when that attempt is terminal.

---

## Replay replacement continuity

When status transitions `failed`/`retrying` → success path:

- Recovered replay equivalence: bundle hash must match newly stored `replay_fingerprint`, or async must remain `quarantined`.
- Replacement continuity: successor `replay_fingerprint` must not silently overwrite without lineage event.

---

## Guarantees and non-claims

**Guarantees (offline audit):**

- Deterministic re-hash of frozen inputs yields stable `execution_fingerprint` or explicit audit failure.
- Stale `replay_fingerprint` vs current bundle is always flagged.

**Non-claims:**

- Does not prove runtime correctness or tactical effectiveness.
- Does not authorize automatic regen or promote.

*End of experiment orchestration replay reconciliation v1.*
