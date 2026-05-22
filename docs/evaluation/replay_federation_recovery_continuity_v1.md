# Replay Federation Recovery Continuity (`replay_federation_recovery_continuity_v1`)

Propagates frozen I3 async recovery semantics into federation-scoped reviewer cognition. **Not** live recovery authority.

See also: [experiment_orchestration_async_recovery_v1.md](experiment_orchestration_async_recovery_v1.md), [orchestration_reconciliation_lineage_index_v1.md](orchestration_reconciliation_lineage_index_v1.md).

## Report artifact

`orchestration_federation_recovery_continuity_v1` at `fixtures/sa_r0/federation/audits/`.

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `orchestration_federation_recovery_continuity_v1` |
| `federation_id` | yes | Parent federation |
| `governance` | yes | Notice + anti_claims |
| `corpus_group_scopes` | yes | Per-group recovery rollup |
| `recovery_issues` | yes | Cross-plane findings |
| `continuity_ok` | yes | Aggregate bool |

## Optional reconciliation index extensions

`orchestration_reconciliation_lineage_index_v1` may add nullable `corpus_group_id` on:

- `retry_groups[]`
- `supersede_edges[]`
- `quarantine_holds[]`

Backward compatible: absent field defaults to primary canonical group.

## Recovery issue kinds (federation plane)

| Kind | Rule |
|------|------|
| `superseded_replay_federation_gap` | Superseded manifest replay not indexed in expected group |
| `quarantine_federation_hold` | Quarantined manifest in federation scope without hold marker |
| `recovery_lineage_federation_break` | Reconciliation edge references manifest outside registry |
| `reconciliation_continuity_propagation` | `recovery_continuity_ok` false while federation claims continuity |

## Invariants

- `quarantined` still blocks promote (I2 guard unchanged).
- Federation does not clear quarantine or supersede from browser.
- Historical queue/audit/recovery artifacts remain immutable.

## Implementation

`audit_federation_recovery_continuity.py` — composes I3 batch audit + federation manifest.
