# Replay Federation Lineage (`replay_federation_lineage_v1`)

Cross-corpus structural derivation edges within a federation. Distinct from corpus-internal lineage ([replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md)) and semantic linkage ([replay_linkage_v1.md](replay_linkage_v1.md)).

## Federation edge object

| Field | Required | Notes |
|-------|----------|-------|
| `edge_id` | yes | `{from_group}__{to_group}__{ref_kind}` |
| `from_corpus_group_id` | yes | Source partition |
| `to_corpus_group_id` | yes | Target partition |
| `ref_kind` | yes | See below |
| `evidence` | yes | `{ source_paths[], note? }` |

### `ref_kind` values

| Kind | Meaning |
|------|---------|
| `federation_release_derived` | Release snapshot derived from canonical index |
| `federation_publication_chain` | Publication packet continuity across groups |
| `federation_recovery_continuity` | Async recovery/reconciliation propagation |
| `federation_study_lineage` | Cross-study structural parent |

## `federation_lineage_refs` (manifest)

Stable string refs matching `edge_id` values in `replay_federation_lineage_graph_v1`.

## Normalization rules

1. No cycles in federation DAG across `corpus_group_id` values.
2. Every edge endpoint must exist in manifest `corpus_groups`.
3. Release-derived edges require matching `release_id` on target group.

## Anti-claims

- Federation lineage documents **artifact partitioning**, not causal inference.
- Cross-group edges do not certify replay equivalence (use orchestration replay reconciliation for that).

## Implementation

`replay_federation_lineage.py` — `validate_federation_lineage_dag()`, `build_federation_lineage_graph()`.
