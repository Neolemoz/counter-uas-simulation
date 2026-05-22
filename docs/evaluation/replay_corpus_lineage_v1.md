# Replay Corpus Lineage (`replay_corpus_lineage_v1`)

Structural derivation semantics for corpus index entries. Distinct from [replay_linkage_v1.md](replay_linkage_v1.md) (semantic sweep similarity).

## `derived_from` ref object

| Field | Required | Notes |
|-------|----------|-------|
| `ref_kind` | yes | Derivation kind (see below) |
| `ref_id` | yes | Stable id: `entry_id`, `pack_id`, `sweep_id`, or `storyboard_id` |
| `artifact_path` | no | Repo-relative path when helpful |

### `ref_kind` values

| Kind | Meaning |
|------|---------|
| `bundle_packed_from_log` | Demo bundle packed from evaluation log |
| `sweep_derived` | Sweep family derived from member bundles / topology packs |
| `synthesis_derived` | Synthesis rollup derived from sweep families |
| `presentation_derived` | Storyboard derived from sweep/bundle refs |
| `export_derived` | Static export derived from sweep manifest |
| `regenerated_from` | Rebuilt from prior artifact revision |
| `research_bundle_aggregated` | Research bundle aggregates synthesis + sweeps |

## `lineage_parent_ids`

Sorted, deduplicated list of parent `entry_id` values. Empty for root topology experiments and standalone demos without catalog parent.

## Normalization rules

1. **`entry_id`** — `{entry_kind}__{slug}` where slug uses `[a-z0-9_]` only.
2. **`lineage_parent_ids`** — sorted lexicographically, no duplicates, must not include self.
3. **DAG** — no cycles; every parent must exist in `entries`.
4. **`content_revision`** — SHA256 of canonical JSON (`sort_keys=True`) of entry minus volatile fields, or primary artifact SHA256.

## Lineage edges (optional `lineage_edges[]`)

| Field | Required | Notes |
|-------|----------|-------|
| `edge_id` | yes | `{parent}__{child}__{ref_kind}` |
| `parent_entry_id` | yes | Parent entry |
| `child_entry_id` | yes | Child entry |
| `ref_kind` | yes | Derivation kind |
| `evidence` | yes | `{ source_paths[], note? }` |

## Reviewer navigation (F1c)

Lineage parent/child links support **read-only** corpus browser jumps via `corpus_entry` URL param. Navigation does not alter derivation semantics or imply operational lifecycle management.

## Long-horizon evolution (F1d)

Cross-release diffs and chronology tiers in [`replay_corpus_evolution_v1.md`](replay_corpus_evolution_v1.md) document replay artifact evolution across release snapshots and generation waves. Evolution summaries are descriptive rollups only.

## Anti-claims

- Lineage documents **artifact derivation**, not causal inference or tactical doctrine.
- Parent/child links do not certify comparability or validity across experiments.
- Missing parents are integrity errors, not runtime faults.

## Implementation

| Role | Module |
|------|--------|
| Library | `replay_corpus_lineage.py` |
| Builder | `build_replay_corpus_index.py` |
| Auditor | `audit_replay_lineage.py` |
