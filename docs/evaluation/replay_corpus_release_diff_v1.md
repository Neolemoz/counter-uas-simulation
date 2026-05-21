# Replay Corpus Release Diff (`replay_corpus_release_diff_v1`)

Deterministic diff between two corpus index snapshots (e.g. frozen release vs canonical index).

See also: [replay_corpus_release_v1.md](replay_corpus_release_v1.md), [replay_corpus_index_v1.md](replay_corpus_index_v1.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/corpus_audits/replay_corpus_release_diff_v1.json` | Default diff: release r1 vs canonical |

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_corpus_release_diff_v1` |
| `schema_version` | yes | `replay_corpus_release_diff_v1` |
| `corpus_id` | yes | Parent corpus |
| `baseline_id` | yes | e.g. `sa_r0_corpus_r1_r1` |
| `target_id` | yes | e.g. `canonical_index` |
| `index_revision_delta` | yes | `{ baseline, target }` |
| `entries_added` | yes | Entry IDs in target only |
| `entries_removed` | yes | Entry IDs in baseline only |
| `entries_changed` | yes | Per-entry field deltas |
| `lineage_edges_added` | yes | Edge IDs in target only |
| `lineage_edges_removed` | yes | Edge IDs in baseline only |
| `release_behind_canonical` | yes | True when canonical differs from frozen release |
| `governance` | yes | Notice + anti-claims |

## Default comparison

- **Baseline:** `fixtures/sa_r0/corpus_releases/sa_r0_corpus_r1_r1/replay_corpus_index_v1.json`
- **Target:** `fixtures/sa_r0/synthesis/replay_corpus_index_v1.json`

Non-empty diff after regen is expected until `build_replay_corpus_release.py` refreshes the snapshot.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `diff_replay_corpus_releases.py` |
| Consumer | Maintainer review, `verify_replay_corpus_reproducibility.py` |
