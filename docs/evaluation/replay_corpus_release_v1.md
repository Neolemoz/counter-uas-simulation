# Replay Corpus Release (`replay_corpus_release_v1`)

Deterministic offline snapshot of a corpus index and selected artifacts for long-horizon reproducibility.

See also: [replay_corpus_index_v1.md](replay_corpus_index_v1.md), [replay_research_bundle_v1.md](replay_research_bundle_v1.md).

## Artifact placement

```
fixtures/sa_r0/corpus_releases/<release_id>/
  replay_corpus_release_manifest_v1.json
  replay_corpus_index_v1.json    # frozen copy at release time
```

## Release manifest fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_corpus_release_manifest_v1` |
| `schema_version` | yes | `replay_corpus_release_manifest_v1` |
| `release_id` | yes | e.g. `sa_r0_corpus_r1_r1` |
| `corpus_id` | yes | Parent corpus |
| `parent_release_ids` | yes | Prior releases (empty for first) |
| `snapshot_root` | yes | Relative path under `corpus_releases/` |
| `indexed_entry_ids` | yes | Entry IDs included in snapshot |
| `files` | yes | `{ path, sha256, size_bytes }[]` relative to snapshot root |
| `governance` | yes | Notice + anti-claims |
| `provenance` | yes | `{ generators, source_root }` |
| `evolution_chain` | no | Release id chain with index revisions (F1d) |
| `publication_revision` | no | Fingerprint of publication artifacts at snapshot time |

## Workflow (maintainer)

1. `python3 scripts/evaluation/build_replay_corpus_index.py`
2. `python3 scripts/evaluation/validate_replay_corpus.py`
3. `python3 scripts/evaluation/build_replay_corpus_release.py`
4. `python3 scripts/evaluation/build_replay_corpus_release.py --check` (CI)

## Conventions

- **Not** deployment packaging — offline research archive only.
- Release copies index JSON verbatim at build time.
- Optional aggregate `MANIFEST.sha256` lists top-level manifest hash.
- Bump `release_id` suffix (`_r1`, `_r2`) for new snapshots; link via `parent_release_ids`.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_release.py` |
| Validator | `validate_replay_corpus.py`, integrity audit `corpus_release_stale` |
