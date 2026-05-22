# Replay Corpus Provenance Rules (`replay_corpus_provenance_rules_v1`)

Governance rules for corpus indexing, lineage, and release reproducibility.

## Replay-derived-only guarantee

All corpus index entries must trace to:

- Committed replay logs / meta sidecars, or
- Derived static artifacts built from those logs (bundles, sweeps, synthesis, exports)

**Never** treat corpus metadata as parser authority, tactical state, or deployment configuration.

## Deterministic corpus expectations

| Expectation | Rule |
|-------------|------|
| Discovery order | Sorted repo-relative paths |
| JSON serialization | `indent=2`, `sort_keys=True` |
| Revisions | `content_revision` and `sha256` from file bytes, not timestamps |
| Builder revision | `generation_revision` string bumped only when index logic changes |
| Staleness | `--check` compares committed JSON to rebuild |

## Offline reproducibility

1. Run `build_replay_corpus_index.py` to regenerate index.
2. Run `validate_replay_corpus.py` to verify schema and paths.
3. Run `audit_replay_lineage.py` for DAG/orphan/stale checks.
4. Optional: `build_replay_corpus_release.py` for frozen snapshot archives.

Reproducibility means **identical bytes given identical inputs**, not live runtime replay.

## Provenance caveats (required copy)

- Corpus lineage is **structural**, not semantic (use linkage index for sweep similarity).
- SHA256 manifests detect fixture drift; they do not validate tactical effectiveness.
- Research bundles and release snapshots are mentor/review artifacts only.
- `corpus_ref` on downstream artifacts is optional until regen; absence is not an error.

## Forbidden semantics

- Operational readiness or deployment certification
- HITL/operator workflow state
- ML recommendations or autonomous planning
- Realtime sync or cloud authority
- Parser/topic contract changes

## Maintainer discipline

After E2 fixture regen, run F1 index build before `export_research_bundle.py` so bundle `included_files` includes the corpus index.

See [sa_platform_maintainer_checklist.md](sa_platform_maintainer_checklist.md).
