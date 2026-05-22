# SA F1a — Corpus Indexing & Lineage Foundations Freeze Audit (PLAT-SA-F1a)

Wave plan: [sa_f1a_corpus_indexing_plan.md](sa_f1a_corpus_indexing_plan.md)

Schema docs: [replay_corpus_index_v1.md](replay_corpus_index_v1.md), [replay_corpus_manifest_v1.md](replay_corpus_manifest_v1.md), [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md), [replay_corpus_provenance_rules_v1.md](replay_corpus_provenance_rules_v1.md), [replay_corpus_release_v1.md](replay_corpus_release_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| F1a schema docs (index, manifest, lineage, provenance, release) | Done |
| `replay_corpus_lineage.py` normalization + DAG validation | Done |
| `build_replay_corpus_index.py` + committed `replay_corpus_index_v1.json` | Done |
| `validate_replay_corpus.py`, `audit_replay_lineage.py` | Done |
| Optional `corpus_ref` on synthesis, publication, research bundle producers | Done |
| `build_replay_corpus_release.py` + `sa_r0_corpus_r1_r1` snapshot | Done |
| `CorpusLineagePanel` + Zod schemas | Done |
| `gen_f1_corpus_fixtures.py`, integrity checks, `tier0-sa-r0` | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / readiness / operational semantics | Pass |
| Structural lineage only (distinct from linkage similarity) | Pass |
| No parser/topic changes | Pass |
| Additive `corpus_ref` optional on artifacts | Pass |
| No ML / cloud / collaborative infrastructure | Pass |
| GovernanceChrome not structurally redesigned | Pass |

## Validation

```bash
python3 scripts/evaluation/build_replay_corpus_index.py --check
python3 scripts/evaluation/validate_replay_corpus.py
python3 scripts/evaluation/audit_replay_lineage.py
python3 scripts/evaluation/build_replay_corpus_release.py --check
python3 -m pytest src/counter_uas/test/test_replay_corpus_index.py -q
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
scripts/ci_eval.sh tier0-sa-r0
```

## UX observations

- Corpus lineage panel collapses by default in sweep workstation; complements linkage panel without duplicating semantic edges.
- Lineage parents list uses stable `entry_id` slugs — reviewers can trace derivation without implying tactical validity.
- Synthesis artifact now carries optional `corpus_ref` for cross-artifact consistency when fixtures are regenerated.

## Limitations

- Single corpus (`sa_r0_corpus_r1`) indexed; multi-corpus registry deferred.
- Sweep manifests and demo bundles retain optional `corpus_ref` until full fixture regen.
- Release snapshots are full static copies, not incremental sync.
- Lineage is artifact derivation, not replay similarity (use `replay_linkage_index_v1` for that).

## Recommended F1b follow-up

- Corpus drift reports across large fixture trees
- Provenance diff between index revisions
- Batch regen orchestration with maintainer reports
- Stricter orphan quarantine policies
- Cross-release lineage diff (release A → release B)

Do not start F1c (navigation/reviewer workflows) or F1d (long-horizon publication ops) until F1b is scoped.

## Verdict

**Verdict: frozen** for PLAT-SA-F1a.
