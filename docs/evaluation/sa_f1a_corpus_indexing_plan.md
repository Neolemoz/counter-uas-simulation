# SA F1a — Corpus Indexing & Lineage Foundations (PLAT-SA-F1a)

**Status:** implementation wave (additive to PLAT-SA-E2, PLAT-SA-STAB)

## Goal

Create deterministic replay corpus indexing and structural lineage foundations for organizing large replay ecosystems while preserving provenance integrity and offline reproducibility.

## Allowed

- Schema docs: corpus index, manifest, lineage, provenance rules, release
- `replay_corpus_lineage.py`, `build_replay_corpus_index.py`, `validate_replay_corpus.py`, `audit_replay_lineage.py`, `build_replay_corpus_release.py`
- Optional `corpus_ref` on bundles, sweeps, synthesis, publication, research bundle
- Read-only `CorpusLineagePanel` in sa-r0-viewer
- `gen_f1_corpus_fixtures.py` orchestrator
- Integrity checks in `audit_sa_platform_integrity.py`, `tier0-sa-r0`
- Tests and freeze audit

## Forbidden

- WebSocket / rosbridge / live ROS
- Operational readiness, HITL, tactical authority
- ML reasoning / recommendations
- Cloud/collaborative infrastructure
- Parser/topic changes
- F1b/F1c/F1d scope (audit ops, navigation workflows, long-horizon publication ops)

## Deliverables

| ID | Deliverable |
|----|-------------|
| F1a.1 | Schema docs (index, manifest, lineage, provenance, release) |
| F1a.2 | `replay_corpus_lineage.py` normalization + DAG validation |
| F1a.3 | `build_replay_corpus_index.py` + committed fixture |
| F1a.4 | `validate_replay_corpus.py`, `audit_replay_lineage.py` |
| F1a.5 | Optional `corpus_ref` integration in producers |
| F1a.6 | Release snapshot example + `build_replay_corpus_release.py` |
| F1a.7 | Viewer corpus lineage panel |
| F1a.8 | Governance updates, CI, freeze audit |

## Validation

```bash
python3 scripts/evaluation/build_replay_corpus_index.py --check
python3 scripts/evaluation/validate_replay_corpus.py
python3 -m pytest src/counter_uas/test/test_replay_corpus_index.py -q
scripts/ci_eval.sh tier0-sa-r0
```

## Related docs

- [replay_corpus_index_v1.md](replay_corpus_index_v1.md)
- [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md)
- [replay_corpus_release_v1.md](replay_corpus_release_v1.md)
- [sa_e2_replay_knowledge_synthesis_plan.md](sa_e2_replay_knowledge_synthesis_plan.md)

## F1b follow-up (out of scope)

Corpus audit operations: drift reports, provenance diffs, batch regen orchestration, cross-release lineage diff.
