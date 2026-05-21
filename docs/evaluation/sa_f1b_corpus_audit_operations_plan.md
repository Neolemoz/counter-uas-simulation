# SA F1b — Corpus Audit & Provenance Operations (PLAT-SA-F1b)

**Status:** implementation wave (additive to PLAT-SA-F1a)

## Goal

Deterministic corpus drift reporting, cross-release diffs, batch regen orchestration, provenance integrity audits, reproducibility verification, and lightweight viewer provenance refinements.

## Allowed

- `replay_corpus_drift_report_v1`, `replay_corpus_release_diff_v1` schemas
- `build_replay_corpus_drift_report.py`, `diff_replay_corpus_releases.py`
- `audit_replay_corpus_provenance.py`, `run_replay_corpus_regen.py`, `verify_replay_corpus_reproducibility.py`
- Shared drift/diff helpers in `replay_corpus_lineage.py`
- CorpusLineagePanel refinements (index revision, corpus_ref mismatch badge)
- `fixtures/sa_r0/corpus_audits/` committed fixtures
- Integrity + `tier0-sa-r0` extensions

## Forbidden

- F1c corpus navigation / entry picker / lineage graph UI
- F1d long-horizon publication batch ops
- WebSocket, ROS, HITL, readiness, ML
- Parser/topic changes

## Deliverables

| ID | Deliverable |
|----|-------------|
| F1b.1 | Schema docs (drift, diff, regen, reproducibility) |
| F1b.2 | `collect_drift_findings`, `diff_corpus_indexes` in lineage lib |
| F1b.3 | Drift report builder + fixtures |
| F1b.4 | Release diff tool + fixture |
| F1b.5 | Provenance audit + integrity hooks |
| F1b.6 | Regen orchestrator + reproducibility verifier |
| F1b.7 | Viewer provenance refinements |
| F1b.8 | Governance, CI, freeze audit |

## Validation

```bash
python3 scripts/evaluation/verify_replay_corpus_reproducibility.py
python3 -m pytest src/counter_uas/test/test_replay_corpus_audit.py -q
scripts/ci_eval.sh tier0-sa-r0
```

## Related

- [sa_f1a_corpus_indexing_plan.md](sa_f1a_corpus_indexing_plan.md)
- [replay_corpus_drift_report_v1.md](replay_corpus_drift_report_v1.md)
