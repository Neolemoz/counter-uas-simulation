# SA F1b — Corpus Audit & Provenance Operations Freeze Audit (PLAT-SA-F1b)

Wave plan: [sa_f1b_corpus_audit_operations_plan.md](sa_f1b_corpus_audit_operations_plan.md)

Schema docs: [replay_corpus_drift_report_v1.md](replay_corpus_drift_report_v1.md), [replay_corpus_release_diff_v1.md](replay_corpus_release_diff_v1.md), [replay_corpus_regen_workflow_v1.md](replay_corpus_regen_workflow_v1.md), [replay_corpus_reproducibility_v1.md](replay_corpus_reproducibility_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| F1b schema docs | Done |
| `collect_drift_findings`, `diff_corpus_indexes` in lineage lib | Done |
| `build_replay_corpus_drift_report.py` + fixtures | Done |
| `diff_replay_corpus_releases.py` + fixture | Done |
| `audit_replay_corpus_provenance.py` | Done |
| `run_replay_corpus_regen.py`, `verify_replay_corpus_reproducibility.py` | Done |
| `CorpusLineagePanel` provenance refinements | Done |
| Integrity + `tier0-sa-r0` + tests | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / readiness semantics | Pass |
| Drift/diff explanatory only | Pass |
| No parser/topic changes | Pass |
| Drift report maintainer-only (not viewer route) | Pass |
| F1c navigation / F1d publication ops not started | Pass |

## Validation

```bash
python3 scripts/evaluation/verify_replay_corpus_reproducibility.py
python3 -m pytest src/counter_uas/test/test_replay_corpus_index.py \
  src/counter_uas/test/test_replay_corpus_audit.py -q
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
scripts/ci_eval.sh tier0-sa-r0
```

## UX observations

- Corpus lineage panel shows truncated `index_revision` and stale `corpus_ref` badge when artifact predates index regen.
- Entry kind and `primary_artifact_path` help reviewers connect sweep context to index inventory without navigation graph (F1c).

## Limitations

- Single release snapshot (`sa_r0_corpus_r1_r1`); multi-release history deferred.
- Drift report not served in viewer (`corpus_audits/` only).
- Unindexed-file and `corpus_ref_missing` findings are informational until full fixture regen.
- Release diff `--check` verifies committed diff bytes, not empty diff (parity with canonical when release refreshed).

## Recommended F1c scope

- Corpus browser / entry picker in sa-r0-viewer
- Lineage parent/child navigation links
- Read-only drift findings in reviewer UI
- Cross-entry jump from `corpus_ref` in workstation/compare mode

Do not start F1d until F1c is scoped.

## Verdict

**Verdict: frozen** for PLAT-SA-F1b.
