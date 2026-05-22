# Replay Corpus Regen Workflow (`replay_corpus_regen_workflow_v1`)

Ordered offline regeneration for corpus index, audits, release snapshot, and research bundle.

See also: [sa_platform_maintainer_checklist.md](sa_platform_maintainer_checklist.md), [replay_corpus_reproducibility_v1.md](replay_corpus_reproducibility_v1.md).

## Recommended command

```bash
python3 scripts/evaluation/run_replay_corpus_regen.py --dry-run   # preview
python3 scripts/evaluation/run_replay_corpus_regen.py             # execute
```

## Default step order

| Step | Script | Purpose |
|------|--------|---------|
| 1 | `gen_f1_corpus_fixtures.py` | Index, validate, research bundle, release |
| 2 | `build_replay_corpus_drift_report.py` | Drift inventory |
| 3 | `diff_replay_corpus_releases.py` | Release vs canonical diff |
| 4 | `build_replay_corpus_release.py` | Refresh frozen snapshot (if index changed) |

Optional upstream (when sweep/synthesis changed):

- `sync_sa_catalog.py`
- `gen_d3_sweep_enrichment.py`
- `gen_e1_presentation_fixtures.py`
- `gen_e2_research_fixtures.py`

Use `run_replay_corpus_regen.py --with-e2` to include E2 steps before F1.

## Outputs

- `fixtures/sa_r0/corpus_audits/regen_run_report_v1.json` — step status report

## Governance

Regen restores deterministic bytes; it does not certify operational effectiveness.
