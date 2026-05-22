# Replay Corpus Reproducibility (`replay_corpus_reproducibility_v1`)

End-to-end offline verification that corpus fixtures match deterministic rebuilds.

## Single gate command

```bash
python3 scripts/evaluation/verify_replay_corpus_reproducibility.py
```

## Checks performed

1. `build_replay_corpus_index.py --check`
2. `validate_replay_corpus.py`
3. `build_replay_corpus_drift_report.py --check`
4. `export_research_bundle.py --check`
5. `build_replay_corpus_release.py --check`
6. `diff_replay_corpus_releases.py` (writes/updates diff artifact)
7. `audit_replay_corpus_provenance.py`

## Full platform gate

```bash
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

## Expectations

- Index and drift report bytes stable given unchanged inputs
- Research bundle SHA256 manifest matches on-disk files
- Release snapshot may lag canonical index until maintainer runs release rebuild (diff documents gap)

## Governance

Reproducibility means identical artifact bytes, not live runtime replay equivalence.
