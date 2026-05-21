# SA F1d — Long-Horizon Synthesis & Publication Operations Freeze Audit (PLAT-SA-F1d)

Wave plan: [sa_f1d_long_horizon_publication_plan.md](sa_f1d_long_horizon_publication_plan.md)

Schemas: [replay_corpus_evolution_v1.md](replay_corpus_evolution_v1.md), [replay_corpus_publication_v1.md](replay_corpus_publication_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| F1d plan + evolution/publication schema docs | Done |
| Evolution metadata on corpus index + release manifest extensions | Done |
| `build_replay_corpus_evolution.py` + committed manifest/summary | Done |
| `build_replay_corpus_publication.py`, `export_replay_corpus_release.py`, `verify_replay_corpus_release.py` | Done |
| `gen_f1d_publication_fixtures.py` orchestrator | Done |
| `CorpusEvolutionPanel`, badges, multi-release browser, chronology filters | Done |
| pytest + vitest + `tier0-sa-r0` F1d gate | Done |
| Integrity checks: evolution/publication stale + release export | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / readiness semantics | Pass |
| Evolution rollups descriptive only | Pass |
| No parser/topic changes | Pass |
| No tracker/fusion/runtime redesign | Pass |
| Viewer does not execute regen or external publish | Pass |
| Publication archive offline fixture only | Pass |

## Validation

```bash
python3 scripts/evaluation/build_replay_corpus_evolution.py --check
python3 scripts/evaluation/build_replay_corpus_publication.py --check
python3 scripts/evaluation/verify_replay_corpus_release.py
python3 -m pytest src/counter_uas/test/test_replay_corpus_index.py \
  src/counter_uas/test/test_replay_corpus_audit.py \
  src/counter_uas/test/test_replay_corpus_navigation.py \
  src/counter_uas/test/test_replay_corpus_evolution.py -q
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

## UX observations

- **Corpus evolution** panel gives chronology timeline and family narratives without implying operational lifecycle management.
- Multi-release browser lists frozen snapshots with `parent_release_ids` chain.
- Evolution tag filters complement F1c navigation tags for long-horizon corpus review.
- Optional `?corpus_chronology=<tier_id>` highlights a chronology tier in the evolution panel.

## Limitations

- Second release snapshot (`r2`) optional until maintainer runs `build_replay_corpus_release.py --parent-release`.
- Evolution narratives are rule-based rollups, not causal inference.
- Publication archive is offline zip under `corpus_releases/<id>/archive/`, not CDN distribution.
- Research bundle remains export-via-CLI, not interactive archive browser.

## Verdict

**Verdict: frozen** for PLAT-SA-F1d.
