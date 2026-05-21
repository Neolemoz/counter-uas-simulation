# SA F1d — Long-Horizon Synthesis & Publication Operations (PLAT-SA-F1d)

**Status:** implementation wave (additive to PLAT-SA-F1c)

## Goal

Extend the replay corpus ecosystem with cross-release/chronology evolution synthesis, deterministic publication/archive tooling, evolution metadata, and lightweight reviewer chronology panels.

## Allowed

- `replay_corpus_evolution_manifest_v1`, `replay_corpus_evolution_summary_v1`, `replay_corpus_publication_packet_v1` schemas
- `build_replay_corpus_evolution.py`, `build_replay_corpus_publication.py`, `export_replay_corpus_release.py`, `verify_replay_corpus_release.py`
- Evolution helpers in `replay_corpus_lineage.py`
- Release `--parent-release` chain support
- Viewer: `CorpusEvolutionPanel`, evolution badges, multi-release browser
- `gen_f1d_publication_fixtures.py`, integrity hooks, tests, freeze audit

## Forbidden

- Runtime/simulation/UI architecture expansion
- WebSocket, ROS, HITL, readiness, ML
- Parser/topic changes
- Auto-publish to external systems from viewer

## Deliverables

| ID | Deliverable |
|----|-------------|
| F1d.1 | Evolution + publication schema docs |
| F1d.2 | Evolution metadata on index + release manifest |
| F1d.3 | `build_replay_corpus_evolution.py` + fixtures |
| F1d.4 | Publication/export/verify CLIs |
| F1d.5 | Viewer evolution panel + badges |
| F1d.6 | Tests, CI, integrity, freeze audit |

## Validation

```bash
python3 scripts/evaluation/build_replay_corpus_evolution.py --check
python3 scripts/evaluation/build_replay_corpus_publication.py --check
python3 scripts/evaluation/verify_replay_corpus_release.py
python3 -m pytest src/counter_uas/test/test_replay_corpus_evolution.py -q
scripts/ci_eval.sh tier0-sa-r0
```

## Related

- [sa_f1c_corpus_navigation_plan.md](sa_f1c_corpus_navigation_plan.md)
- [replay_corpus_evolution_v1.md](replay_corpus_evolution_v1.md)
- [replay_corpus_publication_v1.md](replay_corpus_publication_v1.md)
