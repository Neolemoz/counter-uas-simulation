# SA F1c — Corpus Navigation & Reviewer Workflows (PLAT-SA-F1c)

**Status:** implementation wave (additive to PLAT-SA-F1a, PLAT-SA-F1b)

## Goal

Extend replay corpus operations into corpus-scale reviewer navigation, lineage-aware browsing, read-only provenance/drift surfacing, and cognition-friendly grouping—while preserving deterministic replay governance boundaries.

## Allowed

- Additive `navigation_*` fields on corpus index entries
- Viewer: `CorpusBrowserPanel`, lineage nav, provenance/drift panel, breadcrumbs, release browser
- URL deep links: `?corpus_entry=`, optional `?corpus_release=`
- Dual-write drift/release manifests to `platform/sa-r0-viewer/public/demo/`
- `test_replay_corpus_navigation.py`, vitest corpus navigation tests
- Integrity hook `corpus_viewer_audit_mirror`
- Schema doc [replay_corpus_navigation_v1.md](replay_corpus_navigation_v1.md), freeze audit

## Forbidden

- F1d long-horizon publication batch ops
- WebSocket / rosbridge / live ROS
- Operational readiness, HITL, tactical authority
- ML reasoning / recommendations
- Cloud/database infrastructure
- Parser/topic changes
- Auto-regen from viewer UI

## Deliverables

| ID | Deliverable |
|----|-------------|
| F1c.1 | Navigation schema doc + plan |
| F1c.2 | Index navigation metadata in `build_replay_corpus_index.py` |
| F1c.3 | Audit/release viewer mirrors + integrity |
| F1c.4 | `corpus/` viewer modules + URL resolver |
| F1c.5 | Corpus browser + release browser + App integration |
| F1c.6 | Lineage nav + provenance panel |
| F1c.7 | Cognition grouping (family/chronology/declutter) |
| F1c.8 | Tests, CI, workflow docs, freeze audit |

## Validation

```bash
python3 scripts/evaluation/build_replay_corpus_index.py --check
python3 scripts/evaluation/build_replay_corpus_drift_report.py --check
python3 -m pytest src/counter_uas/test/test_replay_corpus_navigation.py -q
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

## Related

- [sa_f1a_corpus_indexing_plan.md](sa_f1a_corpus_indexing_plan.md)
- [sa_f1b_corpus_audit_operations_plan.md](sa_f1b_corpus_audit_operations_plan.md)
- [replay_corpus_navigation_v1.md](replay_corpus_navigation_v1.md)

## F1d follow-up (out of scope)

Long-horizon synthesis/publication batch operations across releases.
