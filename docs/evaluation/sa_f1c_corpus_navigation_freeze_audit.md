# SA F1c — Corpus Navigation & Reviewer Workflows Freeze Audit (PLAT-SA-F1c)

Wave plan: [sa_f1c_corpus_navigation_plan.md](sa_f1c_corpus_navigation_plan.md)

Schema: [replay_corpus_navigation_v1.md](replay_corpus_navigation_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| F1c plan + navigation schema doc | Done |
| Navigation metadata on all corpus index entries | Done |
| Viewer mirrors for drift report + release manifest | Done |
| `CorpusBrowserPanel` + filters/grouping/declutter | Done |
| `CorpusLineageNavPanel` parent/child/corpus_ref jumps | Done |
| `CorpusProvenancePanel` drift surfacing | Done |
| `CorpusReleaseBrowser` + `?corpus_entry=` URL resolver | Done |
| `corpus_viewer_audit_mirror` integrity check | Done |
| pytest + vitest + `tier0-sa-r0` F1c gate | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / readiness semantics | Pass |
| Drift/provenance badges descriptive only | Pass |
| No parser/topic changes | Pass |
| F1d long-horizon publication ops not started | Pass |
| No auto-regen from viewer | Pass |

## Validation

```bash
python3 scripts/evaluation/build_replay_corpus_index.py --check
python3 scripts/evaluation/build_replay_corpus_drift_report.py --check
python3 -m pytest src/counter_uas/test/test_replay_corpus_index.py \
  src/counter_uas/test/test_replay_corpus_audit.py \
  src/counter_uas/test/test_replay_corpus_navigation.py -q
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

## UX observations

- Collapsible **Corpus inventory** reduces sidebar clutter; default hides `demo_bundle` and `replay_export` kinds.
- Family/chronology/kind grouping helps reviewers scan 57 entries without a database.
- Lineage parent/child buttons and `corpus_ref` jump shorten sweep ↔ synthesis navigation.
- Drift badges surface F1b findings in-workstation without implying operational failure.

## Limitations

- Single release snapshot (`sa_r0_corpus_r1_r1`); no multi-release timeline UI.
- Research bundle entry opens corpus info only (no full bundle runtime loader).
- Drift/regen remains maintainer-script driven; viewer does not execute regen.
- `corpus_ref_missing` on sweep manifests until optional backfill.
- Publication HTML not embedded — routing lands on sweep workstation.

## Recommended F1d scope

- Batch publication packet regeneration across releases
- Cross-release synthesis diff narratives in reviewer UI
- Long-horizon export manifest operations

Do not start F1d until F1c is frozen.

## Verdict

**Verdict: frozen** for PLAT-SA-F1c.
