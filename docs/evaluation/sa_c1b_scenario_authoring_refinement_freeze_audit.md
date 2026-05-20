# PLAT-SA-C1b Freeze Audit — Scenario Authoring Refinement

**Wave:** PLAT-SA-C1b  
**Verdict:** frozen  
**Plan:** [sa_c1b_scenario_authoring_refinement_plan.md](sa_c1b_scenario_authoring_refinement_plan.md)

## Scope delivered

| Area | Status |
|------|--------|
| C1b metadata on 8 scenario packs | Done |
| Extended `lint_scenario_pack` validation | Done |
| Bundle propagation + `comparison_hints` | Done |
| Catalog enrichment + `public/demo/catalog.json` sync | Done |
| Scenario catalog picker + tag filters | Done |
| Provenance panel | Done |
| Replay cognition (annotations, timeline, LOS scope, legend) | Done |
| `comparison_foundations.md` | Done |

## Validation results

```text
pytest test_replay_sa_scenario + test_replay_sa_bundle + test_replay_sa_geometry: 17 passed
npm test (sa-r0-viewer): 14 passed
npm run build: OK
scripts/ci_eval.sh tier0-sa-r0: OK
```

All eight scenario packs validate. Catalog fixtures and public copy are in sync.

## UX observations

| Scenario | Observation |
|----------|-------------|
| `saturation_ingress` | Grouped ambiguity annotations and primary-threat LOS default reduce clutter; prioritization annotations rank 1 remain visible |
| `multi_ridge` | Chained ridge annotations group under shared gap event; LOS scope defaults to primary threat |
| `urban_masking` | Overlay legend helps distinguish eight degraded_visibility blocks; rank-3 clutter annotation collapses when list is long |
| Catalog picker | Category grouping and duration hints make the library feel like an experimentation catalog vs isolated URL aliases |
| Provenance panel | Lineage + fictional disclaimer visible without implying deployment planning |

## Remaining limitations

- No side-by-side compare mode (D1)
- Catalog is not editable in-viewer; pack edits require repo changes + `sync_sa_catalog.py`
- `replay_duration_class` uses fixed line-count thresholds; not tied to wall-clock playback
- `gen_b2_scenarios.py` pack writers still emit pre-C1b metadata if run alone — hand-maintained metadata authoritative until writers updated
- Monte Carlo topology sweep UI not implemented

## Recommended D1 follow-up

- Side-by-side topology/replay compare viewer using `comparison_hints`
- Sensor placement diff overlays
- Strict pack-time overlay vs log span lint (strict mode)
- Per-threat mock pane routing for saturation packs
- Monte Carlo catalog sweep runner

## Boundary check

- No WebSocket/rosbridge/live feeds added
- GovernanceChrome structure preserved (subtitle badge only)
- No operational severity or HITL semantics
- `narrative_rank` documented and labeled as replay emphasis only
