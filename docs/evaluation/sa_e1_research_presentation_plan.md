# SA E1 — Research Presentation & Reviewer Experience Platform (PLAT-SA-E1)

**Status:** implementation wave (additive to PLAT-SA-D3)

## Goal

Transform the replay-review workstation into a governance-safe **presentation and reviewer cognition environment** for mentor demos, replay debriefs, and topology research walkthroughs — using deterministic, offline artifacts only.

## Allowed

- `presentation` on bundles, `presentation_walkthrough` on sweeps, `replay_storyboard_v1` decks
- `build_replay_presentation.py`, `build_replay_storytelling.py`, `export_presentation_pack.py`, `gen_e1_presentation_fixtures.py`
- Presentation mode UI in `platform/sa-r0-viewer/` (dedicated layout, chapter nav, storytelling panel)
- Presentation cognition indicators (non-scoring)
- Curated storyboard fixtures under `fixtures/sa_r0/presentations/`
- Tests, `tier0-sa-r0`, freeze audit

## Forbidden

- WebSocket / rosbridge / live ROS integration
- Tactical recommendations, deployment readiness, operational battle-map styling
- HITL, engage, readiness, tactical authority UX
- Parser/topic contract changes
- GovernanceChrome structural redesign
- ML narration, reviewer scoring, personalization

## Deliverables

| ID | Deliverable |
|----|-------------|
| E1.1 | `replay_presentation_v1.md` + `replay_storyboard_v1.md` |
| E1.2 | `build_replay_presentation.py` + bundle/sweep enrichment |
| E1.3 | Presentation mode UI |
| E1.4 | Guided walkthrough playback |
| E1.5 | Cognition UX (`importance_weights`, declutter, focus transitions) |
| E1.6 | Replay storytelling layer |
| E1.7 | Export workflows + static HTML packet |
| E1.8 | Curated presentation fixtures |
| E1.9 | Presentation cognition analytics |
| E1.10 | CI, freeze audit, registry |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py \
  src/counter_uas/test/test_replay_mc_sweep.py \
  src/counter_uas/test/test_replay_narrative_intelligence.py \
  src/counter_uas/test/test_replay_presentation.py \
  src/counter_uas/test/test_replay_storytelling.py -q
python3 scripts/evaluation/gen_e1_presentation_fixtures.py
python3 scripts/evaluation/sync_sa_catalog.py
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
```

## Related docs

- [replay_presentation_v1.md](replay_presentation_v1.md)
- [replay_storyboard_v1.md](replay_storyboard_v1.md)
- [sa_d3_replay_narrative_intelligence_plan.md](sa_d3_replay_narrative_intelligence_plan.md)
