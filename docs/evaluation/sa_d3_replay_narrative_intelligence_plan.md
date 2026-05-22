# SA D3 — Replay Narrative Intelligence & Experiment Review Workstation (PLAT-SA-D3)

**Status:** implementation wave (additive to PLAT-SA-D2)

## Goal

Evolve the replay experimentation platform from deterministic spatial analytics into a governance-safe **experiment review workstation**: replay narrative intelligence, cohort grouping, N-slot sweep filmstrip review, replay-pattern taxonomy, and enriched static review exports — without parser/runtime changes or operational semantics.

## Allowed

- `replay_narrative_summary` and `replay_cohorts` on `replay_mc_sweep_v1` manifests
- Per-member `replay_pattern_tags` / `replay_pattern_summary` (rule-based, pack-time)
- `classify_replay_pattern.py`, `replay_narrative_intelligence.py`, `gen_d3_sweep_enrichment.py`
- Sweep workstation UX (cohort nav, pattern cards, anomaly highlights)
- N-slot sweep filmstrip (2–4 members, sweep-scoped; not general compare refactor)
- Extended `export_replay_analytics_report.py` review artifacts
- Spatial overlay declutter + cohort filtering in viewer
- Pack-time overlay `active_t_range` ⊆ log span lint (warnings)
- Tests, `tier0-sa-r0`, freeze audit

## Forbidden

- WebSocket / rosbridge / live ROS integration
- ML prediction or dynamic clustering
- Tactical recommendations, deployment readiness, operational guidance framing
- HITL, engage, readiness, tactical authority UX
- Parser/topic contract changes
- GovernanceChrome structural redesign
- Refactoring A/B compare into global N-way compare

## Deliverables

| ID | Deliverable |
|----|-------------|
| D3.1 | `replay_narrative_intelligence_v1` schema + `replay_narrative_intelligence.py` |
| D3.2 | `replay_pattern_taxonomy_v1` + `classify_replay_pattern.py` |
| D3.3 | Deterministic `replay_cohorts` builder |
| D3.4 | Sweep workstation UX (cohort nav, pattern cards, anomalies) |
| D3.5 | N-slot cohort filmstrip (`?filmstrip=`, `?cohort=`) |
| D3.6 | Review exports (narrative, cluster, divergence, hotspot, `replay_review_report_v1`) |
| D3.7 | Spatial declutter + sweep cognition refinements |
| D3.8 | `gen_d3_sweep_enrichment.py` + fixture refresh |
| D3.9 | CI, freeze audit, registry |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py \
  src/counter_uas/test/test_replay_mc_sweep.py \
  src/counter_uas/test/test_replay_narrative_intelligence.py \
  src/counter_uas/test/test_replay_pattern_taxonomy.py -q
python3 scripts/evaluation/gen_d3_sweep_enrichment.py
python3 scripts/evaluation/sync_sa_catalog.py
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
```

## Related docs

- [replay_narrative_intelligence_v1.md](replay_narrative_intelligence_v1.md)
- [replay_pattern_taxonomy_v1.md](replay_pattern_taxonomy_v1.md)
- [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md)
- [sa_d2_monte_carlo_spatial_analytics_plan.md](sa_d2_monte_carlo_spatial_analytics_plan.md)
