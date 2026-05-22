# SA D2 — Monte Carlo Spatial Analytics (PLAT-SA-D2)

**Status:** implementation wave (additive to PLAT-SA-D1)

## Goal

Extend the replay experimentation platform into a deterministic spatial analytics and Monte Carlo replay-analysis environment while preserving replay-first, explanatory-only governance.

## Allowed

- `replay_mc_sweep_v1` manifests and sweeps catalog index
- Precomputed spatial analytics grids and Cesium overlays
- Sweep picker, metadata panel, member carousel (`?sweep=`, `?member=`)
- Lightweight analytics panels (CSS distributions, no chart libs)
- Replay variability cognition (template summaries)
- Four fixture sweep families + static markdown/JSON export
- Optional static `matched_seed_comparison_report` fixture panel
- Extend `comparison_hints` (`sweep_id`, `member_index`)
- Tests, `tier0-sa-r0`, freeze audit

## Forbidden

- WebSocket/rosbridge/live ROS integration
- Live MC orchestration UI or realtime dashboards
- HITL, engage, readiness, tactical authority UX
- Operational effectiveness or deployment readiness framing
- ML prediction / clustering
- Parser/topic contract changes
- GovernanceChrome structural redesign
- N-slot compare refactor (member carousel only)

## Deliverables

| ID | Deliverable |
|----|-------------|
| D2.1 | `replay_mc_sweep_v1` schema + `replay_mc_sweep.py` |
| D2.2 | Spatial analytics overlays (`spatialAnalytics.ts`, `spatialGridLayer.ts`) |
| D2.3 | Sweep catalog (`sweeps_index_v1.json`, `SweepCatalogPicker`) |
| D2.4 | Comparative analytics panels |
| D2.5 | Replay variability cognition |
| D2.6 | Four sweep fixture families |
| D2.7 | `export_replay_analytics_report.py` + committed reports |
| D2.8 | CI, freeze audit, registry |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py \
  src/counter_uas/test/test_replay_mc_sweep.py -q
python3 scripts/evaluation/sync_sa_catalog.py
python3 scripts/evaluation/gen_d2_sweep_fixtures.py
scripts/ci_eval.sh tier0-sa-r0
```
