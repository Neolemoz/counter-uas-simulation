# SA D2 Monte Carlo Spatial Analytics Freeze Audit (PLAT-SA-D2)

**Wave:** PLAT-SA-D2 — Monte Carlo Spatial Analytics  
**Plan:** [sa_d2_monte_carlo_spatial_analytics_plan.md](sa_d2_monte_carlo_spatial_analytics_plan.md)  
**Schemas:** [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md), [replay_spatial_analytics_v1.md](replay_spatial_analytics_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| `replay_mc_sweep_v1` manifests (4 sweep families) | Done |
| `scenario_sweeps_index_v1` catalog | Done |
| Spatial analytics overlays (Cesium grid layer) | Done |
| Sweep picker + metadata + `?sweep=` URL | Done |
| Analytics panels (distribution, LOS, clusters, variability) | Done |
| `export_replay_analytics_report.py` + committed reports | Done |
| Optional `matched_seed_comparison_report` fixture panel | Done |
| Compare strip when shared `sweep_id` | Done |
| `gen_d2_sweep_fixtures.py`, `aggregate_spatial_analytics.py` | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / engage / readiness UX | Pass |
| Explanatory-only spatial/analytics copy | Pass |
| No parser/topic changes | Pass |
| Additive schema only | Pass |
| GovernanceChrome not structurally redesigned | Pass |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py \
  src/counter_uas/test/test_replay_mc_sweep.py -q
python3 scripts/evaluation/sync_sa_catalog.py
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
```

## UX observations

- Spatial overlays default off; enabling ambiguity + LOS layers together remains readable at 40×30 grid on valley sweeps.
- Sweep member carousel preserves compare/compare-pair URLs when switching members (params updated).
- Matched-seed panel appears only for `matched_seed` sweeps with fixture JSON present.

## Limitations

- No N-slot compare filmstrip across all sweep members (D3).
- No live `scripts/monte_carlo.py` orchestration in viewer.
- Strict overlay `active_t_range` vs log span lint still deferred (D3).
- Compare-mode spatial layers follow focus-slot clock mirror only indirectly via shared sweep store.

## Recommended D3 scope

- Sweep member filmstrip / N-way compare carousel
- Pack-time overlay vs log span validator
- CLI import of live MC cohorts (no WebSocket)
- PLAN-VIZ-R2 rosbag/static Plotly merge
