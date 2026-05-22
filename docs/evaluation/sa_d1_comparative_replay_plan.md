# SA D1 — Comparative Replay & Topology Experiments (PLAT-SA-D1)

**Status:** implementation wave (additive to PLAT-SA-C1b)

## Goal

Evolve the platform from single replay exploration into comparative topology experimentation and replay A/B analysis while preserving replay-first, deterministic, explanatory-only governance.

## Allowed

- Side-by-side compare viewer mode (dual bundle, dual clock optional sync)
- Topology diff summaries and map delta highlighting
- Replay outcome comparison panel (replay-side metrics only)
- Sensor placement experiment packs (`fixtures/scenarios/valley_ingress_*`)
- `scenario_compare_pairs_v1` manifest + `replay_compare_v1.md`
- Extended `comparison_hints` (`sensor_layout_id`, `compare_mode`, baseline/paired keys)
- `sync_sa_catalog.py` experiment repack (shared valley log)
- Compare cognition UX (annotation alignment, declutter presets)
- Tests, tier0-sa-r0, freeze audit

## Forbidden

- WebSocket/rosbridge/live ROS integration
- HITL, engage, readiness, tactical authority UX
- Operational effectiveness or superiority framing
- Parser/topic contract changes
- GovernanceChrome structural redesign
- Realtime topology editing

## Deliverables

| ID | Deliverable |
|----|-------------|
| D1.1 | Compare mode UI + compareStore |
| D1.2 | Topology diff + outcome panels |
| D1.3 | Sensor experiment fixtures (4 packs) |
| D1.4 | Compare pairs manifest + schema docs |
| D1.5 | Cognition UX (alignment, declutter) |
| D1.6 | CI, freeze audit, registry |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py -q
python3 scripts/evaluation/sync_sa_catalog.py
scripts/ci_eval.sh tier0-sa-r0
```
