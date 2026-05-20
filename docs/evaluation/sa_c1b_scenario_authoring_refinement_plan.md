# SA C1b — Scenario Authoring Refinement (PLAT-SA-C1b)

**Status:** implementation wave (additive to PLAT-SA-C1a, PLAT-SA-B2)

## Goal

Refine scenario portability, discoverability, replay cognition, provenance clarity, and comparison foundations using the B2 replay library — without realtime editing or operational semantics.

## Allowed

- C1b metadata fields on `scenario_topology_v1` packs
- Catalog enrichment + viewer scenario picker (`fixtures/scenarios/index.json` → `public/demo/catalog.json`)
- Provenance panel in SA-R0 viewer
- Replay cognition: annotation emphasis, LOS declutter, timeline selection sync
- `comparison_hints` on bundles + `comparison_foundations.md`
- `sync_sa_catalog.py` repack helper
- Tests, tier0-sa-r0, freeze audit

## Forbidden

- WebSocket/rosbridge/live ROS integration
- Realtime topology editing, world editors, drag/drop
- HITL, engage, readiness, tactical authority UX
- Full compare mode UI (D1)
- GovernanceChrome structural redesign
- Operational severity labels (`narrative_rank` is replay emphasis only)

## Deliverables

| ID | Deliverable |
|----|-------------|
| C1b.1 | Extended metadata + validation |
| C1b.2 | Catalog sync + scenario picker UX |
| C1b.3 | Provenance panel |
| C1b.4 | Replay cognition refinements |
| C1b.5 | Comparison foundations doc + bundle hooks |
| C1b.6 | CI, freeze audit, governance updates |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py -q
python3 scripts/evaluation/sync_sa_catalog.py
scripts/ci_eval.sh tier0-sa-r0
```
