# SA B2 — Rich Replay Scenario Packs (PLAT-SA-B2)

**Status:** implementation wave (additive to PLAT-SA-C1a)

## Goal

Stress-test `scenario_topology_v1` with diverse replay scenarios: geometry, ingress behavior, ambiguity, and replay cognition challenges. Replay-content expansion only — not operational realism or live simulation.

## Allowed

- Eight scenario packs under `fixtures/scenarios/` (six new/expanded B2 + ridge + valley)
- Committed `fixtures/sa_r0/demo_<pack>/` synthetic logs + `replay_sa_bundle_v1` bundles
- Additive multi-track log parsing (`threat_id=`, `interceptor_id=` on guidance lines)
- Multi-threat LOS segment budget in `replay_sa_geometry.py`
- `gen_b2_scenarios.py` regen helper
- Viewer `DEMO_ALIASES` for `?demo=<pack_id>`
- Tests, `tier0-sa-r0` pack validation loop, freeze audit

## Forbidden

- ROS capture, rosbridge, WebSocket, runtime/topic/parser changes
- HITL, engage, readiness, tactical authority UX
- `GovernanceChrome` structure changes
- Scenario dropdown / compare UX (C1b)
- Photorealistic urban simulation

## Deliverables

| ID | Deliverable |
|----|-------------|
| B2.1 | Expanded `multi_ridge`, `corridor_defense` packs |
| B2.2 | New `saturation_ingress`, `urban_masking`, `delayed_detection`, `long_range_ingress` packs |
| B2.3 | Six demo bundles + per-pack narrative fixtures |
| B2.4 | Multi-track packager + LOS iteration |
| B2.5 | Catalog, CI, freeze audit with UX observations |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py \
  src/counter_uas/test/test_replay_sa_geometry.py -q
scripts/ci_eval.sh tier0-sa-r0
```
