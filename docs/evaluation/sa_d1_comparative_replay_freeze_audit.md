# SA D1 Comparative Replay Freeze Audit (PLAT-SA-D1)

**Wave:** PLAT-SA-D1 — Comparative Replay & Topology Experiments  
**Plan:** [sa_d1_comparative_replay_plan.md](sa_d1_comparative_replay_plan.md)  
**Schema:** [replay_compare_v1.md](replay_compare_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| Side-by-side compare mode (`compareStore`, `CompareView`) | Done |
| URL routing `?pair=`, `?compare=packA,packB` | Done |
| Topology diff panel + map delta highlighting | Done |
| Replay outcome comparison panel | Done |
| Sensor placement experiment packs (4) | Done |
| `compare_pairs_v1.json` manifest | Done |
| Extended `comparison_hints` (`sensor_layout_id`, `compare_mode`) | Done |
| Annotation alignment panel | Done |
| `sync_sa_catalog.py` experiment repack | Done |
| Governance compare badge + footer caveat | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / engage / readiness UX | Pass |
| Explanatory-only compare copy | Pass |
| No parser/topic changes | Pass |
| Additive bundle/catalog schema only | Pass |
| GovernanceChrome not structurally redesigned | Pass |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py -q
python3 scripts/evaluation/sync_sa_catalog.py
scripts/ci_eval.sh tier0-sa-r0
```

## UX observations

- Dual Cesium maps at 50% width remain readable for valley/ridge overlays; dense multi-threat scenarios benefit from “emphasize delta” toggle.
- Shared replay clock works when both bundles share log-line domain; duration mismatch is surfaced in outcome panel Δ column.
- Focus-slot mock panes avoid duplicating six mirrors while keeping compare maps independent.

## Limitations

- No import of `matched_seed_comparison_report` JSON (D2).
- No Monte Carlo catalog sweep runner (D2).
- Overlay `active_t_range` vs log span not strictly linted at pack time.
- Layer toggles in compare mode update focus slot + clock mirror; non-focus slot layers require focus switch to adjust.
- Compare mode does not export static `replay_compare_report_v1` artifact.

## Recommended D2 scope

- Wire `replay_observability.py paired-comparison` artifacts into viewer
- `sweep_id` + MC batch runner bound to scenario catalog
- Strict overlay vs log span validator
- Deterministic compare report JSON export
- Optional full dual mock-pane columns

**Verdict: frozen** for PLAT-SA-D1.
