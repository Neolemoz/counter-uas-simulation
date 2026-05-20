# SA C1a Scenario Schema Foundation Freeze Audit (PLAT-SA-C1a)

## Scope

- [scenario_schema_v1.md](scenario_schema_v1.md)
- `scripts/evaluation/replay_sa_scenario.py`, `scripts/evaluation/validate_scenario.py`
- `scripts/evaluation/replay_sa_bundle.py` (`--scenario-pack`, relative lineage, `scenario_pack_id`)
- `scripts/evaluation/replay_sa_geometry.py` (`load_scenario_topology`, `ingress_corridor` LOS class)
- `fixtures/scenarios/` (ridge_defense, valley_ingress, multi_ridge, corridor_defense skeletons)
- `platform/sa-r0-viewer/` metadata panel + `ingress_corridor` overlay style

No runtime, parser, topic, rosbridge, scenario editor UI, or compare-mode changes.

## Governance Result

**Verdict: frozen** for PLAT-SA-C1a.

| Check | Result |
|-------|--------|
| Authority creep | Pass — topology packs labeled fictional/explanatory |
| Parser safety | Pass — evaluation-side only; bundle schema additive fields |
| Runtime isolation | Pass — static packs and bundles |
| Operational semantics | Pass — no C2/engage/readiness |
| Dual source of truth | Pass — `scenario_overlay.json` removed from sa_r0 demos; packs are canonical |
| SA-R0 boundary | Pass — extends pack/viewer path; does not reopen runtime |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py src/counter_uas/test/test_replay_sa_bundle.py -q
for d in ridge_defense valley_ingress multi_ridge corridor_defense; do
  python3 scripts/evaluation/validate_scenario.py "fixtures/scenarios/$d"
done
cd platform/sa-r0-viewer && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-sa-r0
```

## Known Limitations

- `multi_ridge` and `corridor_defense` are validation skeletons; B2 adds rich replay content
- Scenario catalog (`fixtures/scenarios/index.json`) is not wired to viewer dropdown (C1b)
- Compare / Monte Carlo topology sweeps are not implemented (C1b foundations only)
- Legacy `--scenario-overlay` monolithic JSON remains supported for migration

## Follow-on

- **B2:** Complete — see [sa_b2_rich_scenario_freeze_audit.md](sa_b2_rich_scenario_freeze_audit.md)
- **C1b:** Scenario selection UX, provenance panel, comparison abstractions
