# Scenario topology packs (`scenario_topology_v1`)

Portable replay topology definitions under `fixtures/scenarios/<pack_id>/`.

## B2 rich replay library

| Pack | Lesson | Demo bundle |
|------|--------|-------------|
| `ridge_defense` | Ridge occlusion, TTI selection | `fixtures/sa_r0/demo_ridge_defense/` |
| `valley_ingress` | Valley masking, reacquisition | `fixtures/sa_r0/demo_valley_ingress/` |
| `multi_ridge` | Chained ridge masking, intermittent visibility | `fixtures/sa_r0/demo_multi_ridge/` |
| `corridor_defense` | Narrow corridor, compressed timing | `fixtures/sa_r0/demo_corridor_defense/` |
| `saturation_ingress` | Multi-threat, assignment ambiguity | `fixtures/sa_r0/demo_saturation_ingress/` |
| `urban_masking` | Fictional obstruction clutter | `fixtures/sa_r0/demo_urban_masking/` |
| `delayed_detection` | Late acquisition, urgency | `fixtures/sa_r0/demo_delayed_detection/` |
| `long_range_ingress` | Long-form pacing, launch progression | `fixtures/sa_r0/demo_long_range_ingress/` |

Viewer: `npm run dev` in `platform/sa-r0-viewer/`, then `?demo=<pack_id>` (see `DEMO_ALIASES` in `loadBundle.ts`).

Validate all packs:

```bash
for d in ridge_defense valley_ingress multi_ridge corridor_defense \
  saturation_ingress urban_masking delayed_detection long_range_ingress; do
  python3 scripts/evaluation/validate_scenario.py "fixtures/scenarios/$d"
done
```

Regenerate B2 demos:

```bash
python3 scripts/evaluation/gen_b2_scenarios.py
```

Schema: [docs/evaluation/scenario_schema_v1.md](../../docs/evaluation/scenario_schema_v1.md)
