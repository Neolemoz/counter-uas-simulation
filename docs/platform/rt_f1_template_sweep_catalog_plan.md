# RT-F1 — Template Sweep Catalog (PLAN-RT-F1)

**Phase:** PLAN-RT-F1 — sweep catalog planning (docs only)  
**Prerequisite:** PLAT-RT-S6 runtime templates, PLAT-RT-X1 batch CLI  
**Contract:** [rt_experiment_sweep_catalog_v1.md](../evaluation/rt_experiment_sweep_catalog_v1.md)  
**Fixture:** [fixtures/rt_experiments/sweep_catalog_v1.yaml](../../fixtures/rt_experiments/sweep_catalog_v1.yaml)

## Goal

Define **named, repeatable experiment sweep groups** that compile to `rt_experiment_batch_v1` using builtin `rt_runtime_template_v1` templates only — not SA scenario packs or orchestration queues.

## Sweep families

| Group | Templates | Intent |
|-------|-----------|--------|
| `ridge_defense_variants` | `radar_north_arc_v1`, `interceptor_ready_pair_v1` | Staging layout contrast |
| `sensor_range_variants` | `radar_valley_pair_v1`, `radar_north_arc_v1` | Sensor geometry contrast |
| `tactical_mode_comparison` | `interceptor_ready_pair_v1` + mode hints | Extends demo A/B pattern |

## Workflow

1. Maintainer selects group from catalog (fixture or future UI).  
2. Compile to batch YAML (`experiment_id`: `sweep-<group_id>-<date>`).  
3. Run `rt_experiment_batch.py` (X1).  
4. Derive analytics report (PLAT-RT-F1).  

## Boundaries

| Boundary | Rule |
|----------|------|
| Templates | Builtin catalog only — [template_catalog.py](../../platform/rt-sandbox-bridge/rt_sandbox/template_catalog.py) |
| SA corpus | `assert_template_ref_blocked` — no `fixtures/scenarios/` ids |
| Capture | Maintainer CLI only |
| Distributed | Forbidden (M3) |

## Allowed (PLAN wave)

- Contract + reference fixture YAML
- Documentation of compile strategies (`explicit_list` in fixture)

## Forbidden

- New templates in `template_catalog.py` (separate wave if needed)
- SA orchestration integration
- Implementation in PLAN wave

## PLAT-RT-F1 scope (advisory)

- `SweepCatalogBrowser.tsx` — load catalog, show compile snippet
- Optional `compileSweepGroup()` helper + vitest
- Optional `rt_experiment_sweep_compile.py` maintainer helper

## Related

- [rt_f1_experiment_analytics_plan.md](rt_f1_experiment_analytics_plan.md)
- [rt_experiment_workbench_v1.md](../evaluation/rt_experiment_workbench_v1.md)
