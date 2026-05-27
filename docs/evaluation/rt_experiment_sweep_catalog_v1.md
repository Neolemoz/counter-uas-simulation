# RT Experiment Sweep Catalog Contract (`rt_experiment_sweep_catalog_v1`)

**Phase:** PLAN-RT-F1 — template sweep recipes (docs)  
**Prerequisite:** PLAT-RT-S6 templates, PLAT-RT-X1 batch workflow  
**Authority:** [rt_f1_template_sweep_catalog_plan.md](../platform/rt_f1_template_sweep_catalog_plan.md)

The sweep catalog is **RT-local**: named groups of experiment entries that compile to `rt_experiment_batch_v1` specs. It is **not** SA scenario corpus, not H3/I1 orchestration queues, and not federation manifests.

---

## 1. Governance

| Rule | Detail |
|------|--------|
| Templates | Must be builtin `rt_runtime_template_v1` ids from [template_catalog.py](../../platform/rt-sandbox-bridge/rt_sandbox/template_catalog.py) |
| SA paths | Forbidden — `template_id` must pass `assert_template_ref_blocked` |
| Capture | Only via existing `rt_experiment_batch.py` (maintainer); browser forbidden |
| Semantics | Sweep labels are organizational only — no operational claims |

---

## 2. Schema: `rt_experiment_sweep_catalog_v1`

```json
{
  "schema": "rt_experiment_sweep_catalog_v1",
  "catalog_id": "rt_sweep_catalog_v1",
  "governance_banner": "RT SWEEP CATALOG — local template recipes only; not SA orchestration",
  "sweep_groups": []
}
```

### 2.1 Sweep group

| Field | Required | Rule |
|-------|----------|------|
| `group_id` | Yes | Stable id, e.g. `ridge_defense_variants` |
| `label` | Yes | Human title |
| `description` | Yes | Explanatory purpose |
| `governance_banner` | No | Optional group-level line |
| `compile_strategy` | Yes | `explicit_list` or `cartesian` (see §3) |
| `default_dwell_s` | No | Default for generated batch spec |
| `sweep_entries` | Yes | Non-empty array |

### 2.2 Sweep entry

| Field | Required | Rule |
|-------|----------|------|
| `entry_id` | Yes | Unique within group |
| `label` | Yes | Run label in generated batch |
| `template_id` | Yes | Must exist in builtin catalog |
| `dwell_s` | No | Overrides group default |
| `tactical_mode_hint` | No | Docs/batch metadata only — not bridge-enforced |
| `run_id_suffix` | No | Default: `entry_id` used as `run_id` in batch |

---

## 3. Compile strategies

### `explicit_list`

Each `sweep_entry` becomes one `runs[]` row in `rt_experiment_batch_v1` in entry order.

### `cartesian`

Reserved for PLAT-RT-F1 tooling. Documented pattern: one dimension `template_id` × optional `tactical_mode_hint` list. PLAN-RT-F1 fixture uses `explicit_list` only.

### Generated batch spec

| Field | Value |
|-------|-------|
| `schema` | `rt_experiment_batch_v1` |
| `experiment_id` | `sweep-<group_id>-<YYYYMMDD>` (maintainer chooses date) |
| `default_dwell_s` | From group or catalog default (2.0s documented default) |
| `runs[]` | Compiled entries |

Manifest path after batch: `runs/rt_sandbox/experiments/<experiment_id>/manifest.json` (X1 convention).

---

## 4. Named group families (reference)

| `group_id` | Purpose | Example templates |
|------------|---------|-------------------|
| `ridge_defense_variants` | Radar + interceptor staging contrast | `radar_north_arc_v1`, `interceptor_ready_pair_v1` |
| `sensor_range_variants` | Layout / entity count contrast | `radar_valley_pair_v1`, `radar_north_arc_v1` |
| `tactical_mode_comparison` | Same world, mode hints differ | `interceptor_ready_pair_v1` × manual/assisted/autonomous hints |
| `ingress_lane_baseline` | Single-drone ingress | `drone_ingress_lane_v1` |

Reference fixture: [fixtures/rt_experiments/sweep_catalog_v1.yaml](../../fixtures/rt_experiments/sweep_catalog_v1.yaml).

---

## 5. Maintainer workflow

1. Select sweep group from catalog (UI browser or fixture).  
2. Compile or copy generated `rt_experiment_batch_v1` YAML.  
3. Run `python3 scripts/rt/rt_experiment_batch.py --spec <path>`.  
4. Import or open manifest; derive analytics report (PLAT-RT-F1).  

No step may invoke SA packager or federation register from RT UI.

---

## 6. Sweep rollups (analytics)

When a manifest was produced from a catalog group, optional metadata:

```json
{
  "sweep_group_id": "ridge_defense_variants",
  "catalog_id": "rt_sweep_catalog_v1"
}
```

May be stored in manifest `experiment_id` prefix or sidecar `sweep_meta.json` (PLAT-RT-F1 choice). Rollup fields: `run_count`, `capture_count`, `mode_counts` — see [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md).

---

## 7. Related

- [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md)
- [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md)
- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
