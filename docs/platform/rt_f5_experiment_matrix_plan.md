# RT-F5 — Experiment Matrix & Repeatability (PLAN-RT-F5 companion)

**Phase:** PLAN-RT-F5 — compile semantics (docs only)  
**Prerequisite:** [rt_experiment_model_v1.md](../evaluation/rt_experiment_model_v1.md)  
**Authority:** [rt_f5_advanced_runtime_experiments_plan.md](rt_f5_advanced_runtime_experiments_plan.md)

Companion to the master F5 plan — defines **`cartesian`**, **`repeat_expand`**, and **`explicit_list`** compilation to `rt_experiment_batch_v1`. Mirrors the F1 split between [rt_f1_experiment_analytics_plan.md](rt_f1_experiment_analytics_plan.md) and [rt_f1_template_sweep_catalog_plan.md](rt_f1_template_sweep_catalog_plan.md).

---

## Goal

Normative rules for turning `rt_experiment_spec_v1` into a sequential maintainer batch queue, including stable `run_id` assignment and `spec_fingerprint` for repeatability gates.

---

## Compile strategies

| `compile_strategy` | Used by class | Output |
|--------------------|---------------|--------|
| `explicit_list` | terrain, sensor_range, tactical_mode | One batch run per `spec_entries[]` row |
| `cartesian` | `parameter_matrix` | Product of `matrix_axes[]` |
| `repeat_expand` | `repeatability_sweep` | `repeat_config.count` runs from `base_entry` |

---

## explicit_list

1. Iterate `spec_entries[]` in array order.  
2. `run_id` = `entry_id` unless collision → suffix `-2`, `-3`, …  
3. Each run row:

```yaml
run_id: <entry_id>
label: <label>
template_id: <template_id>
dwell_s: <entry.dwell_s or spec.default_dwell_s or 2.0>
# metadata carried to manifest supplements at pin time (not bridge fields):
# tactical_mode_hint, terrain_profile_ref, f4_layer_preset
```

4. `experiment_id` on batch = spec `experiment_id`.

---

## cartesian

1. Require `matrix_axes[]` length ≥ 1.  
2. Compute product size = ∏ `len(axis.values)`.  
3. **Iteration order:** outermost axis = first array element; innermost = last. For each combination, sort axis ids lexicographically when building `matrix_coords`.  
4. `run_id` = `m-` + joined axis values slugified (`[^a-z0-9]+` → `-`, lowercased), max 64 chars; truncate hash suffix if needed.  
5. `axis_signature` = sorted `axis_id=value` joined by `;`.  
6. `dwell_s` from axis when `dwell_s` is an axis; else `default_dwell_s`.  
7. `template_id` required from axis or single fixed entry in spec — error at compile if missing.  
8. Populate `manifest_expectation.min_run_count` = product size.

**Example:** axes `template_id` × `dwell_s` → 4 runs (see [parameter_matrix.json](../../fixtures/rt_experiments/f5_spec_examples/parameter_matrix.json)).

**Relation to sweep catalog:** [rt_experiment_sweep_catalog_v1.md](../evaluation/rt_experiment_sweep_catalog_v1.md) §3 `cartesian` was reserved for PLAT-RT-F1; F5 cartesian is **spec-authoritative** and supports the full axis whitelist in the model contract.

---

## repeat_expand

1. Require `repeat_config.count` ≥ 2, `jitter_s` = 0.  
2. `run_id` = `<base_entry_id or template>-r<index>` for `index` in `0 .. count-1`.  
3. `repeat_index` = index; `repeat_group_id` from config.  
4. `spec_fingerprint` = hash of repeat_config + base_entry (exclude experiment_id date).  
5. All runs share same `template_id` and `dwell_s` unless maintainer edits post-compile.

---

## spec_fingerprint

Computed at compile time and copied to every manifest run supplement:

```
fingerprint = sha256(canonical_json(spec_without_experiment_id_date_suffix))[0:16]
```

Used by [rt_experiment_metrics_v1.md](../evaluation/rt_experiment_metrics_v1.md) compare gates for repeatability class.

---

## Validation (compile-time, PLAT)

| Check | Action |
|-------|--------|
| Unknown `template_id` | Fail — must exist in builtin catalog |
| SA path in any field | Fail — `assert_template_ref_blocked` |
| Empty cartesian product | Fail |
| Forbidden axis id | Fail |
| `experiment_class` vs strategy mismatch | Warn in CLI; spec author responsible |

---

## Workflow

1. Maintainer authors spec JSON.  
2. Compile → `batch.yaml` at `compile_to_batch_path`.  
3. `python scripts/rt/rt_experiment_batch.py batch.yaml` (X1, unchanged).  
4. Manifest gains F5 supplements during/after batch.  
5. F1 + F5 derive per [rt_experiment_workflow_v1.md](../evaluation/rt_experiment_workflow_v1.md).

---

## Forbidden

- Parallel batch execution across bridge sessions  
- Random seeds or `jitter_s` > 0 in PLAN-RT-F5  
- SA scenario ids as matrix axis values  
- Bridge schema changes to carry matrix metadata

---

## PLAT advisory

- `experimentSpecCompile.ts` — pure functions + vitest golden files from `f5_spec_examples/`  
- `scripts/rt/rt_experiment_spec_compile.py --spec PATH --out PATH` — stdout batch YAML only

---

## Related

- [rt_experiment_model_v1.md](../evaluation/rt_experiment_model_v1.md)
- [rt_experiment_sweep_catalog_v1.md](../evaluation/rt_experiment_sweep_catalog_v1.md)
- [rt_f5_advanced_runtime_experiments_plan.md](rt_f5_advanced_runtime_experiments_plan.md)
