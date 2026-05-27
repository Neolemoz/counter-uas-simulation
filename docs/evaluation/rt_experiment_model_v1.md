# RT Experiment Model Contract (`rt_experiment_model_v1`)

**Phase:** PLAN-RT-F5 — advanced experiment taxonomy (docs only)  
**Prerequisite:** PLAT-RT-X1, PLAT-RT-F1, PLAT-RT-F3, PLAT-RT-F4 frozen  
**Authority:** [rt_f5_advanced_runtime_experiments_plan.md](../platform/rt_f5_advanced_runtime_experiments_plan.md), [rt_f5_experiment_matrix_plan.md](../platform/rt_f5_experiment_matrix_plan.md)

Defines **`rt_experiment_spec_v1`** (build intent) and **experiment classes** that compile to existing X1 batch/manifest artifacts. Specs are **explanatory planning** — not bridge authority, not SA replay truth.

---

## 1. Governance

| Rule | Detail |
|------|--------|
| Banner | `RT EXPERIMENT — local maintainer planning only; not operational authority` |
| Forbidden | winner labels, readiness scores, effectiveness claims, threat neutralization language, statistical proof claims |
| Templates | Builtin `rt_runtime_template_v1` ids only — same as [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md) |
| SA paths | Forbidden on `template_id` and all refs — `assert_template_ref_blocked` pattern at PLAT compile time |
| Terrain / visibility | F4/V2 **cognition** only — not terrain truth or operational sensor coverage |

---

## 2. Experiment classes (`experiment_class`)

Normative enum. Every spec declares exactly one class.

| Class | Intent | Primary dimensions |
|-------|--------|-------------------|
| `terrain_comparison` | Compare runs under different terrain cognition / F4 layer profiles | `terrain_profile_ref`, `f4_layer_preset`, optional `template_id` |
| `sensor_range_comparison` | Compare builtin template variants affecting **explanatory** sensor dome / range fiction | `template_id` sweep entries |
| `tactical_mode_comparison` | Compare manual / assisted / autonomous at pin/capture time | `tactical_mode_hint`, runtime `tactical_mode` in snapshot |
| `repeatability_sweep` | Same spec repeated N times for stability cognition | `repeat_group_id`, `repeat_index`, `spec_fingerprint` |
| `parameter_matrix` | Cartesian product of documented parameters | `matrix_axes[]`, `compile_strategy: cartesian` |

**Not registry RT-1..7:** experiment classes describe RT sandbox maintainer workflows only.

---

## 3. Schema: `rt_experiment_spec_v1`

```json
{
  "schema": "rt_experiment_spec_v1",
  "experiment_id": "exp-2026-05-26-ridge-matrix",
  "experiment_class": "parameter_matrix",
  "governance_banner": "RT EXPERIMENT — local maintainer planning only; not operational authority",
  "hypothesis_label": "Explanatory note for maintainer review only",
  "compile_strategy": "cartesian",
  "matrix_axes": [],
  "repeat_config": null,
  "default_dwell_s": 2.0,
  "manifest_expectation": {
    "min_run_count": 4,
    "required_snapshot_channels": ["tactical_state", "world_summary", "lifecycle_state"]
  },
  "compile_to_batch_path": "runs/rt_sandbox/experiments/<experiment_id>/batch.yaml"
}
```

### 3.1 Core fields

| Field | Required | Rule |
|-------|----------|------|
| `schema` | Yes | `rt_experiment_spec_v1` |
| `experiment_id` | Yes | Stable string; matches manifest/batch `experiment_id` after run |
| `experiment_class` | Yes | One of §2 enum |
| `governance_banner` | Yes | Required line (§1) |
| `hypothesis_label` | No | Human text; forbidden lexicon per §1 |
| `compile_strategy` | Yes | `explicit_list`, `cartesian`, or `repeat_expand` (see [rt_f5_experiment_matrix_plan.md](../platform/rt_f5_experiment_matrix_plan.md)) |
| `default_dwell_s` | No | Default 2.0 when omitted |
| `manifest_expectation` | No | Post-run validation hints for maintainer |
| `compile_to_batch_path` | No | Repo-relative path convention for compiled `rt_experiment_batch_v1` |

### 3.2 Matrix axes (`matrix_axes[]`)

Required when `experiment_class` is `parameter_matrix` and `compile_strategy` is `cartesian`.

| Field | Rule |
|-------|------|
| `axis_id` | Stable id |
| `values` | Non-empty array of strings or numbers (serialized as strings in batch metadata) |

**Allowed axis ids (whitelist):**

| `axis_id` | Maps to batch/run metadata |
|-----------|----------------------------|
| `template_id` | `template_id` per run |
| `tactical_mode_hint` | `tactical_mode_hint` (docs only; not bridge-enforced) |
| `dwell_s` | per-run `dwell_s` |
| `terrain_profile_ref` | manifest supplement `terrain_profile_ref` |
| `f4_layer_preset` | doc-only preset id → manifest `visibility_context.f4_layer_preset` |

Forbidden axes: SA scenario ids, parser topic names, seed lists implying Monte Carlo proof.

### 3.3 Repeat config (`repeat_config`)

Required when `experiment_class` is `repeatability_sweep`.

```json
{
  "repeat_group_id": "repeat-ridge-baseline",
  "count": 3,
  "jitter_s": 0,
  "base_entry": {
    "template_id": "radar_north_arc_v1",
    "label": "ridge baseline"
  }
}
```

| Rule | Detail |
|------|--------|
| `count` | Integer ≥ 2 |
| `jitter_s` | Must be `0` in PLAN-RT-F5 — no random timing |
| `spec_fingerprint` | PLAT compile: stable hash of canonical JSON (excluding `experiment_id` date suffix) |

### 3.4 Explicit entries (`spec_entries[]`)

Used when `compile_strategy` is `explicit_list` (all classes except pure `repeat_expand` / `cartesian`).

| Field | Rule |
|-------|------|
| `entry_id` | Unique within spec |
| `label` | Run label in batch/manifest |
| `template_id` | Builtin catalog id |
| `dwell_s` | Optional override |
| `tactical_mode_hint` | Optional |
| `terrain_profile_ref` | Optional — V2/F4 doc ref |
| `f4_layer_preset` | Optional — e.g. `contours_on`, `vegetation_markers_on` (default-off presets) |

---

## 4. Compile output: `rt_experiment_batch_v1`

Compilation **must** produce [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md) §2 batch schema:

| Field | Value |
|-------|-------|
| `schema` | `rt_experiment_batch_v1` |
| `experiment_id` | From spec |
| `runs[]` | One row per compiled run, stable order documented in matrix plan |
| Per-run metadata | `run_id`, `template_id`, `dwell_s`, optional hints |

**PLAN-RT-F5:** compile specified in docs; implementation is **PLAT-RT-F5** only.

---

## 5. Manifest supplements (F5 additive fields on `runs[]`)

Optional fields on [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md) run records — bridge does not set these; batch CLI or maintainer manifest edit after pin:

| Field | Rule |
|-------|------|
| `experiment_class` | Copy from spec |
| `spec_fingerprint` | Stable compile fingerprint |
| `matrix_coords` | Map `axis_id` → chosen value when matrix class |
| `repeat_index` | 0-based index in repeat group |
| `repeat_group_id` | From `repeat_config` |
| `terrain_profile_ref` | Doc ref string |
| `visibility_context` | Explanatory F4 snapshot at pin: `f4_layer_preset`, `los_cognition_label`, `occlusion_marker_count` |
| `handoff_eligibility_hint` | **Forbidden** on manifest from bridge — set only by metrics derive per [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md) |

---

## 6. `spec_fingerprint`

Deterministic fingerprint for repeatability and compare gates:

1. Canonicalize spec JSON: sort keys; omit `experiment_id` date suffix if pattern `exp-YYYY-MM-DD-*` (PLAT may use full id).  
2. SHA-256 hex digest, first 16 chars → `spec_fingerprint`.  
3. Same spec body → same fingerprint across compile runs.

---

## 7. Related

- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
- [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md)
- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md)
- [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md)
- [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md)
- [rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md)
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
