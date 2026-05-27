# RT Experiment Metrics Contract (`rt_experiment_metrics_v1`)

**Phase:** PLAN-RT-F5 — extended experiment metrics (docs); PLAT-RT-F5 implements derive; PLAN-RT-F5b §11 fidelity supplement (docs)  
**Prerequisite:** [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md) (PLAT-RT-F1), [rt_experiment_model_v1.md](rt_experiment_model_v1.md)  
**Authority:** [rt_f5_advanced_runtime_experiments_plan.md](../platform/rt_f5_advanced_runtime_experiments_plan.md); fidelity: [rt_f5b_runtime_fidelity_coupling_plan.md](../platform/rt_f5b_runtime_fidelity_coupling_plan.md)

F5 metrics are an **extension layer** on F1 analytics. They do **not** modify `rt_experiment_analytics_report_v1` schema (frozen with PLAT-RT-F1). Output uses a separate report schema; PLAT may compute both in one pass.

F5b adds an optional **third report** `rt_experiment_fidelity_metrics_report_v1` — separate from F5 `rt_experiment_metrics_report_v1` so frozen F5 derive remains stable.

---

## 1. Governance

| Rule | Detail |
|------|--------|
| Banner | `RT EXPERIMENT METRICS — derived summaries only; not operational authority` |
| Inputs | Stored manifest, optional batch spec, optional staging reads — **no live bridge pull** |
| Forbidden outputs | `success_rate`, `winner`, `best_run`, `readiness_index`, effectiveness / neutralization language |
| Terrain / visibility | Labels must include cognition disclaimer in UI (see [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md)) |

---

## 2. Inputs

| Input | Required | Use |
|-------|----------|-----|
| `rt_experiment_manifest_v1` | Yes | Runs, snapshots, F5 supplement fields |
| `rt_experiment_analytics_report_v1` | Yes | F1 `per_run` baseline — metrics extend, do not duplicate authority |
| `rt_experiment_batch_v1` | No | Join matrix/repeat metadata |
| `rt_experiment_spec_v1` | No | Expected cell count, class, fingerprint |
| Staging under `runs/rt_sandbox/captures/` | No | Normalization status for repeatability rollup |

---

## 3. Output: `rt_experiment_metrics_report_v1`

```json
{
  "schema": "rt_experiment_metrics_report_v1",
  "experiment_id": "exp-2026-05-26-ridge-matrix",
  "experiment_class": "parameter_matrix",
  "governance_banner": "RT EXPERIMENT METRICS — derived summaries only; not operational authority",
  "spec_fingerprint": "a1b2c3d4e5f67890",
  "per_run_extended": [],
  "compare_pairs_extended": [],
  "rollup_extended": {},
  "handoff_eligibility": {}
}
```

| Field | Rule |
|-------|------|
| `derived_at_utc` | Set only when persisting file; omit in pure derive |
| `experiment_class` | From spec or manifest majority; `unknown` if mixed without spec |

---

## 4. Per-run extended metrics (`per_run_extended[]`)

One object per manifest `run_id`. Merge with F1 `per_run` by `run_id` in UI — do not fork tactical authority.

| Field | Source | Notes |
|-------|--------|-------|
| `run_id` | manifest | |
| `experiment_class` | manifest supplement or spec | |
| `spec_fingerprint` | manifest / spec | |
| `matrix_coords` | manifest | `null` if not matrix class |
| `axis_signature` | derived | Sorted `axis_id=value` joined by `;` |
| `repeat_group_id` | manifest | |
| `repeat_index` | manifest | |
| `terrain_profile_ref` | manifest | |
| `nearest_ridge` | `terrain_context` or snapshot | cognition |
| `elevation_band` | `terrain_context` | |
| `f4_layers_enabled` | `visibility_context` or snapshot | string array; default `[]` |
| `los_cognition_label` | `visibility_context` | explanatory; not sensor truth |
| `occlusion_marker_count` | `visibility_context` | integer or `null` |
| `mode_at_capture` | F1 `tactical_mode` | alias for clarity in rollups |
| `assign_delta_from_prior` | derived | `boolean`; true when assigned id differs from prior run in manifest order with same fingerprint — **not** operational effectiveness |
| `autonomous_pause_count` | `tactical_annex_summary` | count field or `null` |
| `handoff_eligibility_hint` | derived gate summary | `eligible` \| `ineligible` \| `unknown` per-run; see §7 |

---

## 5. Rollup extended (`rollup_extended`)

Count-only aggregates. **Forbidden:** rates implying success, rankings, best cell.

| Block | Fields |
|-------|--------|
| `class_rollup` | `counts_by_class`: map class → run count |
| `terrain_rollup` | `ridge_counts`, `band_counts` — histograms |
| `visibility_rollup` | `los_label_counts` — distribution of `los_cognition_label` |
| `tactical_rollup` | `mode_counts` (may mirror F1), `assign_change_count`, `tti_present_count`, `annex_event_totals` (sum of annex timeline count keys) |
| `repeatability_rollup` | `fingerprints[]`: `{ spec_fingerprint, run_count, capture_count, normalization_status_counts }` |
| `matrix_rollup` | `expected_cells`, `populated_cells`, `missing_cells` — from spec cartesian product vs manifest |

---

## 6. Compare pairs extended (`compare_pairs_extended[]`)

Pairwise entries aligned with F1 `compare_pairs` (`run_id_a`, `run_id_b`).

### 6.1 Comparison criteria (normative gates)

| Criterion | Rule |
|-----------|------|
| Same `experiment_class` | Required for matrix row/column alignment |
| Same `spec_fingerprint` | Required for `repeatability_sweep` compare |
| Matrix peers | Same values on all axes except one compared axis |
| Capture parity | Both captures present or both absent; else badge `capture_asymmetric` |

### 6.2 Badge ids

**Reuse X1/F1:** `mode_changed`, `assignment_changed`, `tti_delta`, `pause_resume_delta`.

**F5 additive (explanatory only):**

| Badge id | Meaning |
|----------|---------|
| `terrain_context_diff` | `nearest_ridge` or `elevation_band` differs |
| `visibility_label_diff` | `los_cognition_label` differs |
| `annex_count_delta` | annex timeline total count differs |
| `matrix_axis_diff` | exactly one axis differs in `matrix_coords` |
| `capture_asymmetric` | one run has capture, other does not |
| `class_mismatch` | compare attempted across classes — informational only |

**Forbidden badge ids:** `winner`, `better`, `success`, `failure`.

---

## 7. Handoff eligibility (`handoff_eligibility`)

Advisory block for maintainer — **does not** invoke [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md) CLIs.

```json
{
  "experiment_level": "ineligible",
  "gates": [
    { "id": "all_captures_present", "pass": false, "detail": "2/4 runs missing capture" },
    { "id": "normalization_available", "pass": false, "detail": "staging unread" }
  ],
  "per_run_gates": []
}
```

| Gate id | Pass condition |
|---------|----------------|
| `all_captures_present` | Every run has `has_capture` true in F1 per_run |
| `normalization_available` | Staging read shows `normalization_status: normalized` for all captures, else `unavailable` → fail |
| `lifecycle_importable` | No run snapshot lifecycle in non-importable set per [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md) §6 |
| `pose_cognition_ack` | Maintainer flag in report metadata only — default `false` until PLAT UI checkbox (docs: manual attestation field `maintainer_ack_pose_reviewed`) |
| `fidelity_truth_ack` | Maintainer flag — default `false`; field `maintainer_ack_fidelity_truth_reviewed` (PLAT-RT-F5b); does not auto-import |
| `no_class_mismatch` | Single `experiment_class` across runs when comparing as one experiment |

`experiment_level`: `eligible` only if all gates `pass`; else `ineligible`; `partial` when some runs eligible (list in `per_run_gates`).

---

## 8. Determinism

Same rules as [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md) §4:

1. Sort `per_run_extended` by `run_id` ascending.  
2. Sort `compare_pairs_extended` by (`run_id_a`, `run_id_b`).  
3. Pure derive **must not** call `Date.now()`.  
4. Missing inputs → `null` or `"unavailable"` — no inference.  
5. Same manifest + F1 report + same optional inputs → identical body (excluding `derived_at_utc`).

---

## 9. Derivation API (PLAT-RT-F5)

```
deriveExperimentMetrics(manifest, f1Report, batchSpec?, spec?, stagingReader?) → metricsReport
buildExtendedComparePairs(per_run_extended, criteria) → compare_pairs_extended
rollupExtended(per_run_extended, spec?) → rollup_extended
evaluateHandoffEligibility(manifest, f1Report, stagingReader?, maintainerFlags?) → handoff_eligibility
```

Maintainer CLI (PLAT-RT-F5 P2): `scripts/rt/rt_experiment_metrics.py --manifest PATH [--batch PATH] [--analytics PATH] [--spec PATH] [--maintainer-ack-pose-reviewed] [--out PATH]`. Derives F1 when `--analytics` omitted; mirrors `deriveExperimentMetrics` in `metricsDerive.ts`.

Maintainer CLI (PLAT-RT-F5b P2 advisory): `scripts/rt/rt_experiment_fidelity_metrics.py` — same inputs plus optional staging read for `fidelity_pose_block` / `rt_fidelity_truth_snapshot_v1`; mirrors `deriveExperimentFidelityMetrics` in `fidelityMetricsDerive.ts`.

---

## 11. Fidelity metrics (`rt_experiment_fidelity_metrics_report_v1`) — PLAN-RT-F5b

Separate optional report. Does **not** modify F5 `per_run_extended` or F1 `per_run`. Truth fields are **derived mirrors** — not operational or SA replay authority.

### 11.1 Governance

| Rule | Detail |
|------|--------|
| Banner | `RT EXPERIMENT FIDELITY METRICS — truth-attested summaries are sim-scoped; not SA replay or operational sensor authority` |
| Inputs | Same as §2 plus optional `fidelity_context` on manifest runs and normalized capture `fidelity_pose_block` |
| vs F5 explanatory | `los_cognition_label` and F4 visibility remain explanatory; `los_truth_label` is truth-attested only when coupling was on at capture |
| Forbidden outputs | Same as §1 plus any claim of sensor coverage proof, detection, or engagement effectiveness |

### 11.2 Output schema

```json
{
  "schema": "rt_experiment_fidelity_metrics_report_v1",
  "experiment_id": "exp-2026-05-26-ridge-matrix",
  "governance_banner": "RT EXPERIMENT FIDELITY METRICS — truth-attested summaries are sim-scoped; not SA replay or operational sensor authority",
  "spec_fingerprint": "a1b2c3d4e5f67890",
  "coupling_required": true,
  "per_run_fidelity": [],
  "compare_pairs_fidelity": [],
  "rollup_fidelity": {}
}
```

| Field | Rule |
|-------|------|
| `coupling_required` | `true` when spec or manifest declares `enable_fidelity_coupling`; derive returns `unavailable` rollup when any run lacks truth snapshot |
| `derived_at_utc` | Set only when persisting file |

### 11.3 Per-run fidelity (`per_run_fidelity[]`)

| Field | Source | Notes |
|-------|--------|-------|
| `run_id` | manifest | |
| `fidelity_attestation_status` | capture / snapshot | `available` \| `stale` \| `unavailable` |
| `los_truth_label` | `rt_fidelity_truth_snapshot_v1` | truth-attested; `null` when coupling off |
| `los_cognition_label` | manifest `visibility_context` | explanatory — for divergence compare |
| `visibility_truth_ref` | truth snapshot | opaque ref string |
| `dome_truth_ref` | truth snapshot | opaque ref string |
| `pose_truth_drift_m` | `fidelity_pose_block` | max per-entity drift or `null` |
| `agl_truth_m` | `fidelity_pose_block` | representative `sim_agl_m` or `null` |
| `cognition_truth_divergence` | derived | `true` when `los_truth_label` present and ≠ `los_cognition_label` |

### 11.4 Compare pairs fidelity (`compare_pairs_fidelity[]`)

Aligned with F5 `compare_pairs_extended` run id pairs.

| Badge id | Meaning |
|----------|---------|
| `cognition_truth_divergence` | Either run has `cognition_truth_divergence` true |
| `pose_truth_drift_delta` | \|drift_a − drift_b\| > configured epsilon (explanatory) |
| `los_truth_label_diff` | truth labels differ |
| `fidelity_attestation_asymmetric` | one run `available`, other `unavailable` |

**Forbidden badge ids:** `winner`, `better`, `success`, `failure`, `more_accurate`.

### 11.5 Rollup fidelity (`rollup_fidelity`)

| Block | Fields |
|-------|--------|
| `attestation_rollup` | `status_counts`: map status → run count |
| `divergence_rollup` | `cognition_truth_divergence_count` |
| `repeatability_truth_rollup` | `truth_fingerprints[]`: `{ spec_fingerprint, truth_fingerprint, run_count, coupling_flag }` |

**`truth_fingerprint`:** stable hash of truth snapshot refs + `enable_fidelity_coupling` + adapter mode at capture. Repeatability under runtime truth requires same fingerprint **and** same `spec_fingerprint` — not manifest order alone.

### 11.6 Manifest supplement `fidelity_context` (optional)

On `rt_experiment_manifest_v1` run entries (PLAT-RT-F5b):

```json
{
  "fidelity_context": {
    "enable_fidelity_coupling": true,
    "adapter_mode": "live",
    "truth_snapshot_ref": "runs/rt_sandbox/captures/cap-m1/fidelity_truth.json"
  }
}
```

### 11.7 Determinism

Same rules as §8. Sort `per_run_fidelity` by `run_id`; sort compare pairs by (`run_id_a`, `run_id_b`). No inference when truth missing.

### 11.8 Derivation API (PLAT-RT-F5b advisory)

```
deriveExperimentFidelityMetrics(manifest, f5MetricsReport?, stagingReader?) → fidelityReport
buildFidelityComparePairs(per_run_fidelity) → compare_pairs_fidelity
rollupFidelity(per_run_fidelity, spec?) → rollup_fidelity
```

Reference fixtures: [fixtures/rt_experiments/f5b_fidelity_examples/](../../fixtures/rt_experiments/f5b_fidelity_examples/).

---

## 12. Related

- [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md)
- [rt_experiment_workflow_v1.md](rt_experiment_workflow_v1.md)
- [rt_experiment_advanced_ui_v1.md](rt_experiment_advanced_ui_v1.md)
- [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md)
- [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)
- [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md)
- [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md)
