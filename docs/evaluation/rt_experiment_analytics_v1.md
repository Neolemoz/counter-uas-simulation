# RT Experiment Analytics Contract (`rt_experiment_analytics_v1`)

**Phase:** PLAN-RT-F1 — derived analytics (docs); PLAT-RT-F1 implements derive  
**Prerequisite:** PLAT-RT-X1 — [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)  
**Authority:** [rt_f1_experiment_analytics_plan.md](../platform/rt_f1_experiment_analytics_plan.md)

Analytics are **explanatory, deterministic summaries** from experiment artifacts. They are **not** parser contracts, operational readiness scores, or SA replay authority.

---

## 1. Governance

| Rule | Detail |
|------|--------|
| Banner | `RT ANALYTICS — derived summaries only; not operational authority` |
| Forbidden outputs | winner labels, readiness scores, effectiveness claims, threat neutralization language |
| Authority | Reports mirror manifest/capture inputs only |
| Bridge | No new commands; no live re-pull required for stored runs |

---

## 2. Inputs

| Input schema | Required | Use |
|--------------|----------|-----|
| `rt_experiment_manifest_v1` | Yes | Primary run records and snapshots |
| `rt_experiment_batch_v1` | No | Join `template_id`, `dwell_s`, `tactical_mode_hint` by `run_id` |
| Staging under `runs/rt_sandbox/captures/<id>/` | No | Optional read for normalization manifest / annex (maintainer or import) |

Pull channel payloads are read from `run.snapshot` when present; derivation does not call the bridge.

---

## 3. Output: `rt_experiment_analytics_report_v1`

```json
{
  "schema": "rt_experiment_analytics_report_v1",
  "experiment_id": "exp-2026-05-26-ridge-sweep",
  "derived_at_utc": "ISO-8601 (set only when persisting file; omit in pure derive)",
  "governance_banner": "RT ANALYTICS — derived summaries only; not operational authority",
  "per_run": [],
  "compare_pairs": [],
  "rollup": {}
}
```

### 3.1 Per-run metrics (`per_run[]`)

One object per manifest `run_id` (stable sort: lexicographic `run_id`).

| Field | Source | Notes |
|-------|--------|-------|
| `run_id` | manifest | |
| `label` | manifest | |
| `session_id_short` | manifest `session_id` | First 8 chars or full if shorter |
| `recorded_at_utc` | manifest | Timing |
| `dwell_s` | batch join | `null` if no batch spec |
| `template_id` | batch join | `null` if absent |
| `tactical_mode_hint` | batch join | Documentation only; not enforced at runtime |
| `entity_count` | `snapshot.world_summary.entity_count` | `null` if unavailable |
| `adapter_mode` | `snapshot.world_summary` | Explanatory mirror field |
| `sync_health` | `snapshot.world_summary` | Explanatory |
| `lifecycle_state` | `snapshot.lifecycle_state` | Opaque record or summary string |
| `tactical_mode` | `snapshot.tactical_state.tactical_mode` | At pin/capture time |
| `selected_id_short` | tactical_state | selected target/interceptor id truncated |
| `assigned_id_short` | tactical_state | assigned target/interceptor id truncated |
| `tti_s` | tactical_state | `null` if non-numeric |
| `autonomous_loop_status` | tactical_state | |
| `has_capture` | manifest | `capture_candidate_id != null` |
| `capture_candidate_id` | manifest | |
| `capture_staging_ref` | manifest | |
| `normalization_status_ref` | staging read | `"unavailable"` if not read; never inferred |
| `annex_timeline_counts` | `tactical_annex_summary.timeline_counts` | Copy or `null` |
| `annex_final_mode` | `tactical_annex_summary.final_tactical_mode` | |
| `terrain_nearest_ridge` | `snapshot.terrain_context.nearest_ridge` | V2 cognition; not terrain truth |

### 3.2 Tactical metrics (within per-run)

Annex fields are **counts only** when `tactical_annex_summary` is present — not full timeline replay (see TAC5 / SA3 boundaries). Full timeline review is **PLAT-RT-F3** via optional annex cache — see [rt_experiment_annex_review_ui_v1.md](rt_experiment_annex_review_ui_v1.md); F1 rollups remain count-based.

### 3.3 Timing metrics

- `recorded_at_utc` per run defines manifest ordering for trend views.
- `dwell_s` from batch spec is **configured wait**, not wall-clock session duration unless separately recorded.

### 3.4 Capture summaries

| Field | Rule |
|-------|------|
| `has_capture` | Boolean from manifest |
| `normalization_status_ref` | If maintainer reads `normalized_manifest.json` under staging, copy `normalization_status` field only; else `"unavailable"` |
| Annex block | Denormalized counts already on manifest run; do not upgrade to SA scrubber semantics |

### 3.5 Compare summaries (`compare_pairs[]`)

Pairwise only. Each entry:

```json
{
  "run_id_a": "run-a",
  "run_id_b": "run-b",
  "badges": [
    { "id": "mode_changed", "label": "mode_changed", "detail": "manual → assisted" }
  ]
}
```

Badge ids **must** match PLAT-RT-X1: `mode_changed`, `assignment_changed`, `tti_delta`, `pause_resume_delta`. Derivation uses the same rules as [experimentCompare.ts](../../platform/rt-sandbox-ui/src/experiment/experimentCompare.ts) `compareBadges`.

**Forbidden:** aggregate winner, best run, success rate.

### 3.6 Rollup (`rollup`)

Experiment-level **counts only**:

| Field | Meaning |
|-------|---------|
| `run_count` | `len(runs)` |
| `capture_count` | runs with `has_capture` |
| `mode_counts` | map `tactical_mode` → count (including `unknown`) |
| `template_ids_used` | distinct non-null template_id from batch join |

**Forbidden:** `success_rate`, `readiness_index`, effectiveness scores.

---

## 4. Determinism

1. Sort `per_run` by `run_id` ascending.  
2. Sort `compare_pairs` by (`run_id_a`, `run_id_b`) ascending.  
3. Pure derive function **must not** call `Date.now()` — optional `derived_at_utc` set only when writing file.  
4. Missing optional inputs → explicit `null` or `"unavailable"` — no default inference.  
5. Same manifest + same batch spec + same optional staging file bytes → identical report body (excluding `derived_at_utc`).

---

## 5. Derivation API (PLAT-RT-F1)

Specified for implementation; **not** implemented in PLAN-RT-F1:

```
deriveExperimentAnalytics(manifest, batchSpec?, stagingReader?) → report
buildComparePairs(report.per_run) → compare_pairs  // all pairs or selected pair list
rollupFromPerRun(per_run) → rollup
```

Optional maintainer CLI (PLAT-RT-F1): `scripts/rt/rt_experiment_analytics.py --manifest PATH [--batch PATH] [--out PATH]`.

---

## 6. Related

- [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md)
- [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md)
- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
