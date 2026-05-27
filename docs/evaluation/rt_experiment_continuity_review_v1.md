# RT Experiment Continuity Review (`rt_experiment_continuity_review_v1`)

**Phase:** PLAN-RT-F3 — integration contract  
**Prerequisite:** [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md), [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md)

Defines how manifest runs, F1 analytics, and optional full annex join in a single review hub.

---

## 1. Inputs per selected run

| Layer | Source |
|-------|--------|
| Run identity | `rt_experiment_manifest_v1.runs[]` |
| Analytics row | `deriveExperimentAnalytics` → `per_run` by `run_id` |
| Annex summary | `run.tactical_annex_summary` (counts on manifest) |
| Full annex | `rt_experiment_annex_cache_v1[run_id]` optional |
| Capture lineage | `capture_candidate_id`, `capture_staging_ref`, `normalization_status_ref` |

---

## 2. Capture lineage (explanatory)

Display handoff pipeline phases A–E vocabulary from RT capture/handoff cognition — **read-only**, no CLI execution from browser.

| Field | Meaning |
|-------|---------|
| `has_capture` | From analytics `per_run` |
| `capture_staging_ref` | Staging path hint for maintainer |
| `normalization_status_ref` | `unavailable` unless maintainer read |

---

## 3. Compare mode extension

When X1 compare uses two **pinned** runs with `run_id` A and B:

| Surface | Content |
|---------|---------|
| X1 compare | Unchanged — tactical/telemetry badges |
| Annex compare strip | Final mode/selected/assigned side-by-side; timeline **counts** table; badges `mode_final_diff`, `annex_missing_*` |

No full dual timeline tables in compare strip (space); full timelines in continuity hub per run.

---

## 4. Authority

| Artifact | Authoritative? |
|----------|----------------|
| Manifest snapshot at pin time | Explanatory mirror |
| Analytics report | Derived — not operational |
| Full annex cache | Replay-boundary scoped — not SA authority |
| Compare badges | Explanatory only |

---

## Related

- [rt_experiment_annex_review_ui_v1.md](rt_experiment_annex_review_ui_v1.md)
- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
