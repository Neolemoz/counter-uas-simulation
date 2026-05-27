# RT-F1 — Analytics Review R1

**Phase:** PLAN-RT-F1 — experiment analytics semantics  
**Contract:** [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md)  
**UI contract:** [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md)

---

## 1. Metric completeness

| Goal area | Contract coverage | Verdict |
|-----------|-------------------|---------|
| Per-run metrics | `per_run[]` entity/sync/lifecycle | **Pass** |
| Tactical metrics | mode, assign, TTI, annex counts | **Pass** |
| Timing metrics | `recorded_at_utc`, `dwell_s` join | **Pass** |
| Capture summaries | has_capture, staging ref, normalization ref | **Pass** |
| Compare summaries | `compare_pairs[]` + X1 badge ids | **Pass** |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| F1-AN-01 | Pass | All user goal areas mapped to schema |
| F1-AN-02 | Pass-with-conditions | Wall-clock session duration not in v1 — use batch `dwell_s` only |

---

## 2. Determinism

| Rule | Review |
|------|--------|
| Stable sort by `run_id` | Documented |
| No wall-clock in pure derive | Documented |
| Missing → null/unavailable | Documented |
| Compare pairs sorted | Documented |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| F1-AN-03 | Pass | Determinism rules sufficient for PLAT implementation |
| F1-AN-04 | Pass | Hash policy delegated to PLAT with ordering rules fixed |

---

## 3. Forbidden lexicon

| Forbidden | In contracts? |
|-----------|---------------|
| winner / best run | Absent — **Pass** |
| readiness / success rate | Explicitly forbidden in rollup — **Pass** |
| effectiveness / engage / intercept | Absent — **Pass** |
| operational authority claims | Governance banner — **Pass** |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| F1-AN-05 | Pass | Analytics review suitable for freeze |

---

## 4. Compare alignment with X1

| X1 badge | Analytics `compare_pairs` | Match |
|----------|---------------------------|-------|
| `mode_changed` | Yes | **Pass** |
| `assignment_changed` | Yes | **Pass** |
| `tti_delta` | Yes | **Pass** |
| `pause_resume_delta` | Yes | **Pass** |

Derivation must reuse same logic as `compareBadges` in PLAT-RT-F1.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| F1-AN-06 | Pass | Compare semantics aligned |

---

## 5. Sweep rollups

| Field | Semantics | Verdict |
|-------|-----------|---------|
| `run_count` | Count | **Pass** |
| `capture_count` | Count | **Pass** |
| `mode_counts` | Distribution | **Pass** |
| Success rate | Forbidden | **Pass** |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| F1-AN-07 | Pass | Rollups are counts only |

---

## 6. Analytics verdict

**Pass** — [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md) and UI contract are complete for PLAN freeze. PLAT-RT-F1 should implement derive + tests per contract §5.
