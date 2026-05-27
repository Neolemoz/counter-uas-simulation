# RT-F5 — Governance Review R1

**Phase:** PLAN-RT-F5 — advanced runtime experiments (docs only)  
**Plan:** [rt_f5_advanced_runtime_experiments_plan.md](../platform/rt_f5_advanced_runtime_experiments_plan.md)  
**Architecture review:** [rt_f5_architecture_review_r1.md](rt_f5_architecture_review_r1.md)  
**Freeze audit:** [rt_f5_freeze_audit.md](rt_f5_freeze_audit.md)

---

## Scope verdict

| Question | Answer |
|----------|--------|
| Docs-only wave? | Yes |
| Bridge unchanged? | Yes |
| SA viewer untouched? | Yes |
| Browser capture forbidden? | Yes — unchanged X1 |
| Metrics explanatory-only? | Yes |
| Parser/topic changes? | No |
| Tactical redesign? | No |
| Distributed / M3? | No |

**Recommendation:** Freeze **PLAN-RT-F5** (docs frozen).

---

## Authority boundaries

| Surface | Authoritative? |
|---------|----------------|
| `rt_experiment_metrics_report_v1` | **No** — derived mirror |
| `rt_experiment_spec_v1` | **No** — planning artifact |
| Manifest / capture files | **No** for replay — staging only |
| `handoff_eligibility` | **No** — advisory gates only |
| SA replay bundle | **Yes** in SA viewer only — F5 does not write bundles |

| Check | Result |
|-------|--------|
| Metrics ≠ parser contract | **Pass** |
| Metrics ≠ operational state | **Pass** |
| Eligibility ≠ approve/import | **Pass** |

---

## RT↔SA separation

| Check | Result |
|-------|--------|
| No SA viewer changes in PLAN wave | **Pass** |
| No auto-import from metrics or eligibility | **Pass** |
| Spec/sweep not SA scenario packs | **Pass** |
| SA3 replay remains read-only | **Pass** |
| SA1 manual workflow unchanged | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| F5-GOV-SA-01 | Pass |

---

## Replay boundaries

| Check | Result |
|-------|--------|
| F5 does not embed SA scrubber | **Pass** |
| Annex counts / F3 hub unchanged in authority | **Pass** |
| Trend views not time-series authority | **Pass** |

---

## Deny-by-default

| Check | Result |
|-------|--------|
| No new bridge commands in PLAN | **Pass** |
| No browser `capture_session` in PLAN | **Pass** |
| No federation from experiment UI plan | **Pass** |
| Forbidden lexicon in contracts | **Pass** |
| No distributed batch queue | **Pass** |

---

## Banner policy

| Banner | Policy |
|--------|--------|
| `BANNER_EXPERIMENT_F5` | Required on F5 surfaces; additive to `BANNER_ANALYTICS` |
| Cognition labels | Terrain/visibility strips prefixed in UI contract |

---

## Session isolation

| Check | Result |
|-------|--------|
| Multi-session workstation compatible | **Pass** |
| Batch serial execution preserved | **Pass** |
| `session_id` not used as compare authority | **Pass** |

---

## Registry RT-1..7

| Check | Result |
|-------|--------|
| PLAN-RT-F5 ≠ registry realism waves | **Pass** — documented in plan + registry |
| F5 ≠ Gazebo sensor-truth coupling | **Pass** — deferred to fidelity frontier |

---

## Governance verdict

**Pass** — Freeze PLAN-RT-F5. Do not start PLAT-RT-F5 without implementation governance review + freeze audit.
