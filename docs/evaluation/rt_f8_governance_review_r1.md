# RT-F8 — Governance Review R1

**Phase:** PLAN-RT-F8 — post-F7 advisory maintainer expansion (docs only)  
**Plan:** [rt_f8_post_f7_advisory_maintainer_expansion_plan.md](../platform/rt_f8_post_f7_advisory_maintainer_expansion_plan.md)  
**Architecture review:** [rt_f8_architecture_review_r1.md](rt_f8_architecture_review_r1.md)  
**Contamination review:** [rt_f8_handoff_contamination_review_r1.md](rt_f8_handoff_contamination_review_r1.md)  
**Freeze audit:** [rt_f8_freeze_audit.md](rt_f8_freeze_audit.md)

---

## Scope verdict

| Question | Answer |
|----------|--------|
| Docs-only wave? | Yes |
| Bridge unchanged in PLAN? | Yes |
| SA viewer untouched? | Yes |
| Browser capture / commit forbidden? | Yes — unchanged |
| Advisory explanatory-only? | Yes |
| Parser/topic changes? | No |
| Distributed multi-bridge? | No |
| Automatic import? | No |
| `readiness_score` / operational readiness? | No — cohort v2 labels only |
| Federation writes from batch? | No |

**Recommendation:** Freeze **PLAN-RT-F8** (docs frozen).

---

## Authority boundaries

| Surface | Authoritative? |
|---------|----------------|
| `filter_preset` | **No** — filter/sort cognition |
| `focus_capture_ids` | **No** — row highlight only |
| `readiness_cohort_v2` | **No** — bulk cognition |
| `multi_capture_cohorts` | **No** — lane occupancy counts |
| `experiment_handoff_rollup` | **No** — warn-only |
| `handoff_rollup` | **No** — stage counts |
| `standup` template output | **No** — render-only |
| F6/F7 advisory ladder | **No** — unchanged semantics |
| SA1 handoff CLIs | **Yes** for staging writes |
| `rt_sa_import commit` | **Yes** for SA corpus lineage |

| Check | Result |
|-------|--------|
| Advisory ≠ parser contract | **Pass** |
| Advisory ≠ operational state | **Pass** |
| Preset ≠ CLI execution | **Pass** |
| Cohort v2 `ready_for_commit_advisory` ≠ committed | **Pass** |
| X2 experiment cohort ≠ readiness cohort v2 | **Pass** |

---

## RT↔SA separation

| Check | Result |
|-------|--------|
| No SA viewer changes in PLAN wave | **Pass** |
| No auto-import from preset or cohort v2 | **Pass** |
| F7 + F6 workflows unchanged | **Pass** |
| Lineage v2 detect does not write corpus | **Pass** |
| `capture_session ≠ SA import` preserved | **Pass** |
| X2 review packet ≠ handoff authority | **Pass** |

---

## Forbidden lexicon

| Term | PLAN status |
|------|-------------|
| `readiness_score` | Forbidden |
| `auto_import` | Forbidden |
| `operational_ready` | Forbidden |
| `tactical readiness` | Forbidden |
| `--commit-all` | Forbidden |

---

## Registry and vocabulary

| Check | Result |
|-------|--------|
| PLAN-RT-F8 ≠ PLAT-RT-F8 | **Pass** — documented |
| PLAN-RT-F8 ≠ PLAT-RT-F7 replacement | **Pass** |
| PLAN-RT-F8 ≠ registry RT-1..7 | **Pass** |
| Experiment cohort (X2) ≠ readiness cohort v2 | **Pass** |

---

## Verdict

**Pass** — PLAN-RT-F8 suitable for docs freeze. PLAT-RT-F8 phases require per-phase governance + contamination review before implementation authorization.
