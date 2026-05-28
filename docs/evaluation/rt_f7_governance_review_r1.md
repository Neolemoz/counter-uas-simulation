# RT-F7 — Governance Review R1

**Phase:** PLAN-RT-F7 — post-F6 advisory expansion (docs only)  
**Plan:** [rt_f7_post_f6_advisory_expansion_plan.md](../platform/rt_f7_post_f6_advisory_expansion_plan.md)  
**Architecture review:** [rt_f7_architecture_review_r1.md](rt_f7_architecture_review_r1.md)  
**Contamination review:** [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md)  
**Freeze audit:** [rt_f7_freeze_audit.md](rt_f7_freeze_audit.md)

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
| `readiness_score` / operational readiness? | No — cohort labels only |

**Recommendation:** Freeze **PLAN-RT-F7** (docs frozen).

---

## Authority boundaries

| Surface | Authoritative? |
|---------|----------------|
| `queue_priority` | **No** — sort metadata |
| `readiness_cohort` | **No** — bulk cognition |
| `blocker_groups` rollup | **No** — explanatory grouping |
| `experiment_rollup` | **No** — warn-only |
| `lineage_warnings` | **No** — detect-only |
| F6 advisory ladder | **No** — unchanged semantics |
| SA1 handoff CLIs | **Yes** for staging writes |
| `rt_sa_import commit` | **Yes** for SA corpus lineage |

| Check | Result |
|-------|--------|
| Advisory ≠ parser contract | **Pass** |
| Advisory ≠ operational state | **Pass** |
| Queue sort ≠ CLI execution | **Pass** |
| Cohort `ready_for_commit_advisory` ≠ committed | **Pass** |

---

## RT↔SA separation

| Check | Result |
|-------|--------|
| No SA viewer changes in PLAN wave | **Pass** |
| No auto-import from queue or cohort | **Pass** |
| F6 SA1 workflow unchanged | **Pass** |
| Lineage detect does not write corpus | **Pass** |
| `capture_session ≠ SA import` preserved | **Pass** |

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
| PLAN-RT-F7 ≠ PLAT-RT-F7 | **Pass** — documented |
| PLAN-RT-F7 ≠ distributed F7 (v1 retired) | **Pass** |
| PLAN-RT-F7 extends F6 — not replacement | **Pass** |

---

## Verdict

**Pass** — PLAN-RT-F7 suitable for docs freeze. PLAT phases require per-phase governance + contamination review before implementation authorization.
