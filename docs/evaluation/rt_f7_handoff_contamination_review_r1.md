# RT-F7 — Handoff Contamination Review R1

**Phase:** PLAN-RT-F7 — advisory expansion contamination (docs only)  
**Plan:** [rt_f7_post_f6_advisory_expansion_plan.md](../platform/rt_f7_post_f6_advisory_expansion_plan.md)  
**Contract:** [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md)  
**F6 baseline:** [rt_f6_handoff_contamination_review_r1.md](rt_f6_handoff_contamination_review_r1.md), [rt_f6_handoff_contamination_review_p2_r1.md](rt_f6_handoff_contamination_review_p2_r1.md)

F7 is ranked **High contamination risk** in [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md) because bulk aggregation and cohort language can approach auto-import if not careful. This review scores F7 vectors and PLAN mitigations.

---

## 1. F7 contamination landmine matrix

| ID | Landmine | Severity | PLAN mitigation | Residual (PLAT) |
|----|----------|----------|-----------------|-----------------|
| F7-CONT-01 | Queue triggers auto CLI | **Critical** | Sort-only contract | **Low** if PLAT obeys |
| F7-CONT-02 | Cohort implies commit | **High** | `ready_for_commit_advisory` + banner | **Med** — UX |
| F7-CONT-03 | Summary as authority | **Critical** | No export events from rollup | **Low** |
| F7-CONT-04 | Experiment rollup overrides ladder | **High** | F6 precedence | **Low** |
| F7-CONT-05 | `--commit-all` on grouped report | **Critical** | Forbidden §6 | **Med** at P2 |
| F7-CONT-06 | Lineage warn auto-blocks commit | **Med** | Warn-only detect | **Med** |
| F7-CONT-07 | Cross-session merged queue | **High** | Per-capture triage | **Low** |
| F7-CONT-08 | SA viewer live hook | **High** | Out of scope | **Low** |
| F7-CONT-09 | Federation from batch | **Critical** | Forbidden | **Low** |
| F7-CONT-10 | Unattended annotate-review batch | **High** | Gated — F6 P2 unchanged | **Low** |
| F7-CONT-11 | Corpus-preview write | **Critical** | Read-only preview | **Low** |
| F7-CONT-12 | Escalation past dry-run | **High** | Depth table §5 | **Med** at P2 |

---

## 2. Authority escalation paths (deny list)

Must remain **impossible** after PLAN freeze and through PLAT P0–P2:

1. Queue sort invokes `next_cli` subprocess
2. Batch summary writes `fixtures/sa_r0/`
3. Rollup emits `handoff_import_committed`
4. Experiment `eligible` triggers `rt_sa_import commit`
5. Cohort label skips `rt_capture_approve.py`
6. Multi-capture failure batch-commits siblings

---

## 3. F6 → F7 delta

| F6 control | F7 extension |
|------------|--------------|
| Per-capture ladder | Unchanged |
| P2 scan/report | Extended summary schema |
| F6-CONT-04 batch commit | F7-CONT-05 grouped report |
| Naming disambiguation | + cohort copy rules |
| P2 dry-run | F7-CONT-12 escalation depth |

---

## 4. Phased contamination tolerance

| PLAT phase | Allowed depth | Required gates |
|------------|---------------|----------------|
| **P0** | Schema + CLI sort/group + lineage warn field | This review closed; golden fixtures |
| **P1** | Read-only triage UI | `tier0-rt-ui`; no action buttons |
| **P2** | Stand-up export + dry-run hardening | **New** P2 contamination re-audit |

**Recommendation:** Authorize **PLAT-RT-F7 P0** after PLAN freeze. Hold P2 until P0/P1 frozen.

---

## 5. Verdict

| Item | Verdict |
|------|---------|
| PLAN docs safe to freeze? | **Pass-with-conditions** |
| PLAT P0? | **Authorize** after PLAN freeze — read-only extensions |
| PLAT P1? | **Authorize** after P0 — UX cohort copy |
| PLAT P2? | **Conditional** — separate re-audit |

**Pass-with-conditions** — PLAN-RT-F7 may freeze provided PLAT obeys [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) §6 deny list.
