# RT-F7 P1 — Handoff Contamination Review R1

**Phase:** PLAT-RT-F7 P1 — triage queue UI  
**Plan:** [rt_plat_f7_p1_advisory_triage_queue_plan.md](../platform/rt_plat_f7_p1_advisory_triage_queue_plan.md)  
**Baseline:** [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md), [rt_plat_f7_p0_handoff_contamination_review_r1.md](rt_plat_f7_p0_handoff_contamination_review_r1.md)

---

## P1 contamination matrix

| ID | Landmine | Severity | P1 mitigation | Residual |
|----|----------|----------|---------------|----------|
| F7-CONT-02 | Cohort implies commit | **High** | `ready_for_commit_advisory` + banner; no commit button | **Low** |
| F7-CONT-06 | Lineage warn auto-blocks | **Med** | Warn list only in triage rows | **Low** |
| F7-CONT-07 | Cross-session merged queue | **High** | Active `session_id` rows only | **Low** |
| F7-CONT-01 | Queue triggers CLI | **Critical** | No action buttons; select → checklist only | **Low** |
| F7-CONT-04 | Experiment rollup overrides | **High** | Warn-only header + `experiment_warn` group | **Low** |

---

## Deny list (P1 UI)

Must remain impossible after P1 freeze:

1. Triage row or copy button invokes `rt_sa_import commit`
2. Triage sort reorders staging or approval state
3. Grouped blocker strip writes export events
4. Experiment rollup promotes captures to `import_ready`
5. Cross-session single commit queue in browser

---

## Verdict

| Item | Verdict |
|------|---------|
| P1 safe to freeze? | **Pass** |
| P2 authorized? | **No** — separate P2 contamination re-audit required |

**Pass** — PLAT-RT-F7 P1 obeys [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) §6 for read-only triage UX.
