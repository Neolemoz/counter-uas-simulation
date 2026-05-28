# RT-F7 P2 — Handoff Contamination Review R1

**Phase:** PLAT-RT-F7 P2 — batch export + dry-run hardening  
**Plan:** [rt_plat_f7_p2_batch_export_helpers_plan.md](../platform/rt_plat_f7_p2_batch_export_helpers_plan.md)  
**Baseline:** [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md), [rt_plat_f7_p1_handoff_contamination_review_r1.md](rt_plat_f7_p1_handoff_contamination_review_r1.md)

---

## P2 contamination matrix

| ID | Landmine | Severity | P2 mitigation | Residual |
|----|----------|----------|---------------|----------|
| F7-CONT-05 | Grouped report → batch commit | **Critical** | No commit subcommand; export only | **Low** |
| F7-CONT-12 | Escalation past dry-run | **High** | `--no-dry-run` rejected; `max_captures`; preview write opt-in | **Low** |
| F7-CONT-01 | Queue triggers auto CLI | **Critical** | Export docs read-only | **Low** |
| F7-CONT-03 | Summary as authority | **Critical** | Banners + validation | **Low** |

---

## Deny list (P2)

Still impossible after P2 freeze:

1. `standup-export` or `grouped-export` invokes commit
2. `dry-run-review` calls `rt_sa_import commit`
3. v2 export sets `import_ready` on staging
4. Browser JSON preview writes corpus
5. `--commit-all` flag introduced

---

## Verdict

| Item | Verdict |
|------|---------|
| P2 safe to freeze? | **Pass** |
| PLAT-RT-F7 complete? | **Yes** |
| Post-F7 expansion? | **Not authorized** without new PLAN |

**Pass** — P2 obeys [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) escalation depth table.
