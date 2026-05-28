# RT-F8 P0 — Handoff Contamination Review R1

**Phase:** PLAT-RT-F8 P0  
**Contract:** [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md)  
**PLAN review:** [rt_f8_handoff_contamination_review_r1.md](rt_f8_handoff_contamination_review_r1.md)

---

## F8-CONT re-check (P0 scope)

| ID | Mitigation in P0 | Residual |
|----|------------------|----------|
| F8-CONT-01 | `apply_filter_preset` filters only; UI banner; no CLI from preset | **Low** |
| F8-CONT-02 | `readiness_cohort_v2` labels + chips; not `readiness_score` | **Low** |
| F8-CONT-03 | v2 summary read-only; no export event emission | **Low** |
| F8-CONT-04 | `experiment_handoff_rollup` warn-only note field | **Low** |
| F8-CONT-05 | `render_template_pack` — no `next_cli` in minimal/daily packs | **Low** |
| F8-CONT-06 | Optional cohort index read paths only; no packet generation | **Low** |
| F8-CONT-07 | Rollups do not write `fixtures/sa_r0/` | **Low** |
| F8-CONT-08 | `apply_focus_set` intersects IDs; no cross-session authority merge | **Low** |
| F8-CONT-09 | No M3 poll merge into advisory queue | **Low** |
| F8-CONT-10 | v2 labels map to F6 states; SA1 CLIs unchanged | **Low** |
| F8-CONT-11 | Corpus-preview unchanged in P0 | **Low** |
| F8-CONT-12 | Template render depth 3 packs only in P0 | **Med** — P2 audit |
| F8-CONT-13 | Cohort index adjacency warn-only | **Low** |
| F8-CONT-14 | Exemplars in rollups capped per contract | **Low** |

F7-CONT-01..12 remain in force.

---

## Deny list (unchanged)

No preset-triggered subprocess, no batch corpus write, no SA viewer hooks, no federation publish from v2 rollup, no `--commit-all`.

---

## Verdict

**Pass** — PLAT-RT-F8 P0 may freeze. P1 requires UI copy re-check on triage integration; P2 requires F8-CONT-12 escalation audit.
