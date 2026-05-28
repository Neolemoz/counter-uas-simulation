# RT-X2 P1 — Handoff Contamination Review R1

**Phase:** PLAT-RT-X2 P1  
**Baseline:** [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md), [rt_plat_x2_p0_governance_review_r1.md](rt_plat_x2_p0_governance_review_r1.md)

## P1 contamination matrix

| ID | Landmine | P1 mitigation | Residual |
|----|----------|---------------|----------|
| X2-CONT-01 | Review lane triggers import | No commit/import buttons in lane | **Low** |
| X2-CONT-02 | Packet implies SA import | Normative banner + clipboard only | **Low** |
| X2-CONT-03 | F7 cohort confused with experiment cohort | Existing copy + navigator hint | **Low** |
| X2-CONT-04 | Handoff hint auto-import | Display-only; no new actions | **Low** |

## Deny list (P1 UI)

1. Review lane does not invoke `rt_sa_import` or `capture_session`
2. Packet copy does not write SA corpus
3. Compare mode does not cross-manifest pair `run_id`
4. No subprocess batch from new dock buttons

## Verdict

**Pass** — PLAT-RT-X2 P1 may freeze.
