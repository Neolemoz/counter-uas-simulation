# RT-F8 — Handoff Contamination Review R1

**Phase:** PLAN-RT-F8 — advisory maintainer expansion contamination (docs only)  
**Plan:** [rt_f8_post_f7_advisory_maintainer_expansion_plan.md](../platform/rt_f8_post_f7_advisory_maintainer_expansion_plan.md)  
**Contract:** [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md)  
**F7 baseline:** [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md), [rt_plat_f7_p2_handoff_contamination_review_r1.md](rt_plat_f7_p2_handoff_contamination_review_r1.md)

F8 is ranked **High contamination risk** in [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md) because presets, template packs, and X2-adjacent rollups can approach auto-import if not careful. This review scores F8 vectors and PLAN mitigations.

---

## 1. F8 contamination landmine matrix

| ID | Landmine | Severity | PLAN mitigation | Residual (PLAT) |
|----|----------|----------|-----------------|-----------------|
| F8-CONT-01 | Preset triggers batch approve | **Critical** | Preset filter-only | **Low** if PLAT obeys |
| F8-CONT-02 | Cohort v2 implies commit | **High** | Labels ≠ score; banner | **Med** — UX |
| F8-CONT-03 | Summary v2 authority | **Critical** | Banner; no export events | **Low** |
| F8-CONT-04 | Rollup overrides ladder | **High** | F6 precedence | **Low** |
| F8-CONT-05 | Template auto-runs `next_cli` | **Critical** | Render-only shells | **Med** at P2 |
| F8-CONT-06 | X2 packet implies commit | **High** | Path metadata only | **Med** — combined UX |
| F8-CONT-07 | Multi-capture summary writes corpus | **Critical** | Read-only rollup | **Low** |
| F8-CONT-08 | Focus set merges authority | **High** | Per-capture triage | **Low** |
| F8-CONT-09 | M3 poll merges queue | **High** | Poll ≠ advisory authority | **Low** |
| F8-CONT-10 | Cohort v2 skips approve | **High** | SA1 unchanged | **Med** |
| F8-CONT-11 | Corpus-preview write | **Critical** | Read-only preview | **Low** |
| F8-CONT-12 | Template exceeds dry-run depth | **High** | Escalation v2 §5 | **Med** at P2 |
| F8-CONT-13 | X2 cohort as import gate | **High** | Warn-only | **Med** |
| F8-CONT-14 | Exemplar IDs imply promotion | **Med** | Cap + full list in captures | **Low** |

---

## 2. Authority escalation paths (deny list)

Must remain **impossible** after PLAN freeze and through PLAT P0–P2:

1. Preset application invokes `next_cli` subprocess
2. Summary v2 writes `fixtures/sa_r0/`
3. Rollup emits `handoff_import_committed`
4. Experiment or X2 cohort `complete` triggers `rt_sa_import commit`
5. Cohort v2 label skips `rt_capture_approve.py`
6. Template pack chains pipeline `--execute`
7. Batch helper generates X2 review packet as authority artifact
8. Multi-capture failure batch-commits siblings

---

## 3. F7 → F8 delta

| F7 control | F8 extension |
|------------|--------------|
| Queue sort | + filter presets |
| Cohort labels | + cohort v2 + stale warn |
| batch_review_v2 standup | + template packs |
| experiment_rollup | + experiment_handoff_rollup + X2 paths |
| F7-CONT-12 escalation | F8-CONT-12 template depth |
| — | F8-CONT-06 X2 packet adjacency |

---

## 4. X2 + F7 batch v2 interaction

| Scenario | Risk | Mitigation |
|----------|------|------------|
| Maintainer exports review packet then runs `report --schema v2` | **Med** — feels like one workflow | Separate actions; banners on both |
| Rollup lists `review_packet_paths` | **Med** — path looks authoritative | BND-07; warn footer |
| Cohort index `status: complete` + `import_advisory_only` preset | **High** | F8-CONT-13; preset is filter only |

---

## 5. Phased contamination tolerance

| PLAT phase | Allowed depth | Required gates |
|------------|---------------|----------------|
| **P0** | 0–2; v2 schema + presets | This review closed; f8 fixtures |
| **P1** | 0–2 UI mirror | Read-only; no action buttons; X2 strip copy |
| **P2** | 0–3 templates + preview | **New** P2 contamination re-audit |

**Recommendation:** Do not authorize **PLAT-RT-F8** until PLAN-RT-F8 frozen. P0 after PLAN freeze; P2 requires re-audit.

---

## 6. Verdict

| Item | Verdict |
|------|---------|
| PLAN docs safe to freeze? | **Pass-with-conditions** |
| PLAT P0? | **Advisory authorize** after PLAN freeze — read-only extensions |
| PLAT P1? | **Advisory authorize** after P0 — UX copy discipline |
| PLAT P2? | **Conditional** — separate re-audit |

**Pass-with-conditions** — PLAN-RT-F8 may freeze provided PLAT obeys [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md) §6 deny list.
