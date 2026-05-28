# RT — Next Frontiers Roadmap v8

**Phase:** Advisory roadmap after PLAN-RT-F8 freeze (post-F7 advisory PLAN plateau)  
**Prerequisite:** [rt_f8_freeze_audit.md](rt_f8_freeze_audit.md), [rt_c3_platform_consolidation_freeze_audit.md](rt_c3_platform_consolidation_freeze_audit.md), [rt_plat_f7_p2_freeze_audit.md](rt_plat_f7_p2_freeze_audit.md)  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md) — v7 history preserved  
**Master plan:** [rt_f8_post_f7_advisory_maintainer_expansion_plan.md](../platform/rt_f8_post_f7_advisory_maintainer_expansion_plan.md)

This document ranks **possible** next waves after PLAN-RT-F8 freeze. **None are authorized** until scoped PLAT/PLAN wave + governance (+ contamination if advisory) + freeze audit per phase.

---

## 1. Vocabulary

| Label | Meaning |
|-------|---------|
| **PLAN-RT-F8** | Post-F7 advisory maintainer expansion — **docs frozen** |
| **PLAT-RT-F8** | F8 implementation backlog — **not authorized** by PLAN alone |
| **PLAN-RT-V4** | Post-V3 visualization — **candidate** |
| **PLAN-RT-X3** | Post-X2 experiment ergonomics — **candidate** |
| **Distributed multi-bridge** | **Forbidden** |

---

## 2. Completed since v7

| ID | Summary | Status |
|----|---------|--------|
| PLAN-RT-F8 | Maintainer workflow v2, contamination gates v2, aggregation v2 | **this wave** |
| PLAT-RT-F7 P0–P2 | Queue, triage UI, batch review v2 export | **frozen — F7 complete** |
| PLAN-RT-C3 | Post-X2 consolidation | **frozen** |
| PLAT-RT-X2 / V3 | Workbench v2 + visualization fidelity | **frozen** |

---

## 3. Candidate frontiers

### PLAT-RT-F8 — Post-F7 advisory implementation

**Description:** Implements [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md) — summary v2, filter presets, cohort v2 UI, template packs, corpus-preview refinements.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med** |
| Contamination risk | **High** |
| Governance cost | **High** |
| Coupling | **High** — `handoff/`, batch CLIs, X2 adjacency |
| Maintainer value | **Med–High** |

**Dependencies:** PLAN-RT-F8 frozen; [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md).

**Roadmap:** [rt_roadmap_plat_rt_f8_v1.md](rt_roadmap_plat_rt_f8_v1.md)

---

### PLAN-RT-V4 — Post-V3 runtime visualization

**Description:** Visualization cognition beyond [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md) — marker budget, legend density, multi-session deconfliction, lazy layer loading. **No** new authority surfaces.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low–Med** |
| Implementation complexity | **Med** |
| Contamination risk | **Low** |
| Governance cost | **Low–Med** |
| Coupling | **Med** — `cesium/`, `App.tsx` |
| Maintainer value | **Med** (demo/reviewer) |

**Note:** No `rt_v4_*` master plan exists yet — **PLAN wave required** before PLAT.

---

### PLAN-RT-X3 — Post-X2 experiment workbench

**Description:** Narrow experiment ergonomics beyond X2 — maintainer cohort CLI mirror, manifest staging documentation, navigation shortcuts, offline ref validation. **No** new bridge commands without contract wave.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med–High** |
| Contamination risk | **Med** |
| Governance cost | **Med** |
| Coupling | **High** — `src/experiment/` (~96 files) |
| Maintainer value | **Med** (incremental) |

**Note:** Pursue only with documented X2 gaps — see [rt_c3_technical_debt_audit_r1.md](rt_c3_technical_debt_audit_r1.md).

---

## 4. Explicit non-frontiers

Unchanged: distributed multi-bridge, parser/topic changes, SA live hooks, auto-import, federation writes, HITL/C2, browser capture mutation, tactical redesign.

---

## 5. Ranking summary

| Rank | Frontier | Arch risk | Complexity | Contamination | Governance | Maintainer value | Coupling |
|------|----------|-----------|------------|---------------|------------|------------------|----------|
| 1 | **PLAT-RT-F8** P0→P2 | Med | Med | **High** | **High** | **Med–High** | **High** |
| 2 | **PLAN-RT-V4** → PLAT | Low–Med | Med | **Low** | Low–Med | Med | Med |
| 3 | **PLAN-RT-X3** → PLAT | Med | Med–High | Med | Med | Med | **High** |
| — | Distributed multi-bridge | High | High | Critical | — | **Not recommended** | — |

---

## 6. Recommended next major step

**Recommendation (advisory):** **PLAT-RT-F8 P0** — implements PLAN-RT-F8 contracts when maintainer advisory throughput is the binding constraint.

**Rationale:**

1. PLAN-RT-F8 closes the post-F7 advisory documentation gap (presets, v2 aggregation, F8-CONT matrix).
2. PLAT-RT-F7 delivered queue/triage/export — F8 P0 is the natural implementation follow-on with highest maintainer leverage on `handoff/`.
3. **V4** remains lower contamination but lower urgency after V3 complete.
4. **X3** couples to a large `experiment/` subtree — defer without maintainer evidence.

**Alternate 1:** **PLAN-RT-V4** (docs) then PLAT — if demo/visualization cognition is binding.

**Alternate 2:** **PLAN-RT-X3** (docs) then PLAT — if X2 compare ergonomics gaps are evidenced.

**Not authorized** by F8 freeze or this roadmap alone.

---

## 7. Wave authorization checklist

Before PLAT-RT-F8, PLAN-RT-V4, or PLAN-RT-X3:

1. Scoped plan in `docs/platform/` (`rt_plat_f8_*`, `rt_v4_*`, `rt_x3_*`)
2. Governance review + freeze audit per phase
3. Contamination review when advisory (**required for PLAT-RT-F8**; recommended for X3)
4. Contract compliance with frozen v1/v2 docs
5. Regression matrix per wave scope

No PLAT work is authorized by this roadmap alone.

---

## Related

- [rt_f8_freeze_audit.md](rt_f8_freeze_audit.md)
- [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md)
- [rt_c3_platform_consolidation_review_r1.md](rt_c3_platform_consolidation_review_r1.md)
