# RT — Next Frontiers Roadmap v6

**Phase:** Advisory roadmap after PLAN-RT-X2 freeze + PLAT-RT-V3 complete  
**Prerequisite:** [rt_x2_freeze_audit.md](rt_x2_freeze_audit.md), [rt_plat_v3_p2_freeze_audit.md](rt_plat_v3_p2_freeze_audit.md)  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v5.md](rt_roadmap_next_frontiers_v5.md) — v5 history preserved

This document ranks **possible** next steps after V3 PLAT complete and X2 PLAN freeze. **None are authorized** until scoped PLAT/PLAN wave + governance (+ contamination if advisory) + freeze audit.

---

## 1. Vocabulary

| Label | Meaning |
|-------|---------|
| **PLAN-RT-X2** | Experiment workbench v2 planning — **docs frozen** |
| **PLAT-RT-X2** | Implementation backlog P0–P2 — **not authorized** by PLAN |
| **PLAN-RT-F8** | Post-F7 advisory expansion — candidate |
| **PLAT-RT-V3** | Visualization fidelity — **complete** (P0–P2 frozen) |
| **Distributed multi-bridge** | **Forbidden** |

---

## 2. Completed since v5

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-V3 P0–P2 | Layer registry, overlays, workstation layout | **frozen** |
| PLAN-RT-X2 | Cohort, unified review, compare v2 contracts | **docs frozen** (this wave) |
| PLAN-RT-C2 | Post-F7 consolidation | **docs frozen** |
| PLAT-RT-F7 / M3 | Advisory + multi-session polish | **complete** |

---

## 3. Candidate frontiers (post-X2 PLAN)

### PLAT-RT-X2 — Experiment workbench implementation

**Description:** Execute [rt_roadmap_plat_rt_x2_v1.md](rt_roadmap_plat_rt_x2_v1.md) — cohort index, unified review panel, compare workflow v2.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med–High** |
| Expected user value | **Med–High** |
| Research value | **Med–High** |
| Contamination risk | **Med** |
| Governance cost | **Med** |

**Dependencies:** PLAN-RT-X2 frozen; PLAT-RT-X1, F1, F3, F5, F5b frozen.

**Touches:** `platform/rt-sandbox-ui/src/experiment/` (~64 files per C2 debt audit).

---

### PLAN-RT-F8 — Post-F7 advisory expansion

**Description:** Maintainer advisory ergonomics beyond F7 — queue/triage/bulk patterns not fully normativized in a dedicated PLAN wave.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med** |
| Contamination risk | **High** |
| Governance cost | **High** |
| Maintainer value | **Med** (when advisory throughput is binding) |

**Dependencies:** PLAT-RT-F7 complete; explicit contamination review before PLAN.

**Note:** No `rt_f8_*` master plan exists yet — PLAN wave required before PLAT.

---

## 4. Explicit non-frontiers

Unchanged: distributed multi-bridge, parser/topic changes, SA live hooks, auto-import, federation writes, HITL/C2, browser capture.

---

## 5. Ranking summary (post-X2 PLAN)

| Rank | Frontier | Arch risk | Complexity | Contamination | Governance | Maintainer value |
|------|----------|-----------|------------|---------------|------------|------------------|
| 1 | **PLAT-RT-X2 P0–P2** | Med | Med–High | Med | Med | **High** (experiment compare) |
| 2 | **PLAN-RT-F8** → PLAT | Med | Med | **High** | **High** | Med (advisory throughput) |
| — | Distributed multi-bridge | High | High | Critical | — | **Not recommended** |

---

## 6. Recommended next major step

**Recommendation (advisory):** **PLAT-RT-X2 P0** — cohort index store + read-only multi-manifest import

**Rationale:**

1. **PLAN-RT-X2** contracts and reviews are complete — implementation backlog is defined.  
2. V3 PLAT complete — visualization plateau closed; experiment compare is the natural successor per user priority.  
3. X2 modularizes `experiment/` intent before further panel accretion (C2 debt).  
4. **F8** remains valuable but **higher contamination** — requires dedicated PLAN + contamination audit before competing with X2 PLAT.  
5. Distributed multi-bridge remains forbidden.

**Alternate 1:** **PLAN-RT-F8** (docs) then PLAT — if maintainer advisory batch/triage is the binding constraint.

**Not authorized** by X2 PLAN freeze or this roadmap alone.

---

## 7. Wave authorization checklist

Before PLAT-RT-X2 (or PLAN-RT-F8):

1. Scoped plan in `docs/platform/` (`rt_plat_x2_p0_*`, etc.)  
2. Governance review + freeze audit per phase  
3. Contamination review when advisory UI adjacent to handoff (X2 P1, F8 PLAN)  
4. Contract compliance with X2 v1 docs (or F8 contracts when written)  
5. Regression matrix per [rt_roadmap_plat_rt_x2_v1.md](rt_roadmap_plat_rt_x2_v1.md)

No PLAT-RT-X2 or PLAN-RT-F8 work is authorized by this roadmap alone.

---

## Related

- [rt_x2_freeze_audit.md](rt_x2_freeze_audit.md)
- [rt_roadmap_plat_rt_x2_v1.md](rt_roadmap_plat_rt_x2_v1.md)
- [rt_roadmap_next_frontiers_v5.md](rt_roadmap_next_frontiers_v5.md)
- [rt_c2_technical_debt_audit_r1.md](rt_c2_technical_debt_audit_r1.md)
