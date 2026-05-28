# RT — Next Frontiers Roadmap v5

**Phase:** Advisory roadmap after PLAN-RT-V3 freeze  
**Prerequisite:** PLAN-RT-V3 frozen; PLAN-RT-C2 frozen; PLAT-RT-F7 + M3 complete  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v4.md](rt_roadmap_next_frontiers_v4.md) — v2–v4 history preserved  
**Master plan:** [rt_v3_runtime_visualization_fidelity_plan.md](../platform/rt_v3_runtime_visualization_fidelity_plan.md)

This document ranks **possible** next steps after V3 planning freeze. **None are authorized** until scoped PLAT/PLAN wave + governance (+ contamination if advisory) + freeze audit.

---

## 1. Vocabulary

| Label | Meaning |
|-------|---------|
| **PLAN-RT-V3** | Visualization fidelity planning — **docs frozen** |
| **PLAT-RT-V3** | Implementation backlog P0–P2 — **not authorized** by PLAN |
| **PLAN-RT-X2** | Experiment workbench v2 — candidate |
| **PLAN-RT-F8** | Post-F7 advisory expansion — candidate |
| **Distributed multi-bridge** | **Forbidden** |

---

## 2. Completed since v4

| ID | Summary | Status |
|----|---------|--------|
| PLAN-RT-V3 | Runtime visualization fidelity contracts + reviews | **docs frozen** (this wave) |
| PLAN-RT-C2 | Post-F7 consolidation | **docs frozen** |
| PLAT-RT-F7 / M3 | Advisory + multi-session polish | **complete** |

---

## 3. Candidate frontiers (post-V3 PLAN)

### PLAT-RT-V3 — Visualization implementation

**Description:** Execute [rt_roadmap_plat_rt_v3_v1.md](rt_roadmap_plat_rt_v3_v1.md) — registry, visibility overlays, workstation layout.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low–Med** |
| Implementation complexity | **Med** |
| Expected user value | **Med–High** |
| Research value | **Med** |
| Contamination risk | **Low** |
| Governance cost | **Low–Med** |

**Dependencies:** PLAN-RT-V3 frozen; PLAT-RT-V1/V2/F4 frozen.

---

### X2 — Experiment workbench v2 (PLAN-RT-X2)

**Description:** Compare/matrix/annex ergonomics beyond X1/F5 — unchanged from v4.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med–High** |
| Contamination risk | **Med** |
| Governance cost | **Med** |

**Touches:** `src/experiment/` (64 files).

---

### F8 — Post-F7 advisory expansion (PLAN-RT-F8)

**Description:** Maintainer advisory ergonomics beyond F7 — unchanged from v4.

| Dimension | Score |
|-----------|-------|
| Contamination risk | **High** |
| Governance cost | **High** |

---

## 4. Explicit non-frontiers

Unchanged from v4: distributed multi-bridge, parser/topic changes, SA live hooks, auto-import, federation writes, HITL/C2, browser capture.

---

## 5. Ranking summary (post-V3 PLAN)

| Rank | Frontier | Arch risk | Complexity | Contamination | Notes |
|------|----------|-----------|------------|---------------|-------|
| 1 | **PLAT-RT-V3 P0–P2** | Low–Med | Med | Low | Natural successor to frozen PLAN |
| 2 | **PLAN-RT-X2** | Med | Med–High | Med | If experiment compare is binding constraint |
| 3 | **PLAN-RT-F8** | Med | Med | **High** | Maintainer advisory only |
| — | Distributed multi-bridge | High | High | Critical | **Not recommended** |

---

## 6. Recommended next major step

**Recommendation (advisory):** **PLAT-RT-V3 P0** — visual layer registry + contract tests

**Rationale:**

1. **PLAN-RT-V3** contracts and realism review are complete — implementation backlog is defined.  
2. C2 ranked V3 first for **low contamination** and mentor/demo value.  
3. P0 de-risks P1 overlays via registry before new Cesium geometry.  
4. **X2** remains valuable but touches largest UI subtree (`experiment/`).  
5. **F8** should not precede V3 PLAT without explicit maintainer bottleneck evidence.

**Alternate 1:** **PLAN-RT-X2** (docs) then PLAT — if comparative experiment workflows are the binding constraint.

**Alternate 2:** **PLAN-RT-F8** — only with contamination review budget.

**Not authorized** by V3 PLAN freeze.

---

## 7. Wave authorization checklist

Before PLAT-RT-V3 (or PLAN-RT-X2 / F8):

1. Scoped plan in `docs/platform/` (`rt_plat_v3_p0_*`, etc.)  
2. Contract compliance with V3 v1 docs (or X2/F8 contracts when written)  
3. Governance review per phase (+ contamination review for F8)  
4. Freeze audit + freeze registry row per PLAT phase  
5. Regression: `lint_rt_runtime_subcommands`, bridge pytest, `tier0-rt-ui`  

---

## 8. Stop line

**PLAN-RT-V3** freeze means: stop before PLAT implementation not meeting §7 checklist.

No PLAT-RT-V3, X2, or F8 work is authorized by this roadmap alone.
