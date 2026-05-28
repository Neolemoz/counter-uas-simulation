# RT — Next Frontiers Roadmap v7

**Phase:** Advisory roadmap after PLAN-RT-C3 freeze (post-X2 plateau)  
**Prerequisite:** [rt_c3_platform_consolidation_freeze_audit.md](rt_c3_platform_consolidation_freeze_audit.md), [rt_plat_x2_p2_freeze_audit.md](rt_plat_x2_p2_freeze_audit.md), [rt_plat_v3_p2_freeze_audit.md](rt_plat_v3_p2_freeze_audit.md)  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v6.md](rt_roadmap_next_frontiers_v6.md) — v6 history preserved  
**Master plan:** [rt_c3_runtime_platform_consolidation_plan.md](../platform/rt_c3_runtime_platform_consolidation_plan.md)

This document ranks **possible** next PLAN waves after the fourth consolidation plateau (post-X2). **None are authorized** until scoped PLAN/PLAT wave + governance (+ contamination if advisory) + freeze audit per phase.

---

## 1. Vocabulary

| Label | Meaning |
|-------|---------|
| **PLAN-RT-C3** | Post-X2 consolidation checkpoint — **docs frozen** |
| **PLAT-RT-X2** | Experiment workbench v2 — **complete** (P0–P2 frozen) |
| **PLAT-RT-V3** | Visualization fidelity — **complete** (P0–P2 frozen) |
| **PLAN-RT-F8** | Post-F7 advisory maintainer expansion — **candidate** |
| **PLAN-RT-V4** | Post-V3 visualization — **candidate** |
| **PLAN-RT-X3** | Post-X2 experiment ergonomics — **candidate** |
| **Distributed multi-bridge** | **Forbidden** |

---

## 2. Completed since v6

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-X2 P0–P2 | Cohort, unified review, multi-manifest diff, packet export | **frozen — X2 complete** |
| PLAN-RT-C3 | Post-X2 platform checkpoint | **this wave** |
| PLAT-RT-V3 P0–P2 | Layer registry, overlays, workstation layout | **frozen — V3 complete** |
| PLAT-RT-F7 / M3 | Advisory + multi-session polish | **complete** |
| PLAN-RT-C2 | Post-F7 consolidation | **frozen** |

Primary RT roadmap (S/G/R/T/M/TAC/SA/V/X) and F1–F6 remain frozen from prior plateaus.

---

## 3. Candidate frontiers

### PLAN-RT-F8 — Post-F7 advisory maintainer expansion

**Description:** Deeper maintainer ergonomics on the F6/F7 stack — triage filter presets, standup template packs, extended golden fixtures for batch v2, optional corpus-preview refinements. **Still no auto-import**, no browser commit, no `--commit-all`.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med** |
| Expected user value | **Med–High** (maintainer throughput when advisory is binding) |
| Research value | **Med** |
| Contamination risk | **High** |
| Governance cost | **High** |
| Coupling (bridge/SA/handoff) | **High** — `handoff/`, batch CLIs, derive parity |

**Dependencies:** PLAT-RT-F7 complete; [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md).

**Touches:** `src/handoff/`, `batch_advisory.py`, maintainer CLIs, possibly `deriveAdvisoryState.ts`.

**Note:** No `rt_f8_*` master plan exists yet — **PLAN wave required** before any PLAT.

---

### PLAN-RT-V4 — Post-V3 runtime visualization

**Description:** Visualization cognition beyond [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md) — e.g. marker performance budget, legend density caps, multi-session marker deconfliction policy, optional lazy layer loading. **No** new authority surfaces.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low–Med** |
| Implementation complexity | **Med** |
| Expected user value | **Med** (demo/reviewer cognition) |
| Research value | **Med** |
| Contamination risk | **Low** |
| Governance cost | **Low–Med** |
| Coupling (bridge/SA/handoff) | **Low** — primarily `cesium/`, `App.tsx` integration |

**Dependencies:** PLAT-RT-V3 complete; T3/T5 Cesium stack frozen.

**Touches:** `src/cesium/`, `CesiumRuntimePanel.tsx`, `App.tsx` (integration risk).

**Note:** No `rt_v4_*` master plan exists yet — **PLAN wave required** before PLAT.

---

### PLAN-RT-X3 — Post-X2 experiment workbench

**Description:** Narrow experiment ergonomics beyond X2 — e.g. maintainer cohort CLI mirror, optional secondary manifest staging path documentation, workbench navigation shortcuts, offline manifest ref validation helpers. **No** new bridge commands without contract wave; **no** cross-manifest run pairing.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med–High** |
| Expected user value | **Med** (incremental over X2) |
| Research value | **Med–High** |
| Contamination risk | **Med** |
| Governance cost | **Med** |
| Coupling (bridge/SA/handoff) | **Med** — `src/experiment/` already 96 files |

**Dependencies:** PLAT-RT-X2 complete; F2 import guards; X2 contracts.

**Touches:** `platform/rt-sandbox-ui/src/experiment/` (largest UI subtree).

**Note:** Defer unless X2 P2 gaps are documented with maintainer evidence — subtree already grew +32 files since C2.

---

### Maintenance-only (not ranked as numbered frontier)

**UI decomposition** — `App.tsx` layout hooks, further `ExperimentWorkbenchPanel` splits. Valuable **P1 later** debt paydown per [rt_c3_technical_debt_audit_r1.md](rt_c3_technical_debt_audit_r1.md); may fold into V4 or F2-style maintenance if explicitly scoped.

---

## 4. Explicit non-frontiers

Unchanged: distributed multi-bridge, parser/topic changes, SA live hooks, auto-import, federation writes, HITL/C2, browser capture mutation, tactical redesign.

---

## 5. Ranking summary

| Rank | Frontier | Arch risk | Complexity | Contamination | Governance | Maintainer value | Coupling |
|------|----------|-----------|------------|---------------|------------|------------------|----------|
| 1 | **PLAN-RT-F8** → PLAT | Med | Med | **High** | **High** | **Med–High** (advisory throughput) | High |
| 2 | **PLAN-RT-V4** → PLAT | Low–Med | Med | **Low** | Low–Med | Med (viz/demo) | Med (`App.tsx`) |
| 3 | **PLAN-RT-X3** → PLAT | Med | Med–High | Med | Med | Med (incremental) | **High** (`experiment/`) |
| — | Distributed multi-bridge | High | High | Critical | — | **Not recommended** | — |

---

## 6. Recommended next major step

**Recommendation (advisory):** **PLAN-RT-F8** — post-F7 advisory maintainer expansion (docs-only PLAN wave first)

**Rationale:**

1. Fourth consolidation plateau (C3) closed X2 and V3 — no mandatory PLAT backlog remains on those frontiers.  
2. Maintainer advisory paths (F6/F7 + X2 packet export adjacency) are the highest **contamination-discipline** surface — normative PLAN reduces risk before further PLAT on `handoff/`.  
3. **V4** offers lower contamination but **lower urgency** after V3 complete.  
4. **X3** couples to an already large `experiment/` subtree — pursue only with narrow PLAN scope and documented gaps from X2 usage.  
5. Distributed multi-bridge remains forbidden.

**Alternate 1:** **PLAN-RT-V4** — if demo/visualization cognition is the binding constraint (mentor/reviewer UX over maintainer batch throughput).

**Alternate 2:** **PLAN-RT-X3** — if experiment compare ergonomics gaps are evidenced post-X2 (e.g. secondary manifest load workflow).

**Not authorized** by C3 freeze or this roadmap alone.

---

## 7. Wave authorization checklist

Before PLAN-RT-F8, PLAN-RT-V4, or PLAN-RT-X3 (and any subsequent PLAT):

1. Scoped plan in `docs/platform/` (`rt_f8_*`, `rt_v4_*`, `rt_x3_*`, or `rt_plat_*` implementation plans)  
2. Governance review + freeze audit per phase  
3. Contamination review when advisory UI adjacent to handoff (**required for F8**; recommended for X3)  
4. Contract compliance with frozen v1 docs  
5. Regression matrix per wave scope  

No PLAN or PLAT work is authorized by this roadmap alone.

---

## Related

- [rt_c3_platform_consolidation_review_r1.md](rt_c3_platform_consolidation_review_r1.md)
- [rt_c3_technical_debt_audit_r1.md](rt_c3_technical_debt_audit_r1.md)
- [rt_roadmap_next_frontiers_v6.md](rt_roadmap_next_frontiers_v6.md)
- [rt_plat_x2_p2_freeze_audit.md](rt_plat_x2_p2_freeze_audit.md)
