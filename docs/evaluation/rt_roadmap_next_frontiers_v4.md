# RT — Next Frontiers Roadmap v4

**Phase:** Advisory roadmap after PLAN-RT-C2 freeze  
**Prerequisite:** PLAN-RT-C2 frozen; PLAT-RT-F7 complete; PLAT-RT-M3 complete  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v3.md](rt_roadmap_next_frontiers_v3.md) — v2/v3 history preserved  
**Master plan:** [rt_c2_runtime_platform_consolidation_plan.md](../platform/rt_c2_runtime_platform_consolidation_plan.md)

This document ranks **possible** next PLAN waves after the post-F7 consolidation plateau. **None are authorized** until scoped plan + governance (+ contamination if advisory) + freeze audit per phase.

---

## 1. Vocabulary

| Label | Meaning |
|-------|---------|
| **PLAN-RT-C2** | Post-F7 consolidation review — **docs frozen** |
| **PLAN-RT-F8** | *Candidate* — post-F7 advisory maintainer expansion |
| **PLAN-RT-V3** | *Candidate* — runtime visualization fidelity beyond V1/V2 |
| **PLAN-RT-X2** | *Candidate* — experiment workbench v2 beyond X1/F5 |
| **Distributed multi-bridge** | **Forbidden** — not a numbered frontier |

---

## 2. Completed since v3

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-F7 P0–P2 | Advisory queue, triage UI, batch v2 export | **frozen — F7 complete** |
| PLAN-RT-C2 | Post-F7 platform consolidation review | **this wave** |
| PLAT-RT-M3 P0–P2 | Session inspect, poll UX, tab reorder | **frozen — M3 complete** |

Primary RT roadmap (S/G/R/T/M/TAC/SA/V/X) and F1–F6 remain frozen from prior plateaus.

---

## 3. Candidate frontiers

### F8 — Post-F7 advisory maintainer expansion (PLAN-RT-F8)

**Description:** Deeper maintainer ergonomics on the F6/F7 stack — optional corpus-preview refinements, standup template packs, triage filter presets, extended golden fixtures for batch v2. **Still no auto-import**, no browser commit, no `--commit-all`.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med** |
| Expected user value | **Med–High** (maintainer throughput) |
| Research value | **Med** |
| Contamination risk | **High** |
| Governance cost | **High** — full F6/F7 contamination review required per PLAT phase |

**Dependencies:** PLAT-RT-F7 complete; [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md).

**Touches:** `handoff/`, `batch_advisory.py`, maintainer CLIs, possibly `deriveAdvisoryState.ts`.

---

### V3 — Runtime visualization fidelity (PLAN-RT-V3)

**Description:** Cesium/marker/bounds/camera cognition polish beyond [rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md) and [rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md) — readability, legend density, multi-session chrome, optional performance budget for marker layers. **No** new authority surfaces.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low–Med** |
| Implementation complexity | **Med** |
| Expected user value | **Med–High** (reviewer/demo cognition) |
| Research value | **Med** |
| Contamination risk | **Low** |
| Governance cost | **Low–Med** — standard governance review; no SA import adjacency |

**Dependencies:** PLAT-RT-V1, PLAT-RT-V2 frozen; T3/T5 Cesium stack.

**Touches:** `cesium/`, `CesiumRuntimePanel.tsx`, `App.tsx` (integration risk).

---

### X2 — Experiment workbench v2 (PLAN-RT-X2)

**Description:** Compare/matrix/annex ergonomics beyond X1 and F5 — saved compare layouts, annex diff highlights, workbench navigation shortcuts, optional batch experiment review CLI. **No** new bridge commands without contract wave.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med–High** |
| Expected user value | **Med–High** (comparative sandbox research) |
| Research value | **High** |
| Contamination risk | **Med** — experiment import guards must hold |
| Governance cost | **Med** — F2 import guards + experiment banners |

**Dependencies:** PLAT-RT-X1, PLAT-RT-F5 P0–P2 frozen.

**Touches:** `src/experiment/` (64 files — largest subtree).

---

### Maintenance-only (defer listing as ranked frontier)

**UI decomposition** (`App.tsx` layout hooks) — valuable **P1 later** debt paydown; not numbered as PLAN-RT-* unless scoped explicitly. May be folded into V3 or F2-style maintenance if chosen.

---

## 4. Explicit non-frontiers

Unchanged from v2/v3:

| Theme | Rationale |
|-------|-----------|
| **Distributed multi-bridge** | Forbidden — local prototype only |
| Parser/topic/schema changes | Platform boundary |
| Operational HITL/C2 / readiness scoring | Forbidden lexicon |
| Browser `capture_session` | Isolation tests |
| SA viewer live RT hooks | Beyond SA3 read-only |
| Federation authority from RT | Export boundary |
| PX4/MAVLink/hardware | Out of scope |
| Post-F7 advisory without new PLAN | F7 P2 stop line |

---

## 5. Ranking summary

| Rank | Frontier | Arch risk | Complexity | User value | Contamination | Gov cost | Notes |
|------|----------|-----------|------------|------------|---------------|----------|-------|
| 1 | **V3 Visualization fidelity** | Low–Med | Med | Med–High | Low | Low–Med | Best post-C2 fit — polish without authority |
| 2 | **X2 Experiment workbench v2** | Med | Med–High | Med–High | Med | Med | High value; touches largest UI subtree |
| 3 | **F8 Advisory expansion** | Med | Med | Med–High | **High** | **High** | Only if maintainer advisory is binding constraint |
| — | Distributed multi-bridge | High | High | Med | Critical | — | **Not recommended** |
| — | UI decomposition only | Low | Med | Low–Med | Low | Low | Defer or bundle with V3 |

---

## 6. Recommended next major frontier

**Recommendation (advisory):** **PLAN-RT-V3** — runtime visualization fidelity

**Rationale:**

1. **PLAN-RT-C2** confirms consolidation-complete post-F7 with **no P0 debt**.  
2. **V3** improves mentor/demo cognition with **lowest contamination risk** among ranked candidates.  
3. **F8** immediately after F7 increases advisory surface area and governance cost — prefer only if maintainer throughput is the proven bottleneck.  
4. **X2** delivers high research value but couples to `experiment/` concentration ([rt_c2_technical_debt_audit_r1.md](rt_c2_technical_debt_audit_r1.md) P1 items).  
5. **App.tsx** decomposition may be partially addressed during V3 Cesium integration work.  
6. Distributed runtime remains out of scope.

**Alternate 1:** **PLAN-RT-X2** if comparative experiment workflows are the binding constraint in field use.

**Alternate 2:** **PLAN-RT-F8** if maintainer advisory batch/triage throughput is the binding constraint — requires explicit contamination review before any PLAT phase.

**Not recommended:** Distributed multi-bridge under any numeric frontier ID.

**Not authorized** by C2 freeze.

---

## 7. Wave authorization checklist

Before any PLAT wave for F8, V3, or X2:

1. Scoped plan in `docs/platform/` (`rt_v3_*`, `rt_x2_*`, `rt_f8_*`, or `rt_plat_*` implementation plans)  
2. Contract/annex in `docs/evaluation/` if maintainer-visible behavior changes  
3. Governance review (+ **contamination review** if advisory/F8)  
4. Freeze audit + freeze registry row per PLAT phase  
5. Regression: `lint_rt_runtime_subcommands`, advisory pytest (if F8), bridge pytest, `tier0-rt-ui` (+ frontier-specific tests)

---

## 8. Stop line

**PLAN-RT-C2** freeze means: stop before PLAT implementation not meeting §7 checklist.

No F8, V3, or X2 work is authorized by this roadmap alone.
