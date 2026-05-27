# RT — Next Frontiers Roadmap v2

**Phase:** PLAN-RT-C1 — advisory roadmap after post-F6 consolidation plateau  
**Prerequisite:** PLAN-RT-C1 frozen  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) — v1 history preserved; v1 “F7 = distributed” label **retired** here  
**Master review:** [rt_c1_platform_consolidation_review_r1.md](rt_c1_platform_consolidation_review_r1.md)  
**Technical debt:** [rt_c1_technical_debt_audit_r1.md](rt_c1_technical_debt_audit_r1.md)

This document ranks **possible** next frontiers after the F6 plateau. **None are authorized** until a scoped plan + governance review + freeze audit completes.

Scoring: **Low / Med / High** per dimension.

---

## 1. Vocabulary reset (vs v1)

| Label | v2 meaning |
|-------|------------|
| **PLAN-RT-M3** | Local multi-session **polish** on single bridge ([rt_roadmap_m1_m2_v1.md](rt_roadmap_m1_m2_v1.md) § PLAT-RT-M3) |
| **PLAN-RT-F7** | Post-F6 **advisory expansion** — maintainer ergonomics, checklist depth, contamination gates |
| **Distributed multi-bridge** | **Not a numbered frontier** — remains forbidden |

---

## 2. Completed platform waves (reference)

| ID | Summary | Status |
|----|---------|--------|
| Primary RT roadmap | S1–S6, G1–G6, R1–R3d, T1–T5, M2, TAC1–5, SA1–3, V1–2, X1 | **Frozen** |
| F1 | Experiment analytics + sweep catalog | **PLAT frozen** |
| F2 | Platform hardening | **PLAT frozen** |
| F3 | Annex continuity review UI | **PLAT frozen** |
| F4 | Runtime realism expansion | **PLAT frozen** |
| F5 | Advanced experiments P0–P2 | **PLAT frozen** |
| F5b | Fidelity coupling P0–P2 | **PLAT frozen** |
| F6 | SA workflow advisory P0–P2 | **PLAT frozen** |
| PLAN-RT-R2 | First maturity plateau | **docs frozen** |
| PLAN-RT-C1 | Second consolidation plateau | **docs frozen** (this wave) |

---

## 3. Candidate frontiers

### M3 — Multi-session optional polish (PLAN-RT-M3)

**Description:** Background pull optimization for non-active sessions, maintainer `rt_session_inspect` health CLI, session tab UX polish (naming, reorder). **Single bridge, cap=3** — not distributed.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low** |
| Implementation complexity | **Low–Med** |
| Expected user value | **Med** |
| Research value | **Low–Med** |
| Contamination risk | **Low** |

**Dependencies:** PLAT-RT-M2 frozen.

**Governance notes:** No new bridge commands without contract wave; no SA viewer changes.

---

### F7 — Post-F6 advisory expansion (PLAN-RT-F7)

**Description:** Deeper advisory checklist ergonomics, maintainer batch report hardening, optional corpus-preview UX refinements, extended golden fixtures — **still no auto-import**, no browser commit.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med** |
| Implementation complexity | **Med** |
| Expected user value | **Med–High** |
| Research value | **Med** |
| Contamination risk | **High** |

**Dependencies:** PLAT-RT-F6 P0–P2 frozen; fresh contamination review required.

**Governance notes:** Must repeat F6 landmine matrix; default dry-run; no `--commit-all`.

---

## 4. Explicit non-frontiers

| Theme | Rationale |
|-------|-----------|
| **Distributed multi-bridge** | Conflicts with local prototype; forbidden in AGENTS.md |
| Parser/topic/schema changes | Platform boundary |
| Operational HITL/C2 / readiness scoring | Forbidden lexicon |
| Browser `capture_session` | Isolation tests |
| SA viewer live RT hooks | Beyond SA3 read-only |
| Federation authority from RT | Export boundary |
| PX4/MAVLink/hardware | Out of scope |

---

## 5. Ranking summary

| Rank | Frontier | Arch risk | Complexity | User value | Contamination | Notes |
|------|----------|-----------|------------|------------|---------------|-------|
| 1 | **M3 Multi-session polish** | Low | Low–Med | Med | Low | Best fit post-consolidation |
| 2 | F7 Advisory expansion | Med | Med | Med–High | High | Requires F6-style contamination audit |
| — | Distributed multi-bridge | High | High | Med | Critical | **Not recommended** |

---

## 6. Recommended next major frontier

**Recommendation (advisory):** **PLAN-RT-M3** — local multi-session polish

**Rationale:**

1. **PLAN-RT-C1** confirms platform consolidation-complete; no P0 debt blockers.  
2. M3 addresses **deferred M1/M2 ergonomics** (background poll, inspect CLI) without new authority surfaces.  
3. **Lower contamination risk** than F7 advisory expansion immediately after F6 P2 batch helpers.  
4. `App.tsx` / experiment concentration benefit from **operational polish** before more advisory UI.  
5. Distributed runtime remains out of scope.

**Alternate:** **PLAN-RT-F7** if maintainer advisory workflows prove the binding constraint in field use — only after explicit PLAN + contamination review.

**Not recommended:** Distributed multi-bridge under any numeric frontier ID.

---

## 7. Wave authorization checklist

Before implementation:

1. Scoped plan doc in `docs/platform/`  
2. Contract or contract annex in `docs/evaluation/` if behavior visible to maintainers  
3. Governance review + isolation audit if UI or bridge touched  
4. Contamination review if advisory or import-adjacent (F7)  
5. Freeze audit + freeze registry row  
6. Regression: bridge pytest + tier0-rt-ui (+ frontier-specific tests)

---

## 8. Stop line

This roadmap is **advisory**. **PLAN-RT-C1** freeze means: stop before any implementation wave not meeting §7 checklist.
