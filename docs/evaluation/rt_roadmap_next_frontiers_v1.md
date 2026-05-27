# RT — Next Frontiers Roadmap v1

**Phase:** PLAN-RT-R2 — advisory roadmap after platform maturity plateau  
**Prerequisite:** PLAN-RT-R2 frozen  
**Master review:** [rt_r2_platform_maturity_review_r1.md](rt_r2_platform_maturity_review_r1.md)  
**Technical debt:** [rt_r2_technical_debt_audit_r1.md](rt_r2_technical_debt_audit_r1.md)

This document ranks **possible** next frontiers. **None are authorized** until a scoped plan + governance review + freeze audit completes.

Scoring: **Low / Med / High** per dimension.

---

## 1. Candidate frontiers

### F1 — RT experiment analytics & template sweep catalog

**Description:** Extend X1 with read-only catalog of experiment manifests, template sweep indexing, comparative charts (explanatory), maintainer doc for batch matrix runs — still no browser capture, no SA auto-import.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low** |
| Implementation complexity | **Med** |
| Expected user value | **High** |
| Research value | **High** |

**Dependencies:** PLAT-RT-X1 frozen; optional thin indexer CLI under `scripts/rt/`.

**Governance notes:** Must preserve explanatory-only compare; no readiness scoring.

---

### F2 — Platform hardening (docs, CI, optional live-stack smoke)

**Description:** Consolidated RT maintainer guide, optional non-default Gazebo smoke job, `App.tsx` decomposition maintenance wave, doc index generation — no feature expansion.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low** |
| Implementation complexity | **Low–Med** |
| Expected user value | **Med** |
| Research value | **Low–Med** |

**Dependencies:** None beyond R2 freeze.

---

### F3 — Deeper tactical annex timeline in RT UI (not SA)

**Description:** X1 stop-line item — richer in-RT timeline visualization from `tactical_annex.json` counts/segments without SA scrubber parity.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low** |
| Implementation complexity | **Med** |
| Expected user value | **Med** |
| Research value | **Med** |

**Governance notes:** Must not become replay authority; stay RT-local.

---

### F4 — Runtime realism expansion (RT UI visualization)

**Description:** Extend PLAT-RT-V2 fictional terrain — contours, elevation bands, vegetation/occlusion landmarks, explanatory LOS/visibility cognition, Cesium polish — RT UI only; no Gazebo or bridge changes.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low** |
| Implementation complexity | **Med** |
| Expected user value | **Med** |
| Research value | **Med** |

**Governance notes:** V2 fictional boundary must not blur; contours default off; heuristic labeling mandatory. **PLAN-RT-F4 / PLAT-RT-F4** authorized via dedicated wave audit.

---

### F5 — Advanced runtime experiments (experiment model — PLAN frozen)

**Description:** Governed experiment taxonomy on PLAT-RT-X1/F1/F3/F4: five experiment classes (`terrain_comparison`, `sensor_range_comparison`, `tactical_mode_comparison`, `repeatability_sweep`, `parameter_matrix`), `rt_experiment_spec_v1` compile to batch, extended metrics report, maintainer workflow, UI planning. **No** bridge changes, **no** SA auto-import.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Low** |
| Implementation complexity | **Med** |
| Expected user value | **High** |
| Research value | **High** |

**Dependencies:** PLAT-RT-X1, F1, F3, F4 frozen.

**Governance notes:** Explanatory-only metrics and handoff eligibility; browser capture forbidden; separate from registry RT-1..7 realism waves.

**Status:** **PLAN-RT-F5 frozen** — see [rt_f5_freeze_audit.md](rt_f5_freeze_audit.md), [rt_roadmap_plat_rt_f5_v1.md](rt_roadmap_plat_rt_f5_v1.md). PLAT-RT-F5 not authorized until implementation wave.

---

### F5b — Runtime fidelity coupling (Gazebo / sensor-truth)

**Description:** Default-off coupling to Gazebo pose truth and sim-scoped sensor-truth snapshots (LOS, dome, visibility) with three-layer authority (command / truth-attested / explanatory) and optional experiment fidelity metrics — high misread risk if labeled as operational sensor truth.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **Med–High** |
| Implementation complexity | **High** |
| Expected user value | **Med** |
| Research value | **High** |

**Governance notes:** V2 fictional terrain boundary must not blur; separate from registry RT-1..7 realism waves; distinct from PLAN-RT-F5 experiment architecture; no bridge HTTP changes in PLAN.

**Status:** **PLAN-RT-F5b frozen** — see [rt_f5b_freeze_audit.md](rt_f5b_freeze_audit.md), [rt_roadmap_plat_rt_f5b_v1.md](rt_roadmap_plat_rt_f5b_v1.md). PLAT-RT-F5b not authorized until implementation wave.

---

### F6 — SA workflow automation advisory

**Description:** Advisory readiness ladder (`capture_ready` → `review_complete` → `approval_ready` → `handoff_ready` → `import_ready`), automation boundaries, checklist cognition UI planning, optional maintainer batch helpers — approaches auto-import if not careful.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **High** |
| Implementation complexity | **Med–High** |
| Expected user value | **High** |
| Research value | **Med** |

**Governance notes:** Manual import only (SA1); advisory ≠ SA authority; federation/orchestration authority still forbidden; no bridge changes in PLAN.

**Status:** **PLAN-RT-F6 frozen** — [rt_f6_freeze_audit.md](rt_f6_freeze_audit.md). **PLAT-RT-F6 complete** (P0–P2) — [rt_plat_f6_p2_freeze_audit.md](rt_plat_f6_p2_freeze_audit.md).

---

### F7 — Distributed multi-bridge / PLAN-RT-M3

**Description:** Multiple bridge instances, cross-machine session coordination — explicitly forbidden without audit.

| Dimension | Score |
|-----------|-------|
| Architecture risk | **High** |
| Implementation complexity | **High** |
| Expected user value | **Med** |
| Research value | **Med** |

**Status:** **Deferred** — AGENTS.md forbids until explicit M3 wave.

---

## 2. Ranking summary

| Rank | Frontier | Arch risk | Complexity | User value | Research value | Notes |
|------|----------|-----------|------------|------------|----------------|-------|
| 1 | **F1 Experiment analytics / sweep catalog** | Low | Med | High | High | Best fit post-X1 |
| 2 | F2 Platform hardening | Low | Low–Med | Med | Low–Med | Good parallel maintenance |
| 3 | F3 RT annex timeline UI | Low | Med | Med | Med | Narrow UX extension |
| 4 | F4 Runtime realism expansion (UI) | Low | Med | Med | Med | PLAN/PLAT-RT-F4 |
| 5 | F5 Advanced runtime experiments | Low | Med | High | High | PLAN-RT-F5 frozen |
| 5b | F5b Runtime fidelity coupling | Med–High | High | Med | High | PLAN-RT-F5b frozen |
| 6 | F6 SA workflow automation advisory | High | Med–High | High | Med | PLAN frozen; high contamination risk |
| 7 | F7 Distributed M3 | High | High | Med | Med | Not recommended near-term |

---

## 3. Recommended next major frontier

**Recommendation (advisory):** **Platform consolidation review** — RT plateau audit (F1–F6 + SA waves); M3/F7 remain deferred

**Rationale:**

1. **PLAN-RT-F6 frozen** — advisory ladder, automation boundaries, and contamination review complete on top of frozen PLAT-RT-SA1/SA2/SA3 and PLAT-RT-F5/F5b.  
2. **PLAT-RT-F6 P0 frozen** — read-only derive, CLI, golden fixtures, handoff/workbench strips.  
3. **P1 surfaces checklist cognition** without batch helpers contamination risk.  
4. Avoids **F7/M3** distributed coordination landmines and auto-import shortcuts.  
5. P2 batch helpers require contamination re-check.

**Previously recommended (completed):** F1 — experiment analytics & sweep catalog (PLAN/PLAT frozen); F5 — advanced runtime experiments (PLAN/PLAT P0–P2 frozen); F5b — runtime fidelity coupling (PLAN/PLAT P0–P2 frozen); F6 P0 — readiness mirror.

**Not recommended near-term:** F7/M3 (distributed). F6 P2 batch helpers only after P0/P1 frozen.

---

## 4. Explicit non-frontiers (remain forbidden)

- Parser/topic/schema changes  
- Operational HITL/C2 / readiness scoring  
- Browser `capture_session`  
- SA viewer live RT hooks (beyond SA3 read-only)  
- Federation/orchestration authority from RT sessions  
- PX4/MAVLink/hardware assumptions  

---

## 5. Wave authorization checklist (any frontier)

Before implementation:

1. Scoped plan doc in `docs/platform/`  
2. Contract or contract annex in `docs/evaluation/` if behavior visible to maintainers  
3. Governance review + isolation audit if UI or bridge touched  
4. Freeze audit + freeze registry row  
5. Regression: bridge pytest + tier0-rt-ui (+ frontier-specific tests)

---

## 6. Stop line

This roadmap is **advisory**. **PLAN-RT-R2** freeze means: stop before any implementation wave not meeting §5 checklist.
