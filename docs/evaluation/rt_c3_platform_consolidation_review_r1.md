# RT-C3 — Platform Consolidation Review R1

**Phase:** PLAN-RT-C3 — post-X2 platform checkpoint review (read-only)  
**Prerequisite:** PLAN-RT-C2 frozen; PLAT-RT-V3 P0–P2 frozen; PLAT-RT-X2 P0–P2 frozen  
**Plan:** [rt_c3_runtime_platform_consolidation_plan.md](../platform/rt_c3_runtime_platform_consolidation_plan.md)  
**Governance review:** [rt_c3_platform_governance_review_r1.md](rt_c3_platform_governance_review_r1.md)  
**Technical debt:** [rt_c3_technical_debt_audit_r1.md](rt_c3_technical_debt_audit_r1.md)  
**Next frontiers:** [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md)  
**Freeze audit:** [rt_c3_platform_consolidation_freeze_audit.md](rt_c3_platform_consolidation_freeze_audit.md)  
**Baseline:** [rt_c2_platform_consolidation_review_r1.md](rt_c2_platform_consolidation_review_r1.md), [rt_plat_x2_p2_freeze_audit.md](rt_plat_x2_p2_freeze_audit.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Platform completeness | **Pass** — primary RT roadmap + F1–F7 + M3 + V3 + **X2** delivered and frozen |
| Architecture layering | **Pass** — four-layer model intact; X2/V3 additive with bounded modules |
| Governance / RT↔SA separation | **Pass** — re-validated post-X2; experiment cohort and review packet ≠ authority |
| Runtime consistency | **Pass-with-conditions** — lifecycle/tactical/capture/fidelity aligned; template workflow residual unchanged |
| UX maturity | **Pass** — workbench v2 (X2), visualization annex (V3), F7 triage, M3 multi-session polish |
| Technical debt | **Pass-with-conditions** — `App.tsx` and `experiment/` concentration grew since C2 |
| Expansion readiness | **Conditional** — see [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md) |

**Platform maturity:** The RT interactive sandbox has reached a **fourth documented consolidation plateau** (post-X2). Since C2, **PLAT-RT-V3** delivered visual layer registry, visibility overlays, and workstation layout annex; **PLAT-RT-X2** delivered cohort index, unified review lane, compare workflow v2 (including `multi_manifest_diff`), advisory review packet export, and `useExperimentWorkbenchV2` hook extraction.

**Strengths:**

- X2 completed without bridge HTTP protocol changes, derive changes, or SA viewer edits
- Compare v2 four modes normative; multi-manifest diff is metadata-only with persistent banners
- V3 closes visualization fidelity planning/implementation trilogy (P0–P2)
- F7 + M3 + primary roadmap remain frozen and contract-anchored
- `useExperimentWorkbenchV2` partially decouples v2 wiring from `ExperimentWorkbenchPanel`

**Residual risks:**

- `App.tsx` integration hub (~776 LOC) and `src/experiment/` (96 files, ~9.8k LOC) remain primary review surfaces
- `ExperimentWorkbenchPanel` still ~831 LOC despite hook extraction
- `src/handoff/` (~30 files) — advisory/triage + X2 packet export increase maintainer-path discipline burden
- Pre-existing bridge string-isolation pytest failures (F6/F7 deny-path literals)
- Cesium production bundle (~502 KB JS minified per X2 P2 build) — structural weight unchanged

**Recommendation:** Freeze **PLAN-RT-C3** (docs only). **Advisory next PLAN wave:** **PLAN-RT-F8** (post-F7 advisory expansion) per [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md) when maintainer advisory throughput is the binding constraint; **PLAN-RT-V4** and **PLAN-RT-X3** are alternates. Distributed multi-bridge remains forbidden.

**Stop line:** No post-C3 implementation without plan + governance review (+ contamination if advisory) + freeze audit.

---

## 1. Architecture consolidation

### 1.1 Layering model

```text
Browser (rt-sandbox-ui, loopback HTTP pull)
    → BridgeSessionManager facade + session_*_handlers
        → RuntimeStub (default) | Gazebo runtime adapter (optional)
            → ROS allow-list topics (when adapter enabled)
```

| Check | Result |
|-------|--------|
| Mirrors non-authoritative | **Pass** — registry command truth; F6/F7/X2 exports are explanatory |
| Writable roots | **Pass** — `runs/rt_sandbox/` only for RT staging |
| Multi-session | **Pass** — `session_registry.py`, cap=3, editing lock; M3 poll policy in UI |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C3-ARCH-01 | Pass | Four-layer stack unchanged; X2/V3 did not add bridge HTTP surface |
| C3-ARCH-02 | Pass | `RuntimeStub` default when `enable_gazebo_adapter=false` |
| C3-ARCH-03 | Pass | Facade delegates per [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) |

### 1.2 Module concentration (read-only audit, May 2026)

| Module | Scale (approx.) | Role | Risk |
|--------|-----------------|------|------|
| `App.tsx` | 776 LOC | UI integration hub | **Medium** — grew from ~725 at C2 |
| `src/experiment/` | 96 files, ~9.8k LOC | X1/F3/F5/X2 UI + derive | **Medium–High** — largest UI subtree |
| `ExperimentWorkbenchPanel.tsx` | 831 LOC | X1 panels + v2 shell host | **Medium** — hook reduced inline v2 wiring |
| `src/handoff/` | 30 files | F6/F7 advisory + triage | **Med** — contamination-sensitive |
| `src/cesium/` | 38 files | V1/V2/V3 + T3/T5 | **Med** — bundle weight |
| `session_manager.py` facade | ~580 LOC | Bridge command router | **Low–Med** — stable post-R3a |
| `advisory_queue.py` + batch | ~500+ LOC | F7 queue + v2 export | **Med** — maintainer paths |
| `useRtSessionWorkspace.ts` | ~372 LOC | M3 poll/slot orchestration | **Low–Med** — bounded policy |
| `useExperimentWorkbenchV2.ts` | ~220 LOC | X2 v2 state/dock/lane | **Low** — positive decomposition |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C3-ARCH-04 | Pass-with-conditions | `App.tsx` primary integration — C3-DEBT-UI-01 (P1 later) |
| C3-ARCH-05 | Pass-with-conditions | `experiment/` grew +32 files since C2; X2 closed scope without bridge changes |
| C3-ARCH-06 | Pass | X2 modules bounded (`multiManifestDiff`, `reviewPacketExport`, v2 hook) |
| C3-ARCH-07 | Pass-with-conditions | `handoff/` + X2 packet export paths — bounded by contracts |

### 1.3 Stack deltas since C2

| Stack | C2 state | C3 state |
|-------|----------|----------|
| **Runtime** | F7/M3 on primary roadmap | Unchanged bridge contract |
| **Tactical** | TAC1–5 frozen | Unchanged |
| **Visualization** | V1/V2; V3 candidate | **+V3 P0–P2** layer registry, overlays, layout |
| **Advisory** | F6 + F7 | Unchanged scope; X2 packet export is experiment-local advisory |
| **Experiment** | X1 + F5/F5b; X2 candidate | **+X2 P0–P2** cohort, unified review, multi-manifest diff, packet file export |

---

## 2. Per-stack maturity

### 2.1 Runtime stack

| Component | Maturity | Notes |
|-----------|----------|-------|
| Bridge contract | **Frozen** | [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) |
| Session lifecycle | **Frozen** | R3b truth table; M3 poll UX additive |
| Gazebo adapter | **Frozen** | G2–G6; mock default |
| Capture / export | **Frozen** | RT-S5 boundary; no X2 capture mutation |

**Verdict:** **Pass** — runtime authority unchanged by X2.

### 2.2 Tactical stack

| Component | Maturity | Notes |
|-----------|----------|-------|
| TAC1–5 | **Frozen** | Manual/assisted/autonomous; capture annex |
| SA3 replay visibility | **Frozen** | Read-only replay panels |

**Verdict:** **Pass** — no tactical redesign in X2/V3.

### 2.3 Visualization stack

| Component | Maturity | Notes |
|-----------|----------|-------|
| V1/V2 | **Frozen** | Markers, terrain, fidelity chrome |
| V3 P0–P2 | **Frozen** | [rt_plat_v3_p2_freeze_audit.md](rt_plat_v3_p2_freeze_audit.md) |
| T3/T5 Cesium | **Frozen** | Interactive editing |

**Verdict:** **Pass** — visualization plateau closed at V3; V4 is optional future PLAN only.

### 2.4 Advisory stack

| Component | Maturity | Notes |
|-----------|----------|-------|
| F6 readiness mirror | **Frozen** | Advisory only |
| F7 queue/triage/export | **Frozen** | `dry_run` always true on v2 export |
| Contamination gates | **Frozen** | [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) |

**Verdict:** **Pass** — F8 candidate ranks separately in v7 (higher contamination).

### 2.5 Experiment stack (X1 / X2)

| Component | Maturity | Notes |
|-----------|----------|-------|
| X1 workbench | **Frozen** | Manifest, pairwise compare, batch |
| F1/F3/F5/F5b | **Frozen** | Derive + panels |
| X2 P0 | **Frozen** | Cohort index + v2 shells |
| X2 P1 | **Frozen** | Unified review + report dock |
| X2 P2 | **Frozen** | Multi-manifest diff + packet export — **PLAT-RT-X2 complete** |

**Verdict:** **Pass** — experiment compare v2 complete; cohort ≠ F7 readiness cohort; no cross-manifest `run_id` join.

---

## 3. Maturity plateau

**Fourth consolidation plateau (post-X2):**

1. Primary RT roadmap (S/G/R/T/M/TAC/SA/V/X) — frozen  
2. F-wave F1–F7 — frozen  
3. M3 local multi-session polish — complete  
4. V3 visualization fidelity — complete  
5. **X2 experiment workbench v2 — complete**

Expansion requires explicit PLAN → PLAT waves per v7. C3 does not authorize implementation.

| Finding ID | Verdict |
|------------|---------|
| C3-MAT-01 | Pass — fourth plateau documented |
| C3-MAT-02 | Pass-with-conditions — UI concentration remains binding for future PLAT |

---

## Related

- [rt_c3_technical_debt_audit_r1.md](rt_c3_technical_debt_audit_r1.md)
- [rt_c3_platform_governance_review_r1.md](rt_c3_platform_governance_review_r1.md)
- [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md)
