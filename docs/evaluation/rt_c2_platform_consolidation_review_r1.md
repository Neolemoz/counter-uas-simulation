# RT-C2 — Platform Consolidation Review R1

**Phase:** PLAN-RT-C2 — runtime platform consolidation review (read-only)  
**Prerequisite:** PLAN-RT-C1 frozen; PLAT-RT-M3 P0–P2 frozen; PLAT-RT-F7 P0–P2 frozen  
**Plan:** [rt_c2_runtime_platform_consolidation_plan.md](../platform/rt_c2_runtime_platform_consolidation_plan.md)  
**Governance review:** [rt_c2_platform_governance_review_r1.md](rt_c2_platform_governance_review_r1.md)  
**Technical debt:** [rt_c2_technical_debt_audit_r1.md](rt_c2_technical_debt_audit_r1.md)  
**Next frontiers:** [rt_roadmap_next_frontiers_v4.md](rt_roadmap_next_frontiers_v4.md)  
**Freeze audit:** [rt_c2_platform_consolidation_freeze_audit.md](rt_c2_platform_consolidation_freeze_audit.md)  
**Baseline:** [rt_c1_platform_consolidation_review_r1.md](rt_c1_platform_consolidation_review_r1.md), [rt_plat_f7_p2_freeze_audit.md](rt_plat_f7_p2_freeze_audit.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Platform completeness | **Pass** — primary RT roadmap + F1–F7 + M3 delivered and frozen |
| Architecture layering | **Pass** — four-layer model intact; F7/M3 additive with bounded modules |
| Governance / RT↔SA separation | **Pass** — re-validated post-F7; advisory/export ≠ authority |
| Runtime consistency | **Pass-with-conditions** — lifecycle/tactical/capture/fidelity aligned; template workflow residual unchanged |
| UX maturity | **Pass** — research workbench with F7 triage, M3 multi-session polish, experiment/fidelity/advisory cognition |
| Technical debt | **Pass-with-conditions** — `App.tsx` / `experiment/` / `handoff/` concentration grew since C1 |
| Expansion readiness | **Conditional** — see [rt_roadmap_next_frontiers_v4.md](rt_roadmap_next_frontiers_v4.md) |

**Platform maturity:** The RT interactive sandbox has reached a **third documented consolidation plateau** (post-F7). Since C1, **PLAT-RT-M3** delivered session inspect CLI, background poll policy UX, tab-switch confirm, display names, and tab reorder; **PLAT-RT-F7** delivered advisory queue/cohort derive, triage queue UI, batch review v2 export, standup/grouped maintainer exports, and dry-run hardening.

**Strengths:**

- Closed F7 trilogy without bridge HTTP protocol changes or SA viewer edits
- M3 local multi-session polish within cap=3 single-bridge model
- Maintainer advisory stack (F6 + F7) with contamination gates and `dry_run` always true on v2 export
- Primary roadmap (S/G/R/T/M/TAC/SA/V/X) plus F1–F7 remain frozen and contract-anchored

**Residual risks:**

- `App.tsx` integration hub (~725 LOC) and `src/experiment/` (64 files) remain primary review surfaces
- `src/handoff/` grew to ~30 files — advisory/triage UI increases contamination discipline burden
- Pre-existing bridge string-isolation pytest failures (F6/F7 deny-path literals) — not RT→SA coupling
- Cesium bundle weight (~455 KB JS build) — unchanged structural risk

**Recommendation:** Freeze **PLAN-RT-C2** (docs only). **Advisory next frontier:** **PLAN-RT-V3** (runtime visualization fidelity) per v4 roadmap — lowest contamination among ranked candidates. **PLAN-RT-F8** (advisory expansion) and **PLAN-RT-X2** (experiment workbench v2) are alternates. Distributed multi-bridge remains forbidden.

**Stop line:** No post-C2 implementation without plan + governance review (+ contamination if advisory) + freeze audit.

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
| Mirrors non-authoritative | **Pass** — registry command truth; F7 queue/export/triage are explanatory |
| Writable roots | **Pass** — `runs/rt_sandbox/` only for RT staging |
| Multi-session | **Pass** — `session_registry.py`, cap=3, editing lock; M3 poll policy in UI |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-ARCH-01 | Pass | Four-layer stack unchanged; F7/M3 did not add bridge HTTP surface |
| C2-ARCH-02 | Pass | `RuntimeStub` default when `enable_gazebo_adapter=false` |
| C2-ARCH-03 | Pass | Facade delegates per [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) |

### 1.2 Module concentration (read-only audit, May 2026)

| Module | LOC (approx.) | Role | Risk |
|--------|---------------|------|------|
| `App.tsx` | 725 | UI integration hub | **Medium** — grew from ~668 at C1 |
| `session_manager.py` | 580 | Bridge command router | **Low–Med** — stable post-R3a |
| `src/experiment/` (64 files) | ~8.1k total | F1/F3/F5/X1 UI + derive | **Medium** — largest UI subtree |
| `src/handoff/` (30 files) | ~1.5k | F6/F7 advisory + triage | **Med** — contamination-sensitive |
| `advisory_queue.py` + `batch_advisory.py` | ~500+ | F7 queue + v2 export | **Med** — maintainer paths |
| `useRtSessionWorkspace.ts` | 372 | M3 poll/slot orchestration | **Low–Med** — bounded policy |
| `tactical_controller.py` stack | ~1.2k (multi-file) | TAC2–5 | **Low** — frozen |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-ARCH-04 | Pass-with-conditions | `App.tsx` primary integration — C2-DEBT-UI-01 (deferred) |
| C2-ARCH-05 | Pass-with-conditions | `experiment/` concentration intentional; X2 would increase surface |
| C2-ARCH-06 | Pass | F7 bridge modules bounded; no facade monolith regression |
| C2-ARCH-07 | Pass-with-conditions | `handoff/` growth post-F7 — bounded by contracts, higher review burden |

### 1.3 Stack deltas since C1

| Stack | C1 state | C2 state |
|-------|----------|----------|
| **Runtime** | F1–F6 on primary roadmap | Unchanged bridge contract; M3 poll constants in UI |
| **Tactical** | TAC1–5 frozen | Unchanged |
| **SA handoff + advisory** | F6 P0–P2 | **+F7** queue, triage panel, batch v2, dry-run-review |
| **Experiment / analytics / fidelity** | F5/F5b + X1 | Unchanged scope; concentration noted |
| **Multi-session UX** | M1/M2 baseline | **+M3** inspect CLI, background poll pause, tab order, display names |

### 1.4 Cross-wave overlaps

| Overlap | Layers | Assessment |
|---------|--------|------------|
| F7 triage vs F6 advisory panel | `handoff/` + workstation | **Intentional** — triage read-only; no commit actions |
| F7 batch v2 vs F6 batch scan | `batch_advisory.py`, CLIs | **Intentional** — v2 schema additive; dry_run enforced |
| M3 poll vs T1 telemetry pull | `useRtSessionWorkspace`, constants | **Intentional** — slot-gated refresh per [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md) |
| X1 workbench vs F7 experiment rollup | experiment panels + triage handoff | **Intentional** — rollup is navigation only |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-ARCH-08 | Pass | No duplicate authority paths from F7/M3 |
| C2-ARCH-09 | Pass-with-conditions | Advisory + experiment field overlap documented |
| C2-ARCH-10 | Pass | F2 import guards still gate experiment UI staging |

### 1.5 Wave subsystem map (F6–F7, M3)

| Wave | Contract(s) | Bridge / UI anchor |
|------|-------------|-------------------|
| F6 | [rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md) | `advisory_derive.py`, `SaWorkflowAdvisoryPanel`, batch CLIs |
| F7 | [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md), [rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md) | `advisory_queue.py`, `AdvisoryTriageQueuePanel`, v2 export |
| M3 | [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md) | `rt_session_inspect.py`, `useRtSessionWorkspace`, `BackgroundDiagnostics` |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-ARCH-11 | Pass | F7/M3 have frozen PLAT audits + contract anchors |
| C2-ARCH-12 | Pass | F7/M3 did not merge platform / simulation / SA frontiers |

---

## 2. Governance review (summary)

Full checklist: [rt_c2_platform_governance_review_r1.md](rt_c2_platform_governance_review_r1.md).

| Area | Verdict |
|------|---------|
| Authority boundaries | **Pass** |
| RT↔SA separation | **Pass** |
| F7 advisory/export ≠ auto-import | **Pass** (P2 re-check) |
| Replay boundaries | **Pass** |
| Deny-by-default | **Pass** |
| Maintainer-only batch surfaces | **Pass-with-conditions** — v2 export discipline ongoing |

---

## 3. Runtime consistency review

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-CONS-01 | Pass | Multi-session cap=3, editing lock, M3 background poll policy |
| C2-CONS-02 | Pass | F5b fidelity default-off unchanged |
| C2-CONS-03 | Pass | F7 triage/export does not invoke SA import or corpus write |
| C2-CONS-04 | Pass | Tactical TAC1–5 unchanged by F7/M3 |
| C2-CONS-05 | Pass-with-conditions | R1-LIFE-04 template/workflow residual unchanged |
| C2-CONS-06 | Pass | Capture → normalization → SA1 manual path intact |
| C2-CONS-07 | Pass | F7 v2 `dry_run` always true; dry-run-review CLI preview-only |

---

## 4. UX / cognition maturity

| Surface | Waves | Assessment |
|---------|-------|------------|
| Multi-session tabs + background diagnostics | M3 | Poll pause, stale chips, tab reorder, display names |
| Advisory triage queue | F7 P1 | Read-only queue panel; grouped blockers |
| Batch export v2 / standup | F7 P2 | Maintainer CLI + copy JSON preview; opt-in file write |
| Prior F-wave cognition | F1–F6, V1–V2, X1 | Unchanged; stacked in workstation |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-UX-01 | Pass | Governance banners stack without contradiction |
| C2-UX-02 | Pass | M3 poll UX does not bypass editing lock |
| C2-UX-03 | Pass-with-conditions | UI density high — mentor/demo suitable, not operational C2 |

---

## 5. Prior-wave re-validation

### 5.1 C1 residuals

| ID | C1 status | C2 status |
|----|-----------|-----------|
| C1-DEBT-UI-01 (`App.tsx`) | Open (~668 LOC) | **Open** (~725 LOC) |
| R1-LIFE-04 | Accepted | **Accepted** |
| F6 deny-path string isolation | Low–Med | **Open** — 2 bridge pytest failures pre-existing |

### 5.2 C1 advisory picks (M3, F7) delivery

| Wave | Plan intent | C2 verdict |
|------|-------------|------------|
| M3 P0 | Session inspect + per-slot poll | **Delivered** |
| M3 P1 | Poll pause + tab confirm + display names | **Delivered** |
| M3 P2 | Tab reorder + richer background rows | **Delivered** — **PLAT-RT-M3 complete** |
| F7 P0 | Queue/cohort/blocker + batch summary v1 | **Delivered** |
| F7 P1 | Triage queue UI | **Delivered** |
| F7 P2 | Batch v2 export + dry-run hardening | **Delivered** — **PLAT-RT-F7 complete** |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C2-REV-01 | Pass | C1-ranked M3 and F7 delivered per freeze audits |
| C2-REV-02 | Pass | No unauthorized bridge commands across F7/M3 |
| C2-REV-03 | Pass | F1–F6 scope unchanged and still frozen |

---

## 6. Platform maturity statement

The **RT interactive sandbox platform is consolidation-complete** at the **post-F7 plateau**. The platform exhibits:

- **Coherent layering** with frozen contracts per wave and additive F7/M3 modules
- **Governance-aware maintainer UX** — triage queue, batch v2 export, dry-run defaults
- **Local multi-session research polish** — inspect CLI, poll policy, tab UX — without distributed runtime
- **Bounded experimentation** — analytics, spec compile, metrics, fidelity compare — without SA auto-import
- **Manual RT→SA bridge** with advisory helpers that default to dry-run

Future work should be **selective frontier expansion** (F8, V3, or X2 per v4), not ad-hoc feature accretion. See [rt_roadmap_next_frontiers_v4.md](rt_roadmap_next_frontiers_v4.md).

---

## Appendix — Subsystem cross-reference

| Subsystem | Contracts | Implementation | Tests |
|-----------|-----------|----------------|-------|
| Bridge | `rt_bridge_contract_v1`, ownership v1 | `session_manager` + handlers | `test_rt_sandbox_bridge.py` |
| Adapter | G1–G6, poll semantics | `runtime_adapter`, `adapter_poll`, `fidelity_coupling` | bridge pytest |
| UI core | T1–T5, V1–V2, M3 | `App.tsx`, workstation, Cesium | vitest + tier0-rt-ui |
| Multi-session | M1–M3, poll policy | `session_registry`, `useRtSessionWorkspace`, `rt_session_inspect.py` | bridge + workspace tests |
| Experiment | X1, F1, F3, F5 | `src/experiment/` | vitest + `test_rt_experiment_batch.py` |
| Fidelity | F5b | `fidelity/`, `capture_fidelity_coupling.py` | `fidelityCognition.test.ts` |
| Handoff / advisory | F6, F7, SA1–2 | `handoff/`, `advisory_derive.py`, `advisory_queue.py`, batch CLIs | `test_advisory_queue.py`, `test_rt_handoff_batch_advisory.py`, isolation |
| Tactical | TAC1–5 | `tactical_*` | bridge + panel tests |
| RT→SA replay | SA3 | SA viewer panel | packager + viewer tests |
