# RT-C1 — Platform Consolidation Review R1

**Phase:** PLAN-RT-C1 — runtime platform consolidation review (read-only)  
**Prerequisite:** PLAN-RT-R2 frozen; PLAT-RT-F1–F6 P2 frozen  
**Plan:** [rt_c1_runtime_platform_consolidation_plan.md](../platform/rt_c1_runtime_platform_consolidation_plan.md)  
**Governance review:** [rt_c1_platform_governance_review_r1.md](rt_c1_platform_governance_review_r1.md)  
**Technical debt:** [rt_c1_technical_debt_audit_r1.md](rt_c1_technical_debt_audit_r1.md)  
**Next frontiers:** [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md)  
**Freeze audit:** [rt_c1_platform_consolidation_freeze_audit.md](rt_c1_platform_consolidation_freeze_audit.md)  
**Baseline:** [rt_r2_platform_maturity_review_r1.md](rt_r2_platform_maturity_review_r1.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Platform completeness | **Pass** — primary RT roadmap + F1–F6 delivered and frozen |
| Architecture layering | **Pass** — four-layer model intact; F-waves additive with bounded modules |
| Governance / RT↔SA separation | **Pass** — re-validated post-F6; advisory ≠ authority |
| Runtime consistency | **Pass-with-conditions** — lifecycle/tactical/capture/fidelity aligned; template workflow residual unchanged |
| UX maturity | **Pass** — research workbench with experiment, fidelity, and advisory cognition |
| Technical debt | **Pass-with-conditions** — R2 UI concentration grew; experiment stack expanded |
| Expansion readiness | **Conditional** — see [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md) |

**Platform maturity:** The RT interactive sandbox has reached a **second documented consolidation plateau**. All major platform waves through **PLAT-RT-F6 P2** are implemented: multi-session sandbox, SVG + Cesium editing, Gazebo adapter path (mock default), terrain and runtime realism cognition, tactical modes and continuity, RT→SA manual handoff and replay visibility, experimentation workbench, analytics and sweep catalog, annex continuity review, advanced experiment model and metrics, fidelity coupling, and SA workflow advisory with batch maintainer helpers.

**Recommendation:** Freeze **PLAN-RT-C1** (docs only). **Advisory next frontier:** **PLAN-RT-M3** (local multi-session polish) — see v2 roadmap. **PLAN-RT-F7** (post-F6 advisory expansion) is the alternate ranked candidate. Distributed multi-bridge remains forbidden.

**Stop line:** No post-C1 implementation without plan + governance review + freeze audit.

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
| Mirrors non-authoritative | **Pass** — registry command truth; feedback/telemetry/fidelity/advisory mirrors labeled |
| Writable roots | **Pass** — `runs/rt_sandbox/` only for RT staging |
| Multi-session | **Pass** — `session_registry.py`, cap=3, editing lock |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-ARCH-01 | Pass | Four-layer stack unchanged from R2; F-waves did not add bridge HTTP surface |
| C1-ARCH-02 | Pass | `RuntimeStub` default when `enable_gazebo_adapter=false` |
| C1-ARCH-03 | Pass | Facade delegates to handlers per [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) |

### 1.2 Module concentration (read-only audit, May 2026)

| Module | LOC (approx.) | Role | Risk |
|--------|---------------|------|------|
| `App.tsx` | 668 | UI integration hub | **Medium** — grew from ~628 at R2 |
| `session_manager.py` | 580 | Bridge command router | **Low–Med** — stable post-R3a |
| `src/experiment/` (64 files) | ~8.1k total | F1/F3/F5/X1 UI + derive | **Medium** — largest UI subtree |
| `tactical_controller.py` | 408 | Per-session tactical | **Low** — isolated |
| `advisory_derive.py` | 393 | F6 readiness mirror | **Low–Med** — maintainer/derive boundary |
| `batch_advisory.py` | 320 | F6 P2 batch scan | **Med** — contamination-sensitive |
| `fidelity_coupling.py` | 296 | F5b default-off coupling | **Med** — misread risk if mislabeled |
| `fidelityCognition.ts` | 308 | F5b UI cognition | **Low–Med** — banner discipline |
| `handoff/` (UI) | ~1.0k | F6 advisory + SA2 phase | **Low–Med** — parallel derive paths |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-ARCH-04 | Pass-with-conditions | `App.tsx` remains primary integration point — C1-DEBT-UI-01 |
| C1-ARCH-05 | Pass-with-conditions | `experiment/` concentration is intentional but increases review surface |
| C1-ARCH-06 | Pass | F-wave bridge modules are bounded files, not monolith growth in facade |

### 1.3 Cross-wave overlaps

| Overlap | Layers | Assessment |
|---------|--------|------------|
| X1 workbench vs F1 analytics | UI `experiment/` + `analyticsDerive.ts` | **Intentional** — F1 extends X1 with sweep catalog; shared manifest schema |
| F3 annex review vs TAC5 capture | UI panels + `tactical_annex.json` | **Intentional** — RT-local timeline; not SA scrubber |
| F5 experiment model vs X1 pin/compare | `experimentSpecCompile.ts`, batch CLI | **Intentional** — F5 adds taxonomy; no new bridge commands |
| F5b fidelity vs R2e pose cognition | `fidelity_coupling.py`, capture block | **Intentional** — tri-source + optional truth-attested layer; default-off |
| F6 advisory vs SA1/SA2 handoff | `deriveAdvisoryState.ts`, mirror strip | **Intentional** — advisory ladder complements manual import; must not commit |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-ARCH-07 | Pass | No duplicate authority paths introduced by F-waves |
| C1-ARCH-08 | Pass-with-conditions | Field overlap (manifest vs capture vs advisory) documented; optional schema note deferred |
| C1-ARCH-09 | Pass | F2 import guards prevent experiment UI from bypassing staging approval |

### 1.4 F-wave subsystem map

| Wave | Contract(s) | Bridge / UI anchor |
|------|-------------|-------------------|
| F1 | [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md) | `analyticsDerive.ts`, `rt_experiment_analytics.py` |
| F2 | [rt_runtime_cleanup_hardening_v1.md](rt_runtime_cleanup_hardening_v1.md) | `clear_tactical_state`, `experimentImportGuards.ts` |
| F3 | [rt_experiment_annex_review_ui_v1.md](rt_experiment_annex_review_ui_v1.md) | `TacticalAnnexReviewPanel`, `rt_experiment_annex_pack.py` |
| F4 | [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md) | contour/LOS layers, terrain cognition |
| F5 | [rt_experiment_model_v1.md](rt_experiment_model_v1.md) | `experimentSpecCompile.ts`, F5 panels, metrics CLI |
| F5b | [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md) | `fidelity_coupling.py`, `FidelityTruthCognitionStrip` |
| F6 | [rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md) | `advisory_derive.py`, `SaWorkflowAdvisoryPanel`, batch CLIs |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-ARCH-10 | Pass | Each F-wave has frozen plan + PLAT audit + contract anchor |
| C1-ARCH-11 | Pass | F-waves did not merge platform / simulation / SA frontiers |

---

## 2. Governance review (summary)

Full checklist: [rt_c1_platform_governance_review_r1.md](rt_c1_platform_governance_review_r1.md).

| Area | Verdict |
|------|---------|
| Authority boundaries | **Pass** |
| RT↔SA separation | **Pass** |
| F6 advisory ≠ auto-import | **Pass** (P2 re-check) |
| Replay boundaries | **Pass** |
| Deny-by-default | **Pass** |
| Maintainer-only batch surfaces | **Pass-with-conditions** — P2 helpers require ongoing discipline |

---

## 3. Runtime consistency review

Re-validated against R2 baseline plus F-wave behavior.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-CONS-01 | Pass | Multi-session cap=3, editing lock, per-session telemetry |
| C1-CONS-02 | Pass | F5b `enable_fidelity_coupling` default-off; truth mirrors explanatory |
| C1-CONS-03 | Pass | F6 `import_ready` does not invoke SA import or corpus write |
| C1-CONS-04 | Pass | Tactical TAC1–5 unchanged by F-waves |
| C1-CONS-05 | Pass-with-conditions | R1-LIFE-04 template/workflow residual unchanged |
| C1-CONS-06 | Pass | Capture → normalization → SA1 manual path intact |

---

## 4. UX / cognition maturity

| Surface | Waves | Assessment |
|---------|-------|------------|
| Experiment analytics + sweep | F1 | Explanatory derive; sweep catalog read-only |
| Annex continuity hub | F3 | RT-local; not replay authority |
| Realism contours / LOS | F4 | Default-off layers; fictional boundary |
| Advanced experiment UI | F5 P1–P2 | Matrix, metrics, repeatability trend |
| Fidelity truth strips | F5b P1–P2 | `BANNER_FIDELITY_TRUTH`; compare strip |
| SA workflow advisory | F6 P0–P2 | `BANNER_SA_WORKFLOW_ADVISORY`; checklist; batch CLI docs only in UI |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-UX-01 | Pass | Governance banners stack without contradiction |
| C1-UX-02 | Pass | Workstation shell absorbs F panels without rosbridge |
| C1-UX-03 | Pass-with-conditions | UI density high — mentor/demo suitable, not C2 |

---

## 5. Prior-wave re-validation

### 5.1 R2 residuals

| ID | R2 status | C1 status |
|----|-----------|-----------|
| R2-DEBT-UI-01 (`App.tsx`) | Residual | **Open** — 668 LOC; deferred decomposition |
| R1-LIFE-04 (template/workflow) | Residual | **Accepted** — documented |
| R2-GOV-SA-03 (automation risk) | Pass-with-conditions | **Mitigated** — F6 shipped with contamination reviews |

### 5.2 F1–F6 delivery vs plan intent

| Wave | Plan intent | C1 verdict |
|------|-------------|------------|
| F1 | Analytics + sweep catalog | **Delivered** — PLAT frozen |
| F2 | Teardown, import guards, staging audit | **Delivered** |
| F3 | Annex timeline in RT workbench | **Delivered** |
| F4 | Runtime realism UI expansion | **Delivered** |
| F5 | Experiment model + metrics + UI | **Delivered** P0–P2 |
| F5b | Fidelity coupling default-off | **Delivered** P0–P2 |
| F6 | Advisory ladder + batch helpers | **Delivered** P0–P2; contamination re-checked at P2 |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| C1-REV-01 | Pass | All F-wave PLAN and PLAT rows in freeze registry match delivered scope |
| C1-REV-02 | Pass | No unauthorized bridge commands across F1–F6 |

---

## 6. Platform maturity statement

The **RT interactive sandbox platform is consolidation-complete** at the F6 plateau. The platform exhibits:

- **Coherent layering** with frozen contracts per wave and additive F-wave modules
- **Governance-aware UX** including fidelity and SA workflow advisory banners
- **Bounded experimentation** — analytics, spec compile, metrics, fidelity compare — without SA auto-import
- **Manual RT→SA bridge** with advisory maintainer helpers that default to dry-run
- **Local multi-session research** without distributed runtime

Future work should be **selective frontier expansion** (M3 polish or F7 advisory), not ad-hoc feature accretion. See [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md).

---

## Appendix — Subsystem cross-reference

| Subsystem | Contracts | Implementation | Tests |
|-----------|-----------|----------------|-------|
| Bridge | `rt_bridge_contract_v1`, ownership v1 | `session_manager` + handlers | `test_rt_sandbox_bridge.py` |
| Adapter | G1–G6, poll semantics | `runtime_adapter`, `adapter_poll`, `fidelity_coupling` | bridge pytest |
| UI core | T1–T5, V1–V2 | `App.tsx`, workstation, Cesium | vitest + tier0-rt-ui |
| Experiment | X1, F1, F3, F5 | `src/experiment/` | vitest + `test_rt_experiment_batch.py` |
| Fidelity | F5b | `fidelity/`, `capture_fidelity_coupling.py` | `fidelityCognition.test.ts` |
| Handoff / advisory | F6, SA1–2 | `handoff/`, `advisory_derive.py`, batch CLIs | advisory + isolation tests |
| Tactical | TAC1–5 | `tactical_*` | bridge + panel tests |
| RT→SA replay | SA3 | SA viewer panel | packager + viewer tests |
