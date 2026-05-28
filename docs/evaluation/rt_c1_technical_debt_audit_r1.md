# RT-C1 — Technical Debt Audit R1

**Phase:** PLAN-RT-C1 — platform consolidation review  
**Plan:** [rt_c1_runtime_platform_consolidation_plan.md](../platform/rt_c1_runtime_platform_consolidation_plan.md)  
**Master review:** [rt_c1_platform_consolidation_review_r1.md](rt_c1_platform_consolidation_review_r1.md)

No runtime code was modified for this audit.

---

## 1. Closure matrix

### 1.1 R2 residuals

| ID | R2 status | C1 status | Notes |
|----|-----------|-----------|-------|
| R2-DEBT-UI-01 | Open (`App.tsx` ~628 LOC) | **Residual** | ~668 LOC; primary UI integration hotspot |
| R1-LIFE-04 | Residual | **Accepted** | Template apply vs workflow reset |
| R1-DEBT-02 | Partial | **Closed** | Subscription path primary |
| R1-DEBT-04 | Mitigated | **Shifted** | Entropy risk moved to `experiment/` subtree |

### 1.2 F-wave debt items (from PLAT freeze audits)

| Wave | Debt / risk noted at freeze | C1 closure |
|------|----------------------------|------------|
| F1 | Sweep catalog maintenance | **Accepted** — fixture + CLI discipline |
| F2 | Staging orphan dirs | **Mitigated** — `rt_staging_integrity_audit.py` optional |
| F3 | Annex schema drift vs TAC5 | **Closed** — shared `tacticalAnnexSchema.ts` |
| F4 | Contour default-off | **Closed** — governance tests |
| F5 | Experiment spec compile complexity | **Accepted** — isolated `experimentSpecCompile.ts` |
| F5b | Fidelity misread risk | **Mitigated** — banners + default-off |
| F6 P2 | Batch helper contamination | **Mitigated** — dry-run default; P2 contamination review on record |

| Finding ID | Verdict |
|------------|---------|
| C1-DEBT-CLOSE-01 | Pass — no open P0 debt from F-waves |

---

## 2. Hotspots (current)

| Hotspot | Scale | Risk | Notes |
|---------|-------|------|-------|
| `App.tsx` | ~668 LOC | **Medium** | Sessions, Cesium, tactical, experiment, handoff, fidelity panels |
| `src/experiment/` | 64 files, ~8.1k LOC | **Medium** | F1/F3/F5 + X1; largest UI subtree |
| `session_manager.py` facade | ~580 LOC | **Low–Med** | Stable router post-R3a |
| `advisory_derive.py` + `batch_advisory.py` | ~713 LOC | **Med** | F6; contamination-sensitive |
| `fidelity_coupling.py` + UI `fidelity/` | ~600 LOC | **Med** | Default-off; labeling critical |
| `tactical_controller.py` stack | ~1.2k LOC (multi-file) | **Low** | TAC2–5 frozen |
| Cesium bundle | Heavy dep | **Med** | Build ~316 KB JS (gzip ~92 KB) — unchanged from R2 |

| Finding ID | Verdict |
|------------|---------|
| C1-DEBT-HOT-01 | Pass-with-conditions — concentration documented; no monolith regression in bridge |

---

## 3. Ownership drift

Cross-walk of [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) vs implementation (May 2026):

| Concern | Documented owner | Drift |
|---------|------------------|-------|
| Lifecycle | `session_lifecycle_handlers.py` | **None** |
| Multi-session registry | `session_registry.py`, handlers | **None** |
| Tactical | `session_tactical_handlers.py`, `tactical_controller.py` | **None** |
| Capture | `session_capture_handler.py` | **None** |
| Fidelity coupling | `fidelity_coupling.py`, capture block | **None** — additive F5b |
| Advisory derive | `advisory_derive.py` (bridge), `deriveAdvisoryState.ts` (UI) | **None** — parallel derive, same contract |
| UI workspace | `useRtSessionWorkspace.ts` | **None** |

**Verdict:** Ownership map holds. F-waves added modules without undocumented logic in facade.

---

## 4. Duplication inventory

| Area | Duplication | Severity | Recommendation |
|------|-------------|----------|----------------|
| Advisory derive | Bridge `advisory_derive.py` vs UI `deriveAdvisoryState.ts` | Low | Intentional dual derive; golden fixtures align |
| Experiment manifest vs capture manifest | Overlapping summary fields | Low | Optional schema note — deferred |
| F6 deny-path strings in UI source | `platform/sa-r0-viewer` literal in batch/preview modules breaks bridge string isolation tests | Low–Med | Move paths to shared constant or test allow-list; optional M3/F2 maintenance |
| Fidelity vs R2e pose cognition | Truth-attested vs tri-source | Low | Contracts sufficient |
| Analytics vs X1 compare | Shared manifest inputs | Low | No merge required |
| Handoff phase vs advisory ladder | SA2 phase mapping | Low | Documented in F6 contract |

---

## 5. Test coverage map

### Bridge (pytest)

| Module | Primary test file | Coverage |
|--------|-------------------|----------|
| Session / lifecycle / multi-session | `test_rt_sandbox_bridge.py` | **Strong** |
| Adapter / sync / fidelity | same | **Strong** |
| Tactical TAC2–5 | same | **Strong** |
| Experiment batch | `test_rt_experiment_batch.py` | **Moderate** |
| Subcommand lint | `lint_rt_runtime_subcommands.py` | **Gate** — tier0 |

Approximate collect: **153** tests in `test_rt_sandbox_bridge.py` (May 2026).

### UI (Vitest)

| Area | Coverage |
|------|----------|
| Governance / isolation | **Strong** — `governance.test.ts`, `isolation.test.ts` |
| Experiment F1/F5 | **Moderate** — schema, compare, metrics, repeatability |
| Fidelity F5b | **Moderate** — `fidelityCognition.test.ts` |
| Handoff F6 | **Moderate** — `advisoryChecklist.test.ts`, `deriveAdvisoryState.test.ts` |
| Multi-session | **Moderate** — `SessionTabBar`, workspace hooks |
| Cesium / terrain | **Moderate** |

---

## 6. Future cleanup candidates (deferred)

| Candidate | Target wave | Rationale |
|-----------|-------------|-----------|
| `App.tsx` decomposition (layout hooks) | Optional F2-style maintenance or post-M3 | Reduces integration risk without feature creep |
| Background session poll optimization | **PLAN-RT-M3** | Named in [rt_roadmap_m1_m2_v1.md](rt_roadmap_m1_m2_v1.md) |
| `rt_session_inspect.py` maintainer CLI | **PLAN-RT-M3** | Multi-session ops ergonomics |
| Experiment manifest field consolidation note | Docs-only annex | Low priority |
| Advisory batch helper hardening | **PLAN-RT-F7** → **PLAT-RT-F7 P2** | Addressed in PLAN contracts; implementation in PLAT P2 |

**None are blocking** consolidation freeze.

---

## 7. Debt verdict

| Category | Verdict |
|----------|---------|
| P0 blockers | **None** |
| Hotspot concentration | **Pass-with-conditions** — `App.tsx` + `experiment/` |
| F6 contamination | **Pass-with-conditions** — monitor on F7 |
| Test posture | **Pass** — bridge strong; UI moderate on new F panels |

**Overall:** **Pass-with-conditions** — suitable for consolidation freeze. Deferred items routed to M3 or F7 roadmaps, not ad-hoc implementation.
