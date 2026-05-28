# RT-C2 — Technical Debt Audit R1

**Phase:** PLAN-RT-C2 — platform consolidation review  
**Plan:** [rt_c2_runtime_platform_consolidation_plan.md](../platform/rt_c2_runtime_platform_consolidation_plan.md)  
**Master review:** [rt_c2_platform_consolidation_review_r1.md](rt_c2_platform_consolidation_review_r1.md)  
**Baseline:** [rt_c1_technical_debt_audit_r1.md](rt_c1_technical_debt_audit_r1.md)

No runtime code was modified for this audit.

---

## 1. Closure matrix

### 1.1 C1 residuals

| ID | C1 status | C2 status | Notes |
|----|-----------|-----------|-------|
| C1-DEBT-UI-01 / R2-DEBT-UI-01 | Open (`App.tsx` ~668 LOC) | **P1 later** | ~725 LOC; integration hub grew with F7 panels |
| R1-LIFE-04 | Accepted | **Acceptable residual** | Template apply vs workflow reset |
| F6 deny-path string isolation | Low–Med | **P1 later** | 2 bridge pytest failures unchanged |

### 1.2 C1 advisory picks (M3, F7)

| Wave | Debt / risk noted at PLAT freeze | C2 closure |
|------|----------------------------------|------------|
| M3 P0 | Poll policy complexity in workspace hook | **Closed** — [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md) + tests |
| M3 P1 | Tab-switch confirm edge cases | **Closed** — frozen P1 |
| M3 P2 | localStorage tab order | **Accepted** — local-only preference |
| F7 P0 | Queue derive vs UI parity | **Closed** — golden fixtures + `test_advisory_queue.py` |
| F7 P1 | Triage UI contamination | **Mitigated** — read-only panel; contamination review on record |
| F7 P2 | v2 export dry_run discipline | **Closed** — always true; P2 freeze audit |

| Finding ID | Verdict |
|------------|---------|
| C2-DEBT-CLOSE-01 | Pass — M3/F7 PLAT debt items closed or accepted |
| C2-DEBT-CLOSE-02 | Pass — no new P0 debt from F7/M3 |

### 1.3 F1–F6 items (from C1)

| C1 closure | C2 status |
|------------|-----------|
| F-wave P0 blockers | **Unchanged — closed** |
| F6 batch contamination | **Mitigated** — extended by F7 with same discipline |

---

## 2. Hotspots (current)

| Hotspot | Scale | Risk | Tier |
|---------|-------|------|------|
| `App.tsx` | ~725 LOC | **Medium** | **P1 later** |
| `src/experiment/` | 64 files, ~8.1k LOC | **Medium** | **P1 later** |
| `src/handoff/` | 30 files, ~1.5k LOC | **Med** | **P1 later** (bounded; F8 would increase) |
| `session_manager.py` facade | ~580 LOC | **Low–Med** | **Acceptable residual** |
| `advisory_queue.py` + `batch_advisory.py` | ~500+ LOC | **Med** | **Acceptable residual** with discipline |
| `advisory_derive.py` + `deriveAdvisoryState.ts` | dual derive | **Low** | **Acceptable residual** |
| `useRtSessionWorkspace.ts` | ~372 LOC | **Low–Med** | **Acceptable residual** |
| Cesium bundle | ~455 KB JS (gzip ~127 KB) | **Med** | **P1 later** — V3 candidate touches this |
| Bridge isolation pytest | 2 failures | **Low–Med** | **P1 later** — deny-path literals |

| Finding ID | Verdict |
|------------|---------|
| C2-DEBT-HOT-01 | Pass-with-conditions — concentration documented; bridge facade stable |

---

## 3. Ownership drift

Cross-walk of [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) vs implementation (May 2026):

| Concern | Documented owner | Drift |
|---------|------------------|-------|
| Lifecycle | `session_lifecycle_handlers.py` | **None** |
| Multi-session registry | `session_registry.py`, handlers | **None** |
| Tactical | `session_tactical_handlers.py` | **None** |
| Capture | `session_capture_handler.py` | **None** |
| Advisory derive | `advisory_derive.py`, `deriveAdvisoryState.ts` | **None** |
| Advisory queue (F7) | `advisory_queue.py`, `advisoryQueue.ts` | **None** — additive |
| UI workspace / poll | `useRtSessionWorkspace.ts` | **None** — M3 policy documented |

**Verdict:** Ownership map holds. F7/M3 added modules without undocumented logic in facade.

| Finding ID | Verdict |
|------------|---------|
| C2-DEBT-OWN-01 | Pass |

---

## 4. Duplication inventory

| Area | Duplication | Severity | Recommendation |
|------|-------------|----------|----------------|
| Advisory derive | Bridge vs UI | Low | Intentional; golden fixtures align |
| Advisory queue | Bridge `advisory_queue.py` vs UI `advisoryQueue.ts` | Low | Intentional mirror |
| F6/F7 deny-path strings | `platform/sa-r0-viewer` literal in batch/preview | Low–Med | Shared constant or test allow-list — **P1 later** |
| Experiment vs handoff rollup | Triage experiment handoff navigation | Low | Documented — no merge required |
| Maintainer CLI overlap | batch advisory + v2 export + dry-run-review | Low | Optional consolidation — **P1 later** |

| Finding ID | Verdict |
|------------|---------|
| C2-DEBT-DUP-01 | Pass-with-conditions — F7 added bounded duplication |

---

## 5. Test coverage map

### Bridge (pytest)

| Module | Primary test file | Coverage |
|--------|-------------------|----------|
| Session / lifecycle / multi-session | `test_rt_sandbox_bridge.py` | **Strong** (2 pre-existing isolation failures) |
| Advisory queue / batch | `test_advisory_queue.py`, `test_rt_handoff_batch_advisory.py` | **Strong** — 26 tests |
| Experiment batch | `test_rt_experiment_batch.py` | **Moderate** |
| Subcommand lint | `lint_rt_runtime_subcommands.py` | **Gate** — tier0 |

### UI (Vitest)

| Area | Coverage |
|------|----------|
| Governance / isolation | **Strong** |
| Handoff F6/F7 | **Moderate** — advisory, triage, batch export preview |
| Multi-session M3 | **Moderate** — tab order, workspace, background rows |
| Experiment F1/F5 | **Moderate** |
| Cesium / terrain V1/V2 | **Moderate** |

**Recorded at C2 freeze:** 249 vitest passed; advisory pytest 26 passed; bridge 155 passed / 2 failed (pre-existing).

| Finding ID | Verdict |
|------------|---------|
| C2-DEBT-TEST-01 | Pass-with-conditions — isolation string debt documented |

---

## 6. Categorization

### P0 urgent

**None identified.** F7/M3 freezes hold; governance pass; tier0-rt-ui passes.

### P1 later

| Item | Rationale |
|------|-----------|
| `App.tsx` decomposition | Reduce integration risk before V3 or X2 |
| `experiment/` subtree modularization | X2 candidate increases surface |
| `handoff/` package boundaries | F8 advisory expansion risk |
| Deny-path string centralization | Fix 2 bridge isolation pytest failures |
| Optional advisory CLI consolidation | Maintainer ergonomics without new authority |
| Cesium bundle diet | V3 may add markers — plan weight budget |

### Acceptable residual

| Item | Rationale |
|------|-----------|
| `session_manager.py` facade router | Post-R3a stable |
| Dual advisory derive | Contract-aligned |
| Mock-default adapter | Governance default |
| M3 localStorage tab order | Local UX preference only |
| R1-LIFE-04 template/workflow | Documented accepted residual |

---

## 7. Debt verdict

**Pass-with-conditions — suitable for freeze.**

No P0 debt blockers. UI concentration (`App.tsx`, `experiment/`, `handoff/`) is the primary maintenance cost going into v4 frontiers. Recommend **PLAN-RT-V3** before **F8** or **X2** if minimizing contamination and authority risk (see v4 §6).

**Stop line:** C2 does not authorize debt-driven implementation without a scoped PLAN wave.
