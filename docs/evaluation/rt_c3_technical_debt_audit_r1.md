# RT-C3 — Technical Debt Audit R1

**Phase:** PLAN-RT-C3 — post-X2 platform checkpoint review  
**Plan:** [rt_c3_runtime_platform_consolidation_plan.md](../platform/rt_c3_runtime_platform_consolidation_plan.md)  
**Master review:** [rt_c3_platform_consolidation_review_r1.md](rt_c3_platform_consolidation_review_r1.md)  
**Baseline:** [rt_c2_technical_debt_audit_r1.md](rt_c2_technical_debt_audit_r1.md)

No runtime code was modified for this audit.

---

## 1. Closure matrix

### 1.1 C2 residuals

| ID | C2 status | C3 status | Notes |
|----|-----------|-----------|-------|
| C2-DEBT-UI-01 / R2-DEBT-UI-01 | Open (`App.tsx` ~725 LOC) | **P1 later** | ~776 LOC; X2 P2 did not refactor `App.tsx` |
| R1-LIFE-04 | Accepted | **Acceptable residual** | Template apply vs workflow reset |
| F6 deny-path string isolation | Low–Med | **P1 later** | 2 bridge pytest failures unchanged |
| C2 `experiment/` ~64 files | P1 later | **P1 later** | **96 files**, ~9.8k LOC post-X2 |

### 1.2 PLAT-RT-X2 (P0–P2)

| Phase | Debt / risk noted at freeze | C3 closure |
|-------|----------------------------|------------|
| X2 P0 | Cohort store + read-only shells | **Closed** — [rt_plat_x2_p0_freeze_audit.md](rt_plat_x2_p0_freeze_audit.md) |
| X2 P1 | F6/F7 adjacency in unified review | **Mitigated** — contamination review on record |
| X2 P1 | `multi_manifest_diff` deferred | **Closed** — P2 table + chips |
| X2 P1 | `reviewPacketExport` deferred | **Closed** — P2 file download + copy |
| X2 P2 | `ExperimentWorkbenchPanel` concentration | **Partial** — `useExperimentWorkbenchV2` extracted; panel still ~831 LOC |
| X2 P2 | `App.tsx` refactor deferred | **Open (P1 later)** — unchanged |

| Finding ID | Verdict |
|------------|---------|
| C3-DEBT-CLOSE-01 | Pass — X2 PLAT scope closed per freeze audits |
| C3-DEBT-CLOSE-02 | Pass — no new P0 debt from X2 P2 |

### 1.3 PLAT-RT-V3 (P0–P2)

| Phase | C3 closure |
|-------|------------|
| V3 P0 | **Closed** — layer registry + contract tests |
| V3 P1 | **Closed** — visibility overlays + cognition grouping |
| V3 P2 | **Closed** — workstation layout annex — **PLAT-RT-V3 complete** |

### 1.4 v6 roadmap

| Item | C3 status |
|------|-----------|
| PLAT-RT-X2 P0–P2 | **Complete** — superseded by v7 ranking |

---

## 2. Hotspots (current)

| Hotspot | Scale | Risk | Tier |
|---------|-------|------|------|
| `App.tsx` | 776 LOC | **Medium** | **P1 later** |
| `src/experiment/` | 96 files, ~9.8k LOC | **Medium–High** | **P1 later** |
| `ExperimentWorkbenchPanel.tsx` | 831 LOC | **Medium** | **P1 later** (partial hook relief) |
| `src/handoff/` | 30 files | **Med** | **P1 later** (F8 would increase) |
| `src/cesium/` | 38 files | **Med** | **Acceptable residual** / P1 if V4 PLAT |
| `advisory_queue.py` + batch | ~500+ LOC | **Med** | **Acceptable residual** |
| `deriveAdvisoryState.ts` + `advisory_derive.py` | dual derive | **Low** | **Acceptable residual** |
| `useRtSessionWorkspace.ts` | ~372 LOC | **Low–Med** | **Acceptable residual** |
| `useExperimentWorkbenchV2.ts` | ~220 LOC | **Low** | **Closed** (X2 P2 decomposition) |
| Cesium bundle | ~502 KB JS (minified) | **Med** | **P1 later** — chunking / budget |
| Bridge isolation pytest | 2 failures | **Low–Med** | **P1 later** — deny-path literals |

| Finding ID | Verdict |
|------------|---------|
| C3-DEBT-HOT-01 | Pass-with-conditions — concentration documented; X2 growth expected and bounded |

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
| Advisory queue (F7) | `advisory_queue.py`, `advisoryQueue.ts` | **None** |
| Experiment v2 (X2) | `platform/rt-sandbox-ui/src/experiment/` | **None** — UI-only; no bridge ownership |
| UI workspace / poll | `useRtSessionWorkspace.ts` | **None** |

**Verdict:** Ownership map holds. X2/V3 added UI modules without undocumented bridge logic.

| Finding ID | Verdict |
|------------|---------|
| C3-DEBT-OWN-01 | Pass |

---

## 4. Maintainer workflow overlap

| Path | Role | Confusion risk |
|------|------|----------------|
| `rt_handoff_batch_advisory.py` / F7 v2 export | Maintainer advisory batch | Low — CLI, `dry_run` |
| `rt_sa_import_dry_run.py` | SA import preview | Low — explicit dry-run |
| X2 `reviewPacketExport.ts` | Browser JSON download | **Med** — must not be mistaken for SA import |
| SA1 `rt_sa_import` | Authoritative corpus commit | High discipline — manual only |
| F6/F7 readiness cohort vs X2 experiment cohort | Label collision | **Mitigated** — copy in cohort navigator |

| Finding ID | Verdict |
|------------|---------|
| C3-DEBT-WF-01 | Pass-with-conditions — document in maintainer checklist if F8 PLAN proceeds |

---

## 5. Test surface

| Suite | Count / status (X2 P2 baseline) |
|-------|-------------------------------|
| Vitest (`rt-sandbox-ui`) | 321 passed |
| `test_rt_experiment_batch.py` | 4 passed |
| `test_advisory_queue.py` | On regression matrix |
| `test_rt_handoff_batch_advisory.py` | On regression matrix |
| `test_rt_sandbox_bridge.py` | 151 passed, 2 failed (pre-existing) |
| `tier0-rt-ui` | pass |

X2 added: `multiManifestDiff.test.ts`, `MultiManifestDiffTable.test.tsx`, `reviewPacketExport.test.ts`, P2 isolation module list.

| Finding ID | Verdict |
|------------|---------|
| C3-DEBT-TEST-01 | Pass — test surface grew with X2; no P0 gap |

---

## 6. Tier summary

| Tier | Items |
|------|-------|
| **P0 urgent** | **None** |
| **P1 later** | `App.tsx` decomposition; further `experiment/` panel splits; bridge deny-path pytest; Cesium chunking; optional F8 handoff doc refresh |
| **Acceptable residual** | Session manager facade; dual advisory derive; M3 localStorage prefs; R1-LIFE-04; F7 queue modules |

| Finding ID | Verdict |
|------------|---------|
| C3-DEBT-TIER-01 | Pass — no P0 blockers at C3 freeze |

---

## Related

- [rt_c3_platform_consolidation_review_r1.md](rt_c3_platform_consolidation_review_r1.md)
- [rt_roadmap_next_frontiers_v7.md](rt_roadmap_next_frontiers_v7.md)
