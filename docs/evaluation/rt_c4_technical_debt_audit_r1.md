# RT-C4 — Technical Debt Audit R1

**Phase:** PLAN-RT-C4 — post-V4 checkpoint cleanup planning  
**Plan:** [rt_c4_checkpoint_cleanup_plan.md](../platform/rt_c4_checkpoint_cleanup_plan.md)  
**Architecture review:** [rt_c4_architecture_review_r1.md](rt_c4_architecture_review_r1.md)  
**Baseline:** [rt_checkpoint_post_v4_technical_debt_audit_r1.md](rt_checkpoint_post_v4_technical_debt_audit_r1.md)

No runtime code was modified for this audit.

---

## 1. Closure matrix

### 1.1 Post-V4 checkpoint → C4 planning

| ID | Post-V4 status | C4 status | Notes |
|----|----------------|-----------|-------|
| POST-V4-DEBT-01 | `App.tsx` 777 LOC — watchlist | **PLAT P2 planned** | Documented extraction sequence |
| POST-V4-DEBT-02 | `ExperimentWorkbenchPanel` 831 LOC — primary debt | **PLAT P0–P1 planned** | Experiment parent first |
| POST-V4-DEBT-03 | Compare/cognition duplication | **Merge later** | After P0–P2 |
| POST-V4-DEBT-04 | Density wording duplication | **Leave as-is** | `densityBudgetSummary` is single source |
| POST-V4-DEBT-05 | Prompt import duplication | **PLAT P0** | `useJsonPromptImport` |
| POST-V4-DEBT-06 | Bundle size warning | **Deferred** | Not in PLAT-RT-C4 scope |

| Finding ID | Verdict |
|------------|---------|
| C4-DEBT-CLOSE-01 | Pass — post-V4 debt items mapped to PLAT phases |
| C4-DEBT-CLOSE-02 | Pass — no new P0 debt from PLAN-RT-C4 (docs only) |

### 1.2 C3 / X2 / V4 / F8 residuals (unchanged)

| Hotspot | Scale | C4 tier | Notes |
|---------|-------|---------|-------|
| `App.tsx` | 777 LOC | **P2** | Integration hub |
| `ExperimentWorkbenchPanel.tsx` | 831 LOC | **P0–P1** | Partial relief from `useExperimentWorkbenchV2` |
| `src/experiment/` | ~43 TSX modules in package | **P0–P1** | Parent concentration, not submodule quality |
| `src/handoff/` | ~30 files | **P1 later** | Touch only with contamination gate |
| `useRtSessionWorkspace.ts` | ~372 LOC | **Acceptable residual** | Do not split authority |
| Cesium bundle | ~533 KB JS minified (V4 P2 build) | **Deferred** | Code-splitting out of C4 |
| Bridge isolation pytest | 2 SA string-scan failures | **P1 later** | Pre-existing; deny-path literals |

---

## 2. Debt priorities (maintainer order)

1. **PLAT-RT-C4 P0** — experiment import helper + manifest toolbar (highest editability ROI, lowest risk).
2. **PLAT-RT-C4 P1** — compare + F5 metrics sections (reduces parent render tree).
3. **PLAT-RT-C4 P2** — `App.tsx` entity-editing hook + optional slot wrapper.
4. **Shared cognition/compare primitives** — after P2 optional maintenance.
5. **Bundle/code splitting** — defer until performance pain beyond build warning.

---

## 3. Duplication inventory

| Pattern | Locations | Category | Notes |
|---------|-----------|----------|-------|
| Cognition strips | `EditingCognitionStrip`, `VisibilityCognitionStrip`, `TelemetryCognitionStrip`, `TerrainCognitionStrip`, `FidelityTruthCognitionStrip`, `CesiumEditingCognitionStrip`, `SessionComparisonCognitionStrip` | **Merge later** | `CognitionStripShell` after PLAT-C4 P2 |
| Compare display | `ExperimentComparePanel`, `TelemetryCompareStrip`, `TacticalAnnexCompareStrip`, `ExperimentFidelityCompareStrip`, `SessionComparisonCognitionStrip` | **Merge later** | `compareStatusVocabulary.ts` — labels only |
| Density summaries | `densityBudgetSummary` → `VisualLayerToggleRail`, `RuntimeCognitionHub` | **Leave as-is** | Single helper; forbid duplicate wording in new UI |
| Import/export prompts | `ExperimentWorkbenchPanel`, `ExperimentReportDockPanel`, `ExperimentAnalyticsPanel`, `ExperimentContinuityReviewPanel`, `ExperimentCohortNavigator`, `SweepCatalogBrowser` | **Merge later** | `useJsonPromptImport` in P0 |
| Toggle/button styling | Layer rail, camera controls, mode checkboxes | **Merge later** | Low priority cosmetic |
| Dual advisory derive | TS + Python | **Leave as-is** | Not UI cleanup scope |
| Advisory rollup in workbench parent | `ExperimentWorkbenchPanel` | **Risky / avoid** early | Contamination if moved |

---

## 4. Non-debt / healthy areas

- Bridge/runtime contracts stable (post-V4 validation cited).
- V4 registry, overlay, rail modules separable.
- `RuntimeWorkstationShell` layout-only (~80 LOC).
- Experiment submodules have focused tests despite parent concentration.
- `useExperimentWorkbenchV2` closed X2 P2 decomposition debt partially.

---

## 5. PLAT authorization

No technical debt cleanup is **implemented** or **authorized** by PLAN-RT-C4. Each PLAT-RT-C4 phase requires its own plan, governance review, regression matrix, and freeze audit.

**Stop line:** See [rt_c4_freeze_audit.md](rt_c4_freeze_audit.md).
