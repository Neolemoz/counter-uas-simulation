# RT-C4 — Post-V4 Checkpoint Cleanup Planning (PLAN-RT-C4)

**Phase:** PLAN-RT-C4 — post-V4 checkpoint cleanup planning (docs only)  
**Prerequisite:** CHECKPOINT-RT-POST-V4 frozen; PLAT-RT-F8 P0–P2 frozen; PLAT-RT-V4 P0–P2 frozen; PLAT-RT-X2 P0–P2 frozen  
**Baseline:** [rt_checkpoint_post_v4_review.md](rt_checkpoint_post_v4_review.md), [rt_checkpoint_post_v4_freeze_audit.md](../evaluation/rt_checkpoint_post_v4_freeze_audit.md), [rt_roadmap_next_frontiers_v10.md](../evaluation/rt_roadmap_next_frontiers_v10.md)  
**Authority:** [AGENTS.md](../../AGENTS.md)

## Vocabulary (critical)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-C4** (this wave) | Post-V4 **cleanup planning** — documentation only; defines future PLAT scope |
| **PLAT-RT-C4** | Future UI refactor waves (P0–P2) — **not authorized** until per-phase PLAT plan + governance + freeze |
| **CHECKPOINT-RT-POST-V4** | Review-only concentration/duplication checkpoint — **not** cleanup implementation |
| **PLAN-RT-C1/C2/C3** | Holistic consolidation checkpoints — **not** C4 cleanup planning |
| **PLAN-RT-X3** | Docs-first experiment follow-on — **not** C4 |
| **Distributed multi-bridge** | Explicit **non-frontier** |

New artifacts use `rt_c4_*` filenames to avoid collision with C1–C3 consolidation IDs and F-wave IDs.

## Goal

Turn the frozen post-V4 checkpoint into a **scoped, governance-safe UI concentration reduction roadmap**. Reduce edit concentration in `App.tsx` and `ExperimentWorkbenchPanel.tsx` without changing runtime behavior, bridge authority, or SA/import boundaries. Produce architecture/governance/debt reviews, next-frontier roadmap v11, and freeze — **without** implementing cleanup.

## Allowed

- Cleanup plan, architecture review, governance review, technical debt audit, next-frontier roadmap v11, freeze audit
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md)
- Regression evidence citations from post-V4 / PLAT-RT-V4 P2 baseline (no new tests in PLAN-RT-C4)

## Forbidden

- Changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, `src/counter_uas/`
- Authorization of **PLAT-RT-C4** implementation by this plan alone
- Bridge command/telemetry expansion, parser/topic/schema changes
- SA viewer edits, auto-import, corpus writes, federation, distributed multi-bridge
- **PLAN-RT-X3** implementation, tactical redesign, import semantic changes
- Bundle/code-splitting work (deferred; documented as residual debt only)

## Evidence baseline

| Surface | LOC | Role |
|---------|-----|------|
| `platform/rt-sandbox-ui/src/App.tsx` | 777 | Session workspace orchestration, edit mirror, layer memory, tactical wiring, experiment chrome, shell assembly |
| `platform/rt-sandbox-ui/src/experiment/ExperimentWorkbenchPanel.tsx` | 831 | Manifest + X1 panels + X2 shell host + F5/F5b metrics + compare + advisory rollup |
| `platform/rt-sandbox-ui/src/workstation/RuntimeWorkstationShell.tsx` | 80 | Layout-only — **preserve boundary** |
| `platform/rt-sandbox-ui/src/experiment/useExperimentWorkbenchV2.ts` | ~234 | v2 review lane / dock orchestration (X2 P2 extraction) |
| `platform/rt-sandbox-ui/src/hooks/useRtSessionWorkspace.ts` | ~372 | Session/telemetry ownership — **do not relocate bridge authority** |

Regression baseline (post-V4 checkpoint): `npm test` 91 files / 346 passed; build OK (Vite chunk warning); `tier0-rt-ui` OK; subcommand lint OK; bridge pytest 151 passed + 2 pre-existing SA string-scan failures.

## Workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | `App.tsx` concentration | Extraction candidates, state ownership, shell boundaries |
| 2 | `ExperimentWorkbenchPanel` concentration | X2 hook preservation, container splits, staged path |
| 3 | Duplication inventory | Merge later / leave as-is / risky |
| 4 | Governance impact | Additive cleanup, no authority drift, contamination gates |
| 5 | Roadmap v11 | PLAT-RT-C4 vs PLAN-RT-X3 vs plateau |

---

## 1. App.tsx concentration plan

### Orchestration responsibilities

| Concern | Owner today | Extraction candidate | PLAT phase |
|---------|-------------|---------------------|------------|
| Multi-session / pull / tabs | `useRtSessionWorkspace`, `useSessionTabOrder`, `useSessionDisplayNames` | **Leave** — already decomposed | — |
| Per-session edit mirror + commands | `editBySession`, `runEntityCommand`, spawn/move/delete handlers | `useSessionEntityEditing` hook | **P2** |
| Layer visibility memory | `layerVisibility`, `sessionLayerVisibilityStore` | Optional `useSessionLayerVisibility` | **P2** (optional) |
| Experiment mode chrome | compare/analytics/continuity/F5 toggles, `experimentRollup` | `useExperimentPanelChrome` or pass-through only | **P2** (optional) |
| Tactical panels | `useTacticalState` + mirrors column JSX | Thin `TacticalMirrorsColumn` (presentation) | **P2** (optional) |
| Shell assembly | `RuntimeWorkstationShell` slot props | `AppWorkstationSlots` presentational wrapper | **P2** |

### State ownership rules (must preserve in PLAT waves)

- Bridge `session_id` and entity commands remain authoritative via existing bridge modules.
- Local entity mirror stays optimistic overlay; M3 tab-switch confirm unchanged.
- `experimentRollup` to `CaptureHandoffWorkflowPanel` stays derived advisory — not command or parser truth.
- `RuntimeWorkstationShell` stays layout-only; no new business logic in shell.

### UI shell boundaries

- **Header / session rail / workflow strip:** session lifecycle cognition — keep wired from `useRtSessionWorkspace`.
- **Cognition column:** `RuntimeCognitionHub` — V4 density/registry inputs only; no experiment logic inside hub.
- **World / viz columns:** editing + Cesium — share handlers from entity-editing hook after P2.
- **Mirrors column:** tactical + telemetry mirrors — presentation grouping only.
- **Pipeline footer:** handoff + experiment — experiment chrome state may lift to optional hook in P2.

### Sequencing

`App.tsx` decomposition is **last** (PLAT-RT-C4 P2) after experiment parent stabilizes (P0–P1) to avoid regressions in handoff rollup and workbench props.

---

## 2. ExperimentWorkbenchPanel concentration plan

### X2 orchestration (do not regress)

`useExperimentWorkbenchV2` owns:

- v2 state persistence (`workbenchV2State`)
- review-step → panel activation (`reviewLaneOrchestration`)
- report dock import/export
- compare-mode sync with legacy compare toggles

**Do not re-merge** v2 logic into the parent panel.

### Remaining concentration blocks

| Block | Approx lines | Future module | PLAT phase |
|-------|--------------|---------------|------------|
| Manifest toolbar + JSON prompts | 391–604 | `ExperimentManifestToolbar`, `useJsonPromptImport` | **P0** |
| Compare A/B + `ExperimentComparePanel` | 652–689 | `ExperimentCompareSection` | **P1** |
| F5 metrics / fidelity / matrix / extended compare / advisory strips | 718–822 | `ExperimentF5MetricsSection` | **P1** |
| v2 shell host | 520–533 | **No split** | — |

### Grouping boundaries

| Plane | Contents | Touch policy |
|-------|----------|--------------|
| Import/export | `window.prompt` + `experimentImportGuards` | P0 — behavior-neutral helper only |
| Derived reports | `analyticsDerive`, `metricsDerive`, `fidelityMetricsDerive` | Read-only; move with containers |
| Advisory | `deriveAdvisoryForRow`, rollup `useEffect` | **Risky** — contamination review if moved |

### Staged cleanup path

1. **P0:** Shared import helper + manifest toolbar extraction (no advisory rollup moves).
2. **P1:** Compare section + F5 metrics section (parent becomes composition root).
3. **Defer:** Cognition/compare vocabulary consolidation until after P2 App hook (optional maintenance).

---

## 3. Duplication inventory

| Pattern | Locations | Category | PLAT notes |
|---------|-----------|----------|------------|
| Cognition strips | `EditingCognitionStrip`, `VisibilityCognitionStrip`, `TelemetryCognitionStrip`, `TerrainCognitionStrip`, `FidelityTruthCognitionStrip`, `CesiumEditingCognitionStrip`, `SessionComparisonCognitionStrip` | **Merge later** | Shared `CognitionStripShell` after P2 |
| Compare display | `ExperimentComparePanel`, `TelemetryCompareStrip`, `TacticalAnnexCompareStrip`, `ExperimentFidelityCompareStrip`, `SessionComparisonCognitionStrip` | **Merge later** | `compareStatusVocabulary.ts` — labels only |
| Density summaries | `densityBudgetSummary` in `visualLayerRegistry.ts` → `VisualLayerToggleRail`, `RuntimeCognitionHub` | **Leave as-is** | Single helper; no duplicate wording |
| Import prompts | Workbench, report dock, analytics, continuity, cohort navigator, sweep browser | **Merge later** | `useJsonPromptImport` in P0 |
| Toggle/button styling | Layer rail, camera, mode checkboxes | **Merge later** | Low priority cosmetic |
| Dual advisory derive | TS + Python | **Leave as-is** | Out of UI cleanup scope |
| Advisory rollup in workbench parent | `ExperimentWorkbenchPanel` | **Risky / avoid** early | Handoff contamination gate |

---

## 4. Future PLAT-RT-C4 phased roadmap (planning only)

| Phase | Scope | Risk | Governance |
|-------|--------|------|------------|
| **PLAT-RT-C4 P0** | Experiment import helper + `ExperimentManifestToolbar`; no `App.tsx` | Low | Standard PLAT freeze |
| **PLAT-RT-C4 P1** | `ExperimentCompareSection`, `ExperimentF5MetricsSection`; keep v2 hook | Low–Med | Standard PLAT freeze |
| **PLAT-RT-C4 P2** | `useSessionEntityEditing`, optional `AppWorkstationSlots`; thin `App` | Med | Standard + handoff adjacency check |

**Explicit non-goals:** bundle splitting, bridge changes, advisory semantic changes, handoff automation, Cesium module moves, `RuntimeWorkstationShell` growth.

Each PLAT phase requires: scoped plan in `docs/platform/`, governance review, `tier0-rt-ui` + targeted Vitest, contamination review if advisory rollup touched, freeze audit, registry + AGENTS update.

---

## 5. Governance impact (summary)

Behavior-neutral UI extraction is compatible with additive-only evolution when:

- No bridge commands, telemetry channels, or import-guard validation rules change.
- No new “ready/automatic” language; advisory rollup unchanged in meaning.
- No `platform/sa-r0-viewer/` edits or corpus writes.

Full matrix: [rt_c4_governance_review_r1.md](../evaluation/rt_c4_governance_review_r1.md).

---

## Deliverables

| Artifact | Path |
|----------|------|
| Master cleanup plan | This document |
| Architecture review | [rt_c4_architecture_review_r1.md](../evaluation/rt_c4_architecture_review_r1.md) |
| Governance review | [rt_c4_governance_review_r1.md](../evaluation/rt_c4_governance_review_r1.md) |
| Technical debt audit | [rt_c4_technical_debt_audit_r1.md](../evaluation/rt_c4_technical_debt_audit_r1.md) |
| Next-frontier roadmap v11 | [rt_roadmap_next_frontiers_v11.md](../evaluation/rt_roadmap_next_frontiers_v11.md) |
| Freeze audit | [rt_c4_freeze_audit.md](../evaluation/rt_c4_freeze_audit.md) |

## Validation

Documentation-only diff hygiene: no changes under `platform/*` implementation trees or `src/counter_uas/`.

Evidence citation baseline (no re-run required for PLAN-RT-C4 freeze):

- PLAT-RT-V4 P2: `npm test` 346 passed, build OK, `tier0-rt-ui` OK
- Post-V4 checkpoint: bridge pytest 151 passed; 2 pre-existing SA string-scan failures

## Stop line

**PLAN-RT-C4** freezes cleanup planning. Do not start **PLAT-RT-C4**, **PLAN-RT-X3**, bridge work, SA viewer work, import automation, federation, or distributed runtime until:

1. An explicit PLAT-RT-C4 phase plan addresses a step from the phased roadmap above (or a ranked frontier from [rt_roadmap_next_frontiers_v11.md](../evaluation/rt_roadmap_next_frontiers_v11.md)), and  
2. That wave completes governance review (+ contamination review if advisory adjacency) + freeze audit.

**Recommended next (advisory):** PLAT-RT-C4 P0 after PLAN-RT-C4 freeze — not authorization by this document alone.

## Related

- [rt_checkpoint_post_v4_review.md](rt_checkpoint_post_v4_review.md)
- [rt_checkpoint_post_v4_architecture_review_r1.md](../evaluation/rt_checkpoint_post_v4_architecture_review_r1.md)
- [rt_checkpoint_post_v4_technical_debt_audit_r1.md](../evaluation/rt_checkpoint_post_v4_technical_debt_audit_r1.md)
- [rt_roadmap_next_frontiers_v10.md](../evaluation/rt_roadmap_next_frontiers_v10.md)
