# RT-C4 — Architecture Review R1

**Phase:** PLAN-RT-C4 — post-V4 checkpoint cleanup planning (read-only)  
**Prerequisite:** CHECKPOINT-RT-POST-V4 frozen; PLAT-RT-V4 complete  
**Plan:** [rt_c4_checkpoint_cleanup_plan.md](../platform/rt_c4_checkpoint_cleanup_plan.md)  
**Governance review:** [rt_c4_governance_review_r1.md](rt_c4_governance_review_r1.md)  
**Technical debt:** [rt_c4_technical_debt_audit_r1.md](rt_c4_technical_debt_audit_r1.md)  
**Next frontiers:** [rt_roadmap_next_frontiers_v11.md](rt_roadmap_next_frontiers_v11.md)  
**Freeze audit:** [rt_c4_freeze_audit.md](rt_c4_freeze_audit.md)  
**Baseline:** [rt_checkpoint_post_v4_architecture_review_r1.md](rt_checkpoint_post_v4_architecture_review_r1.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Platform stability | **Pass** — bridge/runtime authority unchanged; post-V4 checkpoint confirmed |
| UI layering | **Pass-with-conditions** — shell healthy; parent panels concentrated |
| Cleanup feasibility | **Pass** — behavior-neutral extractions are architecturally sound |
| X3 readiness | **Conditional** — large X3 PLAT work should follow experiment parent split |

**Architecture verdict:** The RT platform remains coherent. Dominant risk is **UI orchestration concentration**, not bridge/runtime instability. PLAN-RT-C4 documents a safe decomposition sequence; no cleanup is authorized until PLAT-RT-C4 phases freeze individually.

---

## Findings

| ID | Area | Finding | Severity |
|----|------|---------|----------|
| C4-ARCH-01 | `App.tsx` (777 LOC) | Centralizes session selection, per-session edit mirror, layer memory, tactical panels, experiment chrome, handoff/workbench composition, shell wiring | Medium |
| C4-ARCH-02 | `ExperimentWorkbenchPanel.tsx` (831 LOC) | Combines manifest persistence, prompt imports, analytics, F5/F5b metrics, v2 shell host, compare, advisory rollup | Medium–High |
| C4-ARCH-03 | `RuntimeWorkstationShell` (80 LOC) | Layout-only; correct boundary after V4 P2 | Low (healthy) |
| C4-ARCH-04 | V4 modules | Registry, overlays, rail, cognition reasonably separated | Low (healthy) |
| C4-ARCH-05 | `useExperimentWorkbenchV2` | Partial v2 decoupling — preserve; do not re-merge | Low (healthy) |
| C4-ARCH-06 | `useRtSessionWorkspace` | Session/telemetry ownership — must remain authority-adjacent | Low (do not split authority) |

---

## App.tsx — orchestration and boundaries

### Responsibilities today

| Layer | Components / hooks | Boundary rule |
|-------|-------------------|-----------------|
| Session transport | `useRtSessionWorkspace`, tab order, display names | Bridge pull truth; no relocation of HTTP calls into presentational children |
| Entity editing | `editBySession`, `runEntityCommand`, spawn/move/delete | Optimistic mirror; commands via `entityCommands` |
| Visualization chrome | `layerVisibility`, `sessionLayerVisibilityStore` | Explanatory/display; not parser truth |
| Experiment chrome | Mode toggles, `experimentRollup` callback | Derived advisory rollup to handoff panel |
| Tactical | `useTacticalState`, manual/assisted/autonomous panels | Sandbox tactical only; not HITL/C2 |
| Composition | `RuntimeWorkstationShell` slots | Props-only assembly |

### Recommended extractions (PLAT-RT-C4 P2)

1. **`useSessionEntityEditing`** — edit state map, `runEntityCommand`, entity handlers, derived `entities` / `mergedWorldSummary`.
2. **`AppWorkstationSlots`** (optional) — maps hook outputs to shell slot props without new semantics.
3. **`useSessionLayerVisibility`** (optional) — layer memory + persist on tab switch.
4. **Keep in thin `App`:** `useRtSessionWorkspace`, governance chrome, experiment chrome state, shell render.

### Sequencing rationale

App decomposition runs **after** experiment parent split (P0–P1) because `App` passes many props into `ExperimentWorkbenchPanel` and `CaptureHandoffWorkflowPanel`; stabilizing experiment composition reduces P2 integration risk.

---

## ExperimentWorkbenchPanel — X2 and grouping

### Preserve (no architectural regression)

- `useExperimentWorkbenchV2` for review lane, dock, compare-mode activation.
- `ExperimentWorkbenchV2Shell` as v2 host — do not fold back into parent logic.

### Recommended extractions

| Container / helper | Scope | Phase |
|--------------------|-------|-------|
| `useJsonPromptImport` | Shared prompt → parse → error surface | P0 |
| `ExperimentManifestToolbar` | experiment_id, pin, import/export buttons | P0 |
| `ExperimentCompareSection` | compare A/B selectors + `ExperimentComparePanel` | P1 |
| `ExperimentF5MetricsSection` | F5 block including fidelity compare, matrix, extended compare, advisory strips | P1 |

### Advisory plane

Rollup `useEffect` and `deriveAdvisoryForRow` integration should **remain in parent** until a dedicated contamination-reviewed PLAT sub-phase — not P0/P1 default.

---

## Layer boundary assessment

| Layer | Post-V4 status | C4 cleanup impact |
|-------|----------------|-------------------|
| Runtime / bridge | Stable | None if PLAT stays UI-local |
| Replay / SA | Manual handoff only | None |
| Advisory | F8 advisory ≠ authority | Avoid rollup semantic moves in early PLAT |
| Experiment | Derived artifacts | Container splits only |
| Visualization | V4 display-only | No Cesium module moves in C4 |

---

## Architecture assessment

The platform can sustain small additive patches. Large **PLAN-RT-X3** implementation without prior cleanup would likely compound concentration in `ExperimentWorkbenchPanel`. PLAT-RT-C4 P0→P2 reduces that risk while preserving the four-layer model and frozen shell boundary.

**Stop line:** No PLAT-RT-C4 implementation without per-phase plan + governance + freeze.
