# RT - Post-V4 Architecture Review R1

**Phase:** Post-V4 checkpoint review
**Status:** accepted for freeze

## Architecture Verdict

The RT platform remains coherent after F8 and V4. The dominant architectural risk is no longer bridge/runtime instability; it is UI orchestration concentration in `App.tsx` and `ExperimentWorkbenchPanel.tsx`.

## Findings

| Area | Finding | Severity |
|------|---------|----------|
| `App.tsx` | Centralizes session selection, edit state, layer memory, tactical panels, handoff/workbench composition, and shell wiring | Medium |
| `ExperimentWorkbenchPanel.tsx` | Combines manifest persistence, import/export, analytics, F5/F5b metrics, review lane, filters, and handoff advisory rollup | Medium-High |
| Workstation shell | Remains layout-only and healthy after V4 P2 | Low |
| Visualization V4 modules | Registry, overlay, rail, and cognition surfaces are reasonably separated | Low |
| Experiment submodules | Many focused submodules exist; parent composition is the main bottleneck | Medium |

## Architecture Assessment

The platform can sustain small additive patches, but large PLAN-RT-X3 implementation work would likely increase UI concentration unless preceded by cleanup. A future cleanup should be scoped to extracting orchestration hooks or container components without changing behavior.

## Recommended Future Cleanup Candidates

1. Extract an `useRtWorkstationComposition` or equivalent hook from `App.tsx` for derived props and panel composition state.
2. Split `ExperimentWorkbenchPanel` into import/controller, manifest controls, metrics area, and review-lane container components.
3. Consolidate repeated compare/cognition display helpers after usage stabilizes.
4. Keep `RuntimeWorkstationShell` layout-only.

No cleanup is authorized by this review.
