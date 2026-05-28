# RT - Post-V4 Technical Debt Audit R1

**Phase:** Post-V4 checkpoint review
**Status:** frozen audit

## Debt Summary

| Debt | Evidence | Risk | Future action |
|------|----------|------|---------------|
| `App.tsx` orchestration concentration | 777 lines; owns session, editing, layer memory, tactical, handoff, workbench, layout props | Medium | Extract derived state/composition hook after checkpoint |
| `ExperimentWorkbenchPanel` concentration | 831 lines; many state groups, imports, effects, metrics, review lane, prompts | Medium-High | Split into manifest controls, import controller, metrics container, review container |
| Compare/cognition duplication | Multiple strips and compare panels repeat role/caveat patterns | Medium | Shared display primitives/vocabulary helpers |
| Density wording duplication | Toggle rail and hub both present budget/density lines | Low-Med | Keep `densityBudgetSummary` as wording source |
| Prompt-based import flows | Manifest/spec/metrics/fidelity imports repeat parse/error patterns | Low-Med | Future `useJsonPromptImport` style helper |
| Bundle size | Vite warning, 533.22 kB minified JS at latest V4 P2 build | Medium | Consider code splitting for experiment/Cesium-heavy panels |

## Non-Debt / Healthy Areas

- Bridge/runtime contracts remain stable.
- V4 registry and overlay modules are separable.
- `RuntimeWorkstationShell` is layout-only and small.
- Experiment submodules have good focused coverage despite parent concentration.
- Validation confidence is acceptable for local maintainer workstation use.

## Cleanup Priority

1. **Checkpoint cleanup:** lowest governance cost, highest editability value.
2. **Experiment parent split before X3:** prevents X3 from compounding concentration.
3. **Shared cognition primitives:** useful after patterns settle.
4. **Bundle/code splitting:** defer until performance pain appears outside build warnings.

No technical debt cleanup is implemented or authorized by this audit.
