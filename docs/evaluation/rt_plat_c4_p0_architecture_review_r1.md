# RT-C4 P0 — Architecture Review R1 (PLAT-RT-C4 P0)

**Phase:** PLAT-RT-C4 P0 — experiment workbench cleanup  
**Plan:** [rt_plat_c4_p0_experiment_workbench_cleanup_plan.md](../platform/rt_plat_c4_p0_experiment_workbench_cleanup_plan.md)  
**Baseline:** [rt_c4_architecture_review_r1.md](rt_c4_architecture_review_r1.md)  
**Status:** pass

## Verdict

P0 reduces UI concentration in `ExperimentWorkbenchPanel` via presentational extraction and a shared prompt-import helper. No layer boundary regression.

## Concentration relief

| Surface | Before P0 | After P0 |
|---------|-----------|----------|
| `ExperimentWorkbenchPanel.tsx` | ~831 LOC | ~780 LOC (toolbar + inline prompts removed) |
| `useJsonPromptImport.ts` | — | Shared prompt → parse → alert → onSuccess |
| `ExperimentManifestToolbar.tsx` | — | experiment_id, pin, manifest/spec import/export, mode toggles |

## Preserved architecture

- `useExperimentWorkbenchV2` remains the v2 review-lane owner; not re-merged into parent.
- `ExperimentWorkbenchV2Shell` unchanged as v2 host.
- Advisory rollup `useEffect` stays in parent (contamination-sensitive).
- Compare and F5 blocks remain inline in parent until P1 section extractions.

## Layer assessment

| Layer | Impact |
|-------|--------|
| Runtime / bridge | None |
| Replay / SA | None |
| Advisory | Rollup logic untouched |
| Experiment | Same derive/import paths; UI-only refactor |
| Visualization | None |

## Stop line

No P1 compare/F5 section splits or `App.tsx` decomposition in this wave.
