# RT-C4 P0 — Freeze Audit (PLAT-RT-C4 P0)

**Phase:** PLAT-RT-C4 P0 — experiment import helper + manifest toolbar extraction  
**Status:** frozen

**Plan:** [rt_plat_c4_p0_experiment_workbench_cleanup_plan.md](../platform/rt_plat_c4_p0_experiment_workbench_cleanup_plan.md)

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | `useJsonPromptImport` + `promptJsonImport` | `platform/rt-sandbox-ui/src/experiment/useJsonPromptImport.ts` |
| 2 | `ExperimentManifestToolbar` | `platform/rt-sandbox-ui/src/experiment/ExperimentManifestToolbar.tsx` |
| 3 | `ExperimentWorkbenchPanel` refactor | Same path; ~831 → ~780 LOC |
| 4 | Vitest coverage | `useJsonPromptImport.test.ts` (5 tests) |
| 5 | Isolation module presence | `isolation.test.ts` |
| 6 | Architecture review | [rt_plat_c4_p0_architecture_review_r1.md](rt_plat_c4_p0_architecture_review_r1.md) |
| 7 | Governance review | [rt_plat_c4_p0_governance_review_r1.md](rt_plat_c4_p0_governance_review_r1.md) |
| 8 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/sa-r0-viewer/`, or `App.tsx`.

## P0 architecture summary

Shared JSON prompt import consolidates `window.prompt` → `safeParse*` → `formatImportError` alert → `onSuccess` for manifest, spec, metrics, and fidelity imports in the workbench parent. Manifest toolbar is a presentational extraction of experiment_id, pin, manifest/spec import/export, and mode toggles. `useExperimentWorkbenchV2`, advisory rollup, compare block, and F5 section remain in the parent.

## Boundary guarantees

- No bridge endpoints, telemetry channels, subcommands, parser, topic, or schema changes
- No SA viewer changes and no import/federation behavior
- No browser→ROS authority
- Import guards (`experimentImportGuards.ts`) unchanged
- Advisory rollup `useEffect` not relocated
- No P1 compare/F5 section extractions

## Validation evidence

| Suite | Result |
|-------|--------|
| Full Vitest | 92 files, 351 passed |
| `npm run build` | pass; JS 533.87 kB / gzip 146.92 kB; existing chunk-size warning |
| `scripts/ci_eval.sh tier0-rt-ui` | OK |
| `lint_rt_runtime_subcommands.py --check` | OK (7 subcommands) |
| `test_rt_sandbox_bridge.py -q` | 151 passed, 2 failed (pre-existing SA string-scan failures) |
| `git diff --check` | OK |

The two bridge pytest failures are the previously documented `test_rt_sandbox_ui_isolation` and `test_rt_sandbox_ui_world_editing_commands` literal scans for `platform/sa-r0-viewer` in unrelated guard modules; no bridge or SA paths changed in P0.

| Diff guard `App.tsx` / bridge / SA | no matches |

## Stop line

PLAT-RT-C4 P0 frozen. Do not start **PLAT-RT-C4 P1** (`ExperimentCompareSection`, `ExperimentF5MetricsSection`), **PLAT-RT-C4 P2** (`App.tsx` hooks), or **PLAN-RT-X3** without scoped plan + governance + freeze audit.
