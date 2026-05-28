# RT-X3 P0 — Freeze Audit (PLAT-RT-X3 P0)

**Phase:** PLAT-RT-X3 P0 — workbench v3 shell ergonomics  
**Status:** frozen

**Plan:** [rt_plat_x3_p0_workbench_v3_shell_plan.md](../platform/rt_plat_x3_p0_workbench_v3_shell_plan.md)

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | V3 navigation state + banner | `workbenchV2State.ts`, `BANNER_EXPERIMENT_V3` |
| 2 | Program context + roster + secondary picker | `ExperimentProgramContextStrip.tsx`, `ExperimentManifestRoster.tsx`, `ExperimentSecondaryManifestPicker.tsx` |
| 3 | V3 shell integration | `ExperimentWorkbenchV3Shell.tsx`, `ExperimentWorkbenchV2Shell.tsx` |
| 4 | Packet sections UI + schema parse | `reviewPacketSections.ts`, optional `sections[]` on `reviewPacketSchema` |
| 5 | Compare coach + status vocabulary | `compareModeCoach.ts`, `compareStatusVocabulary.ts`, `MultiManifestDiffTable.tsx` |
| 6 | Vitest + isolation | 95 files, 364 tests; `isolation.test.ts` X3 P0 block |
| 7 | Reviews + registry | This audit + architecture/governance/experiment reviews |

No changes under `platform/rt-sandbox-bridge/`, `platform/sa-r0-viewer/`, `src/counter_uas/`, or `App.tsx`.

## P0 architecture summary

V3 wraps frozen v2 layout with read-only navigation: cohort label and tag filter, breadcrumb focus, manifest roster with ref status chips, explicit secondary manifest select, compare mode coach, multi-manifest status column, and packet section cards in the report dock preview tab. Export/copy review packets remain identical to pre-P0 (no `sections[]` in exported JSON). Drill-down buttons set primary/secondary refs only — no manifest merge or cross-manifest run pairing.

## Boundary guarantees

- No bridge endpoints, telemetry, subcommands, parser, topic, or runtime changes
- No SA viewer or automatic import
- No derive math changes (`metricsDerive`, `analyticsDerive`, etc. untouched)
- Import guards unchanged; `buildReviewPacketPreview` output shape unchanged for export

## Validation evidence

| Suite | Result |
|-------|--------|
| Full Vitest | 95 files, 364 passed |
| `npm run build` | pass; JS 547.88 kB / gzip 150.34 kB; chunk-size warning (pre-existing) |
| `scripts/ci_eval.sh tier0-rt-ui` | OK |
| `lint_rt_runtime_subcommands.py --check` | OK (7 subcommands) |
| `test_rt_sandbox_bridge.py -q` | 151 passed, 2 failed (pre-existing SA string-scan in guard modules) |
| `git diff --check` | OK |

The two bridge pytest failures are `test_rt_sandbox_ui_isolation` and `test_rt_sandbox_ui_world_editing_commands` literal scans for `platform/sa-r0-viewer` in unrelated guard modules; no bridge or SA paths changed in P0.

| Diff guard bridge / SA / `src/counter_uas` | no matches |

## Stop line

PLAT-RT-X3 P0 frozen. Do not start **PLAT-RT-X3 P1** without scoped plan, governance review, contamination review, and freeze audit.
