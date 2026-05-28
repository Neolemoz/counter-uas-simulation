# RT-V4 P1 - Freeze Audit (PLAT-RT-V4 P1)

**Phase:** PLAT-RT-V4 P1 - visibility overlay and terrain cognition refinement
**Status:** frozen

**Plan:** [rt_plat_v4_p1_visibility_overlay_plan.md](../platform/rt_plat_v4_p1_visibility_overlay_plan.md)

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | V4 P1 registry rows | `visualLayerRegistry.ts` |
| 2 | Visibility / terrain overlay module | `visibilityOverlayV4.ts` |
| 3 | Cesium sync wiring | `CesiumRuntimeView.tsx` |
| 4 | Visibility cognition refinement | `visibilityCognition.ts`, `VisibilityCognitionStrip.tsx`, `RuntimeCognitionHub.tsx` |
| 5 | Compare emphasis cognition | `sessionComparisonCognition.ts`, `SessionComparisonCognitionStrip.tsx` |
| 6 | Vitest coverage | `visualLayerRegistry.test.ts`, `visibilityOverlayV4.test.ts`, `sessionComparisonCognition.test.ts` |
| 7 | Architecture/governance/realism reviews | P1 review docs |
| 8 | Registry + AGENTS + roadmap | Yes |

No changes under `platform/rt-sandbox-bridge/` or `platform/sa-r0-viewer/`.

## P1 architecture summary

PLAT-RT-V4 P1 adds default-off visibility corridor, occlusion band, terrain relation label, and compare emphasis display layers. The Cesium overlay module creates decorative entities only and the workstation cognition surfaces describe each cue as heuristic/explanatory. P2 multi-session comparison layout/geometry work is not implemented.

## Boundary guarantees

- Entity registry and bridge handlers remain command authority.
- No bridge endpoints, telemetry channels, subcommands, parser, topic, or schema changes.
- No SA viewer changes and no import/federation behavior.
- No browser->ROS authority.
- No tactical redesign.
- No P2 layout or X3 work.

## Validation evidence

| Suite | Result |
|-------|--------|
| `npm test` | 91 files, 343 passed |
| `npm run build` | pass; JS 531.23 kB / gzip 146.25 kB; existing chunk-size warning |
| `scripts/ci_eval.sh tier0-rt-ui` | OK |
| `lint_rt_runtime_subcommands.py --check` | OK (7 subcommands) |
| `test_rt_sandbox_bridge.py -q` | 151 passed, 2 failed pre-existing SA string-scan failures |
| `git diff --check` | pass |

The two bridge pytest failures are the previously documented `test_rt_sandbox_ui_isolation` and `test_rt_sandbox_ui_world_editing_commands` literal scans for `platform/sa-r0-viewer`; P1 changed no bridge or SA viewer paths.

## Stop line

PLAT-RT-V4 P1 frozen. Do not start **PLAT-RT-V4 P2**, **PLAN-RT-X3**, checkpoint work, bridge work, SA viewer work, import automation, federation, or distributed runtime without a scoped plan, governance review, validation, and freeze audit.
