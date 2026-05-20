# SA-R0 Replay Platform Implementation Plan

Phase name: **SA-R0 Replay Platform**

Build recommendation: **scoped implementation** on the platform frontier only. Does not authorize runtime, launch, topic, schema, parser-contract, HITL, or live ROS coupling.

`AGENTS.md` remains primary authority. Frozen replay tooling contracts are not modified by this wave.

## Purpose

Deliver a **read-only, replay-derived** interactive visualization platform for simulation replay analysis and research presentation:

- Python packager: `replay_sa_bundle_v1`
- Browser viewer: `platform/sa-r0-viewer/` (React + TypeScript + Cesium)

## Allowed scope

- Evaluation-side `scripts/evaluation/replay_sa_bundle.py` and `replay_sa_geometry.py`
- Additive `replay_sa_bundle_v1` artifact (not parser-visible)
- React viewer under `platform/sa-r0-viewer/` (isolated from legacy `web/` rosbridge)
- Committed demo bundle under `fixtures/sa_r0/`
- Tests: `test_replay_sa_bundle.py`, viewer unit tests
- Documentation: schema doc, freeze audit, registry row `PLAT-SA-R0`
- Optional `scripts/ci_eval.sh tier0-sa-r0` gate (viewer build + bundle tests)

## Forbidden scope

- Live ROS/rosbridge/WebSocket subscriptions
- Runtime, launch, topic, schema, parser-contract changes
- HITL, engage/disengage, operator approval, readiness scoring
- Extending legacy `web/` without separate audit
- Tactical C2 / battle-management UX
- Merging replay + live into one picture
- Cesium as operational geography (display-only fictional georef required)

## Cesium display exception (relative to PLAN-SA-R1)

PLAN-SA-R1 defaulted to 2D scenario ENU. SA-R0 uses Cesium as a **visualization canvas** only:

- Authoritative geometry remains `scenario_enu` in the bundle
- Fictional `georef_display.anchor` for map projection
- Mandatory banner: scenario-local — not deployed geography
- Dashed/sparse track semantics preserved

## Data flow

```
log + meta → replay_observability → replay_narrative → replay_static_visualization
  → replay_sa_bundle.py pack → index.json (+ assets)
  → platform/sa-r0-viewer (static load)
```

## Deliverables (PHASE A)

| ID | Content |
|----|---------|
| A1 | Cesium map, static entities, zones, layer toggles |
| A2 | Timeline scrub, play/pause, highlight, camera jump |
| A3 | Synchronized mock panes (radar, EO/IR, onboard, telemetry, threat) |
| A4 | Narrative timeline, bookmarks, annotations |
| A5 | Portable bundle export, metadata panel, demo loading |

## Implementation waves

See [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md) and freeze audit after regression.

## Status

Implementation authorized on platform frontier. Freeze recorded in [sa_r0_freeze_audit.md](sa_r0_freeze_audit.md).

## Motion + readability refinement (viewer-only)

Narrow follow-up under `platform/sa-r0-viewer/`:

- Sparse-sample interpolation on master clock (`trackPlayback.ts`)
- Trail + faint future path + launch-base segment on Cesium map
- Zone labels and softer fills (`zoneStyles.ts`, optional `display_label` in packager)
- Timeline-aware onboard/radar/EOIR mock panes (`onboardPhase.ts`)
- Fit replay / reset view toolbar on strategic map

Does not change parser contracts or runtime behavior.
