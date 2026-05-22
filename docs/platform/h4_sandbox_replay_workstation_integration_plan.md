# PHASE H4 — Sandbox Replay Workstation Integration (PLAT-SA-H4)

**Status:** implementation wave  
**Depends on:** PLAN-SA-H1 (frozen), PLAT-SA-H2 (frozen), PLAT-SA-H3 (frozen)

## Purpose

Unify the SA-R0 viewer into a coherent **replay-first experimentation workstation**: scenario discovery → orchestration lineage mirrors → replay review → compare → corpus evolution → report/presentation — with workflow continuity and reviewer-oriented navigation only.

## Identity (unchanged)

- Governance-aware replay experimentation
- Deterministic offline orchestration mirrors (explanatory)
- Comparative topology analysis
- Corpus/research operations
- No operational dashboard, HITL, live ROS, or browser execution authority

## Allowed

- Integrated workflow UX and segment-complete views
- Experiment lineage navigation (`workflow.lineage`)
- Central `experimentNavigation` deep links (SPA, read-only)
- Orchestration-aware replay navigation (mirror → bundle/corpus/compare)
- Compare/report workspace views
- Shared experiment discover rail
- Experiment review rail for single-demo replay
- URL param coherence (`orchestration_queue`, `corpus_entry`, `demo`, `pair`)
- Panel registry enforcement helper (`useSegmentPanels`)
- Governance chrome experiment context strip

## Forbidden

- Live execution, runtime streaming, rosbridge/WebSocket
- Browser-triggered simulation or `run_experiment_queue.py`
- HITL/C2/tactical/readiness/deployment semantics
- ML recommendations or ranking UX
- Parser/topic/schema changes
- New required bundle fields
- Async workers, default CI Gazebo capture (H3 §5 deferred)
- Inline scenario YAML authoring

## Deliverables

| ID | Artifact |
|----|----------|
| D0 | This plan, `experiment_workflow_continuity_v1.md`, freeze audit |
| D1 | `experimentNavigation.ts` + Vitest |
| D2 | `ExperimentLineagePanel` |
| D3 | `CompareWorkspaceView`, `ReportWorkspaceView`, scenario handoff |
| D4 | `ExperimentDiscoverRail`, `ExperimentReviewRail` |
| D5 | Sweep walkthrough steps, corpus/compare wiring, segment store policy |
| D6 | Governance chrome context strip |
| D7 | `useSegmentPanels` |

## H5 boundary

- H3 execution lane (async workers, CI capture automation)
- Concept B focus/drawer layout
- Authoring workstation
- Browser orchestration triggers
- PLAN-VIZ-R2

*End of PLAT-SA-H4 plan.*
