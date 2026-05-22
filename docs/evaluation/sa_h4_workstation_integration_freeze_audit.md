# SA H4 — Sandbox Replay Workstation Integration Freeze Audit (PLAT-SA-H4)

## Scope

Viewer workflow integration per [h4_sandbox_replay_workstation_integration_plan.md](../platform/h4_sandbox_replay_workstation_integration_plan.md):

- `platform/sa-r0-viewer/src/navigation/experimentNavigation.ts` — SPA deep links (scenario, compare, corpus, sweep walkthrough)
- `platform/sa-r0-viewer/src/workflow/` — `ExperimentLineagePanel`, `ExperimentReviewRail`, `SweepWalkthroughNav`
- `platform/sa-r0-viewer/src/workspace/views/CompareWorkspaceView.tsx`, `ReportWorkspaceView.tsx`
- Shared `ExperimentDiscoverRail`, segment store snapshot on mode transitions
- [experiment_workflow_continuity_v1.md](experiment_workflow_continuity_v1.md)

No parser/topic/schema changes, no orchestration CLI changes, no live ROS.

## Governance Result

**Verdict: frozen** for PLAT-SA-H4 (workstation integration).

## Boundary Checks

| Check | Result |
|-------|--------|
| Authority creep | Pass — navigation loads static mirrors/bundles only |
| Parser safety | Pass — no schema changes |
| Live vs replay | Pass — static JSON fetch only |
| Orchestration execution | Pass — queue panel read-only; no launch controls |
| HITL / C2 | Pass — no command UX |
| Scoring / readiness | Pass — no ranking or deployment semantics |
| Browser runtime control | Pass — no Gazebo/WebSocket/rosbridge from viewer |

## Cognition / workflow improvements

- **Segment-complete Compare and Report** — nav tabs always render a workspace surface (pair catalog, storyboard picker).
- **Experiment lineage panel** — read-only chain across scenario, validation mirror, queue job, bundle, corpus, compare.
- **Orchestration deep links** — queue jobs and validation mirror open linked replay/corpus artifacts.
- **Scenario handoff** — loading a pack switches to Replay and sets `corpus_entry` when indexed.
- **Single-demo experiment review rail** — lineage, provenance, linkage, compare entry without requiring sweep mode.
- **Sweep walkthrough steps** — `compare_pair`, `cohort_filmstrip`, `chapter` steps navigate via SPA stores.
- **URL coherence** — `orchestration_queue` synced on queue select; compare/presentation preserve segment intent via store snapshot.

## Validation

```bash
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
python3 scripts/evaluation/governance_lint_sa.py
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

## Related

- PLAN-SA-H1, PLAT-SA-H2, PLAT-SA-H3 (frozen prerequisites)
- H5: async workers, focus layout, authoring workstation — see H4 plan §H5 boundary
