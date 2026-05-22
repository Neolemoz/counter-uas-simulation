# SA H2 — Sandbox Workspace Shell Freeze Audit (PLAT-SA-H2)

## Scope

Viewer-only refactor per [h1_sandbox_ux_architecture_plan.md](../platform/h1_sandbox_ux_architecture_plan.md) (PLAN-SA-H1):

- `platform/sa-r0-viewer/src/workspace/` — shell, segment nav, panel registry, discover rails
- Refactored `App.tsx`, `CompareView.tsx`, `CohortFilmstripView.tsx`, `PresentationLayoutShell.tsx`
- Governance chrome title unification

No parser/topic/schema changes, no orchestration execution, no live ROS.

## Governance Result

**Verdict: frozen** for PLAT-SA-H2 (viewer layout refactor).

## Boundary Checks

| Check | Result |
|-------|--------|
| Authority creep | Pass — replay-derived only |
| Parser safety | Pass — no schema changes |
| Live vs replay | Pass — static JSON fetch only |
| Orchestration | Pass — no launch controls |
| HITL / C2 | Pass — no command UX |
| Scoring | Pass — no ranking UI added |
| Compare clock | Pass — Option A: `useActiveReplaySlot` for mock panes |

## Validation

```bash
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
python3 scripts/evaluation/governance_lint_sa.py
```

## Related

- PLAN-SA-H1: [h1_sandbox_ux_architecture_plan.md](../platform/h1_sandbox_ux_architecture_plan.md)
- H3 deferred: authoring shell, orchestration read-only panel
