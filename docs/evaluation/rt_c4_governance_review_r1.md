# RT-C4 — Governance Review R1

**Phase:** PLAN-RT-C4 — post-V4 checkpoint cleanup planning  
**Plan:** [rt_c4_checkpoint_cleanup_plan.md](../platform/rt_c4_checkpoint_cleanup_plan.md)  
**Baseline:** [rt_checkpoint_post_v4_governance_review_r1.md](rt_checkpoint_post_v4_governance_review_r1.md)  
**Status:** pass

## Governance verdict

PLAN-RT-C4 is **docs-only** and preserves all AGENTS.md RT boundaries. Future **behavior-neutral** UI extraction (PLAT-RT-C4) is compatible with additive-only evolution and freeze-before-expansion **if** each PLAT phase avoids authority, import, and advisory semantic changes.

## Boundary checks

| Boundary | Verdict |
|----------|---------|
| AGENTS authority preserved | Pass |
| Additive-only (PLAT = refactor without semantic change) | Pass — conditional on PLAT discipline |
| Freeze-before-expansion | Pass — PLAN-RT-C4 freezes before any PLAT wave |
| Explanatory != authority | Pass — cognition/density/compare remain display-only |
| Advisory != authority | Pass — rollup/handoff remain advisory |
| RT-only scope | Pass |
| No bridge/runtime changes in PLAN-RT-C4 | Pass |
| No browser→ROS authority | Pass |
| No SA contamination | Pass |
| No import semantic changes | Pass — guards unchanged in PLAN; PLAT must not alter validation |
| No federation / distributed runtime | Pass |
| Parser-safe boundaries | Pass |

## Cleanup vs authority drift

| Risk | Mitigation |
|------|------------|
| Moving bridge calls into deep children | Keep `useRtSessionWorkspace` and `entityCommands` as call sites; hooks wrap, do not re-route authority |
| Import guard changes during prompt consolidation | `useJsonPromptImport` must call existing `safeParse*` functions unchanged |
| Advisory rollup relocation | Defer or require contamination review; no new “ready/import now” language |
| Experiment compare labels implying operational readiness | Shared vocabulary helpers are display-only; no readiness scoring |

**Verdict:** Extracting hooks and presentational containers does **not** inherently drift authority if `session_id`, bridge commands, and derive inputs/outputs are unchanged.

## Layer boundary review

- **Runtime:** Bridge/session command paths remain authoritative; PLAN-RT-C4 implements nothing.
- **Replay/SA:** Manual handoff/import boundary intact; no SA viewer changes.
- **Advisory:** F8 labels remain maintainer workflow aids, not operational scoring.
- **Experiment:** Manifests, metrics, and review packets remain derived review artifacts.
- **Visualization:** V4 surfaces remain display-only; density budgets warn only.

## Contamination gates (for future PLAT-RT-C4)

| Trigger | Required review |
|---------|-----------------|
| Touch `deriveAdvisoryState`, `advisory_aggregate`, batch summary builders | Contamination review |
| Move handoff rollup effect out of workbench parent | Contamination review |
| Change `experimentImportGuards` validation rules | Governance + parser-safe review (likely out of scope) |
| Any write under `platform/sa-r0-viewer/` or `fixtures/sa_r0/**` | Forbidden |

PLAN-RT-C4 does not authorize PLAT work; gates apply when PLAT plans are drafted.

## Governance risk

Primary risk is **language drift** from duplicated cognition/compare/density wording across strips. Future shared helpers should centralize **wording only**, not authority semantics. Early PLAT waves should not merge advisory derive paths with experiment UI helpers.

## Verdict

**Pass.** PLAN-RT-C4 is docs-only and preserves RT/SA separation. PLAT-RT-C4 remains **unauthorized** until per-phase plan, governance re-check, validation, and freeze audit.
