# RT - Next Frontiers Roadmap v10

**Phase:** Advisory roadmap after post-V4 checkpoint review
**Prerequisite:** [rt_checkpoint_post_v4_freeze_audit.md](rt_checkpoint_post_v4_freeze_audit.md), [rt_plat_v4_p2_freeze_audit.md](rt_plat_v4_p2_freeze_audit.md), [rt_plat_f8_p2_freeze_audit.md](rt_plat_f8_p2_freeze_audit.md)
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v9.md](rt_roadmap_next_frontiers_v9.md)

This roadmap ranks candidate next steps after PLAT-RT-V4 completion and the post-V4 checkpoint. It authorizes no implementation.

## Completed Baseline

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-F8 P0-P2 | Advisory summary v2, triage integration, guardrails/corpus preview | Frozen - F8 complete |
| PLAT-RT-V4 P0-P2 | Density/registry foundations, visibility overlays, workstation cohesion | Frozen - V4 complete |
| PLAT-RT-X2 P0-P2 | Cohort index, unified review, multi-manifest export | Frozen - X2 complete |
| PLAT-RT-M3 P0-P2 | Local session inspect, poll UX, reorder/diagnostics | Frozen - M3 complete |
| PLAN-RT-C3 | Post-X2 consolidation | Frozen |
| Post-V4 checkpoint | Concentration, boundary, duplication, validation, bundle review | This wave - docs frozen |

## Candidate Frontiers

### 1. Checkpoint Cleanup

**Description:** Narrow implementation cleanup to reduce UI concentration without changing behavior.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low-Med** - UI-local but touches concentrated parents |
| Governance cost | **Low** - no semantic change if scoped tightly |
| Maintainer value | **High** - improves future editability before X3 |

Constraints: no behavior changes, no bridge/runtime changes, no SA/import/federation work, no new authority semantics.

### 2. PLAN-RT-X3

**Description:** Docs-first experiment workbench follow-on only if maintainers document concrete post-X2 gaps.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Med-High** - experiment parent is concentrated |
| Governance cost | **Med** - manifest/review/export/advisory adjacency |
| Maintainer value | **Med-High** - valuable if actual X2 friction exists |

Constraints: docs-only first; no manifest authority changes; no parser/bridge/import changes; no SA writes.

### 3. Pause Plateau

**Description:** Hold the platform at the current frozen state and avoid new waves until maintainer pain is clear.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low** |
| Governance cost | **Low** |
| Maintainer value | **Med** - preserves stability, but does not reduce concentration |

Constraints: continue only routine review/validation; no hidden implementation.

## Ranking Summary

| Rank | Frontier | Coupling risk | Governance cost | Maintainer value | Rationale |
|------|----------|---------------|-----------------|------------------|-----------|
| 1 | **Checkpoint cleanup** | Low-Med | Low | High | Best value/risk tradeoff; reduces editability risk after F8/V4 growth |
| 2 | **PLAN-RT-X3** | Med-High | Med | Med-High | Worth planning only after concrete experiment gaps are documented |
| 3 | **Pause plateau** | Low | Low | Med | Stable option if no immediate maintainer pain exists |

## Recommendation

**Recommended next (advisory):** checkpoint cleanup, scoped narrowly to UI concentration and duplication without semantic or runtime changes.

**Alternate 1:** PLAN-RT-X3 docs-only if maintainers have concrete X2 workflow gaps.

**Alternate 2:** Pause plateau if platform stability is more valuable than lowering UI concentration right now.

## Explicit Non-Frontiers

- Distributed multi-bridge
- Bridge command or telemetry expansion
- SA live hooks or SA viewer mutation
- Auto-import, corpus writes, federation writes
- Browser->ROS authority
- Tactical redesign or HITL/C2 semantics
- Operational readiness scoring
- Parser/topic/schema changes

## Authorization Checklist

Before cleanup, PLAN-RT-X3, or any checkpoint successor:

1. Scoped plan in `docs/platform/`
2. Governance review and freeze audit
3. Visualization realism review if visual surfaces change
4. Contamination review if advisory/import/handoff adjacency appears
5. Regression matrix per wave scope
6. Freeze registry and AGENTS updates

No candidate is authorized by this roadmap alone.
