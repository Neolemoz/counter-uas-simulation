# RT - Next Frontiers Roadmap v11

**Phase:** Advisory roadmap after PLAN-RT-C4 cleanup planning  
**Prerequisite:** [rt_c4_freeze_audit.md](rt_c4_freeze_audit.md), [rt_checkpoint_post_v4_freeze_audit.md](rt_checkpoint_post_v4_freeze_audit.md), [rt_plat_v4_p2_freeze_audit.md](rt_plat_v4_p2_freeze_audit.md)  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v10.md](rt_roadmap_next_frontiers_v10.md)

This roadmap ranks candidate next steps after PLAN-RT-C4 cleanup planning. It authorizes no implementation.

## Completed Baseline

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-F8 P0–P2 | Advisory summary v2, triage integration, guardrails/corpus preview | Frozen — F8 complete |
| PLAT-RT-V4 P0–P2 | Density/registry, visibility overlays, workstation cohesion | Frozen — V4 complete |
| PLAT-RT-X2 P0–P2 | Cohort index, unified review, multi-manifest export | Frozen — X2 complete |
| PLAT-RT-M3 P0–P2 | Session inspect, poll UX, reorder/diagnostics | Frozen — M3 complete |
| CHECKPOINT-RT-POST-V4 | Concentration, boundary, duplication, validation, bundle review | Frozen |
| PLAN-RT-C4 | Post-V4 cleanup planning (App/workbench/duplication/PLAT phasing) | This wave — docs frozen |

## Candidate Frontiers

### 1. PLAT-RT-C4 (UI concentration cleanup)

**Description:** Behavior-neutral UI decomposition per [rt_c4_checkpoint_cleanup_plan.md](../platform/rt_c4_checkpoint_cleanup_plan.md): P0 experiment imports/toolbar, P1 compare + F5 sections, P2 App entity-editing hook.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low–Med** — UI-local; touches concentrated parents |
| Governance cost | **Low** — if scoped tightly and import guards unchanged |
| Maintainer value | **High** — lowers edit risk before X3 |

Constraints: no semantic/authority changes; no bridge/runtime/SA/import/federation; per-phase PLAT plan + freeze.

### 2. PLAN-RT-X3

**Description:** Docs-first experiment workbench follow-on only if maintainers document concrete post-X2 gaps.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Med–High** — experiment parent remains concentrated until PLAT-C4 P0–P1 |
| Governance cost | **Med** — manifest/review/export/advisory adjacency |
| Maintainer value | **Med–High** — valuable if actual X2 friction exists |

Constraints: docs-only first; no manifest authority changes; no parser/bridge/import changes; no SA writes.

### 3. Pause Plateau

**Description:** Hold the platform at current frozen state until maintainer pain is clear.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low** |
| Governance cost | **Low** |
| Maintainer value | **Med** — preserves stability; does not reduce concentration |

Constraints: routine review/validation only; no hidden implementation.

## Ranking Summary

| Rank | Frontier | Coupling risk | Governance cost | Maintainer value | Rationale |
|------|----------|---------------|-----------------|------------------|-----------|
| 1 | **PLAT-RT-C4** (P0→P2) | Low–Med | Low | **High** | Best value/risk after C4 plan freeze; implements v10 cleanup recommendation |
| 2 | **PLAN-RT-X3** | Med–High | Med | Med–High | Worth planning only after concrete experiment gaps or post–P0–P1 cleanup |
| 3 | **Pause plateau** | Low | Low | Med | Stable if no immediate maintainer pain |

## Recommendation

**Recommended next (advisory):** After PLAN-RT-C4 freeze, start **PLAT-RT-C4 P0** (experiment import helper + manifest toolbar) with a scoped PLAT plan and freeze — not authorized by this roadmap alone.

**Alternate 1:** **PLAN-RT-X3** docs-only if maintainers have documented X2 workflow gaps and choose to defer PLAT cleanup.

**Alternate 2:** **Pause plateau** if stability outweighs concentration reduction.

## Explicit Non-Frontiers

- Distributed multi-bridge
- Bridge command or telemetry expansion
- SA live hooks or SA viewer mutation
- Auto-import, corpus writes, federation writes
- Browser→ROS authority
- Tactical redesign or HITL/C2 semantics
- Operational readiness scoring
- Parser/topic/schema changes
- Bundle/code-splitting as part of PLAT-RT-C4 (deferred)

## Authorization Checklist

Before PLAT-RT-C4, PLAN-RT-X3, or any successor:

1. Scoped plan in `docs/platform/` (PLAT per phase)
2. Governance review and freeze audit
3. Contamination review if advisory/import/handoff adjacency appears
4. Regression matrix: `tier0-rt-ui`, targeted Vitest, relevant bridge tests per scope
5. Freeze registry and AGENTS updates

No candidate is authorized by this roadmap alone.
