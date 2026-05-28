# RT — Next Frontiers Roadmap v12

**Phase:** Advisory roadmap after PLAN-RT-X3 experiment workbench v3 planning  
**Prerequisite:** [rt_x3_freeze_audit.md](rt_x3_freeze_audit.md), [rt_plat_c4_p2_freeze_audit.md](rt_plat_c4_p2_freeze_audit.md), [rt_plat_x2_p2_freeze_audit.md](rt_plat_x2_p2_freeze_audit.md)  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v11.md](rt_roadmap_next_frontiers_v11.md)

This roadmap ranks candidate next steps after PLAN-RT-X3 freeze. It authorizes no implementation.

## Completed Baseline

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-X2 P0–P2 | Cohort index, unified review, multi-manifest diff, packet export | Frozen — X2 complete |
| PLAT-RT-C4 P0–P2 | Import helper, manifest toolbar, compare/F5 sections, App hooks | Frozen — C4 complete |
| PLAT-RT-V4 P0–P2 | Density/registry, visibility overlays, workstation cohesion | Frozen — V4 complete |
| PLAT-RT-F8 P0–P2 | Advisory summary v2, triage integration, guardrails | Frozen — F8 complete |
| CHECKPOINT-RT-POST-V4 | Concentration, boundary, duplication review | Frozen |
| PLAN-RT-C4 | Post-V4 cleanup planning | Frozen |
| PLAN-RT-X3 | Experiment workbench v3 ergonomics planning | This wave — docs frozen |

## Candidate Frontiers

### 1. PLAT-RT-X3 (experiment workbench v3 ergonomics)

**Description:** Implement v3 navigation, grouped dock, packet sections, compare coach per [rt_roadmap_plat_rt_x3_v1.md](rt_roadmap_plat_rt_x3_v1.md).

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Med** — UI-local `experiment/`; post-C4 decomposition lowers edit risk |
| Governance cost | **Med** — F6/F7 adjacency at P1; contamination review required |
| Maintainer value | **High** — closes documented post-X2 friction |

Constraints: no import/capture semantic changes; no bridge/runtime/SA; per-phase PLAT plan + freeze.

### 2. Pause plateau

**Description:** Hold platform at current frozen state until maintainer pain is clear.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low** |
| Governance cost | **Low** |
| Maintainer value | **Med** — preserves stability |

### 3. CHECKPOINT-RT-POST-X3 (optional docs-only)

**Description:** Review concentration after PLAT-RT-X3 if `ExperimentWorkbenchPanel` or `App.tsx` regrows.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low** |
| Governance cost | **Low** |
| Maintainer value | **Med** — only if PLAT-X3 increases orchestration LOC |

**Not** a substitute for POST-V4 or PLAN-RT-C4 reviews.

## Alternate frontier

**Visualization / advisory maintenance** — shared compare-status primitives, cognition strip row helpers per [rt_checkpoint_post_v4_architecture_review_r1.md](../platform/rt_checkpoint_post_v4_architecture_review_r1.md). Scoped debt paydown; not a PLAN wave unless expanded.

## Ranking Summary

| Rank | Frontier | Coupling risk | Governance cost | Maintainer value | Rationale |
|------|----------|---------------|-----------------|------------------|-----------|
| 1 | **PLAT-RT-X3** (P0→P2) | Med | Med | **High** | Best value after X3 plan freeze; builds on X2 + C4 |
| 2 | **Pause plateau** | Low | Low | Med | Stable if no near-term experiment UX pain |
| 3 | **CHECKPOINT-RT-POST-X3** | Low | Low | Med | Optional after PLAT-X3 if concentration regrows |

## Recommendation

**Recommended next (advisory):** After PLAN-RT-X3 freeze, start **PLAT-RT-X3 P0** (cohort navigation ergonomics) with scoped PLAT plan and freeze — not authorized by this roadmap alone.

**Alternate 1:** **Pause plateau** if experiment navigation is not the binding constraint.

**Alternate 2:** **Visualization/advisory maintenance** for shared compare-status helpers without full X3 scope.

## Explicit Non-Frontiers

- Distributed multi-bridge
- Bridge command or telemetry expansion
- SA live hooks or SA viewer mutation
- Auto-import, corpus writes, federation writes
- Browser→ROS authority
- Tactical redesign or HITL/C2 semantics
- Operational readiness scoring
- Parser/topic/schema changes
- Import semantic changes under guise of “ergonomics”

## Authorization Checklist

Before PLAT-RT-X3 or any successor:

1. Scoped plan in `docs/platform/` (`rt_plat_x3_p*` per phase)
2. Governance review and freeze audit
3. Contamination review at P1 if advisory/import/handoff adjacency appears
4. Regression matrix: `tier0-rt-ui`, targeted Vitest, relevant bridge tests per scope
5. Freeze registry and AGENTS updates

No candidate is authorized by this roadmap alone.
