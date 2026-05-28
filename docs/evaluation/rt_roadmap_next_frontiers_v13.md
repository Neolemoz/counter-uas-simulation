# RT — Next Frontiers Roadmap v13

**Phase:** Advisory roadmap after CHECKPOINT-RT-POST-X3
**Prerequisite:** [rt_checkpoint_post_x3_freeze_audit.md](rt_checkpoint_post_x3_freeze_audit.md), [rt_plat_x3_p2_freeze_audit.md](rt_plat_x3_p2_freeze_audit.md), [rt_plat_c4_p2_freeze_audit.md](rt_plat_c4_p2_freeze_audit.md), [rt_plat_v4_p2_freeze_audit.md](rt_plat_v4_p2_freeze_audit.md)
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v12.md](rt_roadmap_next_frontiers_v12.md)

This roadmap ranks candidate next steps after PLAT-RT-V4, PLAT-RT-C4, and PLAT-RT-X3 completion. It authorizes no implementation.

## Completed baseline

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-V4 P0–P2 | Density/registry, visibility, workstation cohesion | Frozen — V4 complete |
| PLAT-RT-C4 P0–P2 | Import helper, sections, App/workstation slots | Frozen — C4 complete |
| PLAT-RT-X3 P0–P2 | V3 shell, review lane, multi-manifest/packet polish | Frozen — X3 complete |
| PLAT-RT-F8 P0–P2 | Advisory v2, triage, guardrails | Frozen — F8 complete |
| PLAT-RT-X2 P0–P2 | Cohort, unified review, compare v2 | Frozen |
| PLAT-RT-M3 P0–P2 | Session inspect, poll, reorder | Frozen |
| CHECKPOINT-RT-POST-V4 | Post-V4 concentration review | Frozen |
| CHECKPOINT-RT-POST-X3 | Post-X3 maturity re-baseline | This wave — docs frozen |
| PLAN-RT-X3 / PLAN-RT-C4 / PLAN-RT-V4 | Planning waves | Frozen |

## Candidate frontiers (prospective)

### 1. Pause plateau

**Description:** Hold platform at frozen PLAT state; routine validation only.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low** |
| Governance cost | **Low** |
| Maintainer value | **High** — stability after V4+C4+X3 at `4d85865` |

### 2. PLAN-RT-F9 (advisory maintainer expansion v9)

**Description:** Docs-first follow-on to F8 — triage/batch/contamination ergonomics, maintainer workflow planning. **Not** in repo yet.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Med** — handoff/advisory UI adjacency |
| Governance cost | **High** — F6/F7/SA import boundaries |
| Maintainer value | **Med-High** — if triage/batch pain is binding |

Constraints: advisory only; no SA auto-import; no readiness scoring; contamination review required.

### 3. PLAN-RT-V5 (visualization fidelity v5)

**Description:** Docs-first post-V4/X3 visualization planning — density/bundle/multi-session visual cognition, Cesium/workstation coupling. **Not** in repo yet.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Med-High** — Cesium + workstation + compare chrome |
| Governance cost | **Med** — display-only surfaces |
| Maintainer value | **Med** — if bundle/density/visual compare pain dominates |

Constraints: no bridge/runtime; no authority semantics; code-split planning only in PLAN wave.

## Ranking summary

| Rank | Frontier | Coupling risk | Governance cost | Maintainer value | Rationale |
|------|----------|---------------|-----------------|------------------|-----------|
| 1 | **Pause plateau** | Low | Low | **High** | Platform stable; three major PLAT tracks just completed |
| 2 | **PLAN-RT-F9** | Med | **High** | Med-High | Valuable if advisory maintainer workflows are the pain point |
| 3 | **PLAN-RT-V5** | Med-High | Med | Med | Valuable if visualization/bundle/density is the pain point |

## Recommendation

**Recommended next (advisory):** **Pause plateau** — re-baseline complete at CHECKPOINT-RT-POST-X3.

**Alternate 1:** **PLAN-RT-F9** (docs-only) if maintainer triage/batch workflows need formal planning.

**Alternate 2:** **PLAN-RT-V5** (docs-only) if visualization bundle/density is the binding constraint.

No candidate is authorized by this roadmap or checkpoint alone.

## Explicit non-frontiers

- Distributed multi-bridge
- Bridge command or telemetry expansion
- SA live hooks or SA viewer mutation
- Auto-import, corpus writes, federation writes
- Browser→ROS authority
- Tactical redesign or HITL/C2 semantics
- Operational readiness scoring
- Parser/topic/schema changes
- Import semantic changes
- PLAT work without per-phase plan + freeze

## Authorization checklist

Before any PLAN or PLAT successor:

1. Scoped plan in `docs/platform/`
2. Governance review and freeze audit
3. Contamination review if F6/F7 or SA adjacency
4. Regression matrix: `tier0-rt-ui`, Vitest, bridge tests per scope
5. Freeze registry and AGENTS updates
