# RT - Next Frontiers Roadmap v9

**Phase:** Advisory roadmap after PLAN-RT-V4 freeze  
**Prerequisite:** [rt_v4_freeze_audit.md](rt_v4_freeze_audit.md), [rt_plat_f8_p2_freeze_audit.md](rt_plat_f8_p2_freeze_audit.md), [rt_c3_platform_consolidation_freeze_audit.md](rt_c3_platform_consolidation_freeze_audit.md)  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v8.md](rt_roadmap_next_frontiers_v8.md)

This roadmap ranks candidate next steps after PLAN-RT-V4. It authorizes no implementation.

## 1. Completed baseline

| ID | Summary | Status |
|----|---------|--------|
| PLAT-RT-F8 P0-P2 | Advisory summary v2, integrated triage, guardrails/corpus preview | Frozen - F8 complete |
| PLAT-RT-V3 P0-P2 | Layer registry, visibility overlays, workstation layout | Frozen - V3 complete |
| PLAT-RT-X2 P0-P2 | Cohort index, unified review, multi-manifest export | Frozen - X2 complete |
| PLAT-RT-M3 P0-P2 | Local session inspect, poll UX, reorder/diagnostics | Frozen - M3 complete |
| PLAN-RT-V4 | Visualization fidelity v4 planning | This wave - docs frozen |

## 2. Candidate frontiers

### PLAT-RT-V4 - Visualization fidelity implementation

**Description:** Implements V4 density policy, advanced visibility/terrain cognition, and local multi-session comparison visuals under the RT-only, explanatory-only boundary.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Med** - Cesium/workstation concentration |
| Governance cost | **Low-Med** - visual realism and authority labels |
| Maintainer value | **Med-High** - demo/reviewer cognition and session comparison |

**Dependencies:** PLAN-RT-V4 frozen; per-phase PLAT plan and reviews.

**Roadmap:** [rt_roadmap_plat_rt_v4_v1.md](rt_roadmap_plat_rt_v4_v1.md)

### PLAN-RT-X3 - Experiment workbench follow-on

**Description:** Narrow post-X2 experiment ergonomics planning, only if concrete X2 comparison gaps are documented.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **High** - experiment/workbench adjacency |
| Governance cost | **Med** - manifest/review/export semantics |
| Maintainer value | **Med** - useful but less urgent than V4 visual cognition |

**Constraints:** No manifest authority change, no bridge commands, no SA import, no federation writes.

### Future checkpoint review

**Description:** Documentation-only consolidation after PLAT-RT-V4 or X3, depending on which frontier lands first.

| Dimension | Score |
|-----------|-------|
| Coupling risk | **Low** |
| Governance cost | **Low** |
| Maintainer value | **Med** after a PLAT wave; **Low** immediately after PLAN only |

**Constraints:** No PLAT-C4 concept unless explicitly defined; checkpoint is review-only.

## 3. Ranking summary

| Rank | Frontier | Coupling risk | Governance cost | Maintainer value | Rationale |
|------|----------|---------------|-----------------|------------------|-----------|
| 1 | **PLAT-RT-V4** | Med | Low-Med | Med-High | PLAN now exists; value is clear for visualization/reviewer cognition; contamination low |
| 2 | **PLAN-RT-X3** | High | Med | Med | Potentially useful, but X2 surface is broad and needs evidenced gaps |
| 3 | **Checkpoint review** | Low | Low | Low-Med | Best after a delivery wave, not immediately after docs-only V4 |

## 4. Recommendation

**Recommended next (advisory):** **PLAT-RT-V4 P0** - density policy foundations only, after an explicit PLAT plan and governance review.

**Alternate 1:** **PLAN-RT-X3** if maintainers identify concrete post-X2 experiment workflow gaps that outweigh visualization needs.

**Alternate 2:** **Checkpoint review** if no implementation wave should start and the priority is another governance plateau.

## 5. Explicit non-frontiers

- Distributed multi-bridge
- Bridge command or telemetry expansion
- SA live hooks or SA viewer mutation
- Auto-import, corpus writes, federation writes
- Browser->ROS authority
- Tactical redesign or HITL/C2 semantics
- Operational readiness scoring

## 6. Authorization checklist

Before PLAT-RT-V4, PLAN-RT-X3, or checkpoint work:

1. Scoped plan in `docs/platform/`
2. Governance review and freeze audit
3. Visualization realism review for visual changes
4. Contamination review if advisory/import/handoff adjacency appears
5. Regression matrix per wave scope
6. Freeze registry and AGENTS updates

No candidate is authorized by this roadmap alone.

## Related

- [rt_v4_freeze_audit.md](rt_v4_freeze_audit.md)
- [rt_roadmap_next_frontiers_v8.md](rt_roadmap_next_frontiers_v8.md)
- [rt_roadmap_plat_rt_v4_v1.md](rt_roadmap_plat_rt_v4_v1.md)
