# RT - PLAT-RT-V4 Implementation Roadmap v1

**Phase:** PLAN-RT-V4 frozen -> future **PLAT-RT-V4** (not authorized)  
**Prerequisite:** [rt_v4_freeze_audit.md](rt_v4_freeze_audit.md) (PLAN-RT-V4 docs frozen)  
**Contracts:** [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md), [rt_visual_density_management_v1.md](rt_visual_density_management_v1.md), [rt_visual_multi_session_cognition_v1.md](rt_visual_multi_session_cognition_v1.md)

This roadmap is advisory. It ranks a conservative implementation path for V4 but does not authorize any runtime/UI work.

## P0 - Density policy foundations

**Prerequisite:** PLAN-RT-V4 frozen; V4 governance review accepted for P0.

| Item | Future location | Status |
|------|-----------------|--------|
| Density policy helper | `platform/rt-sandbox-ui/src/cesium/` | Planned |
| Layer budget summary | Existing V3 toggle/cognition surfaces | Planned |
| Legend grouping for density | Existing visualization rail | Planned |
| Vitest policy checks | RT UI tests | Planned |
| Governance check | `rt_plat_v4_p0_*` docs | Required |

**Risk:** Low-Med. The phase should be policy-first and avoid new visual geometry.

## P1 - Advanced visibility and terrain cognition

**Prerequisite:** P0 frozen.

| Item | Future location | Status |
|------|-----------------|--------|
| Visibility corridor visual | Existing Cesium visualization modules | Planned |
| Occlusion confidence bands | Existing terrain/visibility cognition | Planned |
| Terrain relation labels | Existing terrain overlays | Planned |
| Default-off toggle wiring | Existing V3 layer registry surface | Planned |
| Realism review | `rt_plat_v4_p1_visualization_realism_review_r1.md` | Required |

**Risk:** Med. Misread risk is the main cost; every new visual must remain explanatory.

## P2 - Multi-session comparison visuals

**Prerequisite:** P1 frozen; M3 session isolation re-check.

| Item | Future location | Status |
|------|-----------------|--------|
| Session comparison strip | Existing workstation session area | Planned |
| Ghosted comparison geometry | Existing Cesium panel | Planned, default off |
| Background diagnostic cohesion row | Existing background diagnostics | Planned |
| Session compare legend | Existing cognition hub/rail | Planned |
| Isolation tests | RT UI tests + bridge deny-path tests | Required |

**Risk:** Med. The phase must not introduce cross-session command affordances or distributed runtime assumptions.

## Explicit out of scope

- Bridge protocol or subcommand changes
- ROS/Gazebo runtime changes
- SA viewer changes
- Auto-import, corpus writes, or federation
- Browser->ROS authority
- Tactical redesign or operational readiness scoring
- Distributed multi-bridge

## Validation (future PLAT)

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

Each PLAT phase requires its own plan, governance review, visualization realism review when visuals change, freeze audit, and freeze registry row.

## Stop line

This roadmap does not start PLAT-RT-V4. Implementation remains blocked until an explicit PLAT phase is opened.

## Related

- [rt_roadmap_next_frontiers_v9.md](rt_roadmap_next_frontiers_v9.md)
- [rt_v4_governance_review_r1.md](rt_v4_governance_review_r1.md)
