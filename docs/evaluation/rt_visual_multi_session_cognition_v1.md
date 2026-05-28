# RT Visual Multi-Session Cognition Contract (`rt_visual_multi_session_cognition_v1`)

**Phase:** PLAN-RT-V4 - multi-session visualization planning  
**Authority:** [rt_v4_visualization_fidelity_plan.md](../platform/rt_v4_visualization_fidelity_plan.md)  
**Supplements:** [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md), [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md), [rt_visual_density_management_v1.md](rt_visual_density_management_v1.md)

Planning contract for local multi-session visual cognition after V3/M3. This contract improves how reviewers compare sessions visually, while preserving the local cap=3 single-bridge model and selected-session command authority.

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Session model | Local cap=3; no distributed multi-bridge |
| Command target | Selected session only |
| Background sessions | Pull/display context only |
| Comparison | Explanatory; no merge, sync, import, or batch command |
| Persistence | UI-local preferences only |

## 1. Session visual states

| State | Meaning | Visual rule |
|-------|---------|-------------|
| `selected` | Active tab/session; command affordances follow existing locks | Full accent |
| `comparison` | Explicitly chosen visual comparison session | Muted accent, dashed geometry |
| `background` | Nonselected local session with pull diagnostics | Row/chip only by default |
| `stale_background` | Background pull stale | Warning chip; no auto-reset |
| `paused_background` | Background pull paused | Neutral paused chip |

## 2. Multi-session comparison surfaces

| Surface | PLAN-RT-V4 rule |
|---------|-----------------|
| Session comparison strip | Shows active/comparison/background labels and pull age |
| Globe comparison overlay | Optional future PLAT; default off; ghosted only |
| Terrain comparison row | Displays terrain/visibility context differences as text |
| Layer-state comparison | Shows layer-on/layer-off deltas; no remote mutation |
| Diagnostic cohesion row | Summarizes stale/paused/dirty without health score |

## 3. Command isolation

| Check | Requirement |
|-------|-------------|
| Click on comparison ghost | Selects or focuses comparison view only; does not command it |
| Drag on comparison ghost | Forbidden unless session becomes selected under existing editing lock |
| Batch layer toggle | UI-local display preference only; no bridge call |
| Session compare export | Not an SA import or corpus write |

## 4. Visual comparison grammar

| Item | Selected | Comparison | Background |
|------|----------|------------|------------|
| Marker | Solid accent | Dashed/muted accent | Hidden from globe by default |
| Bounds | Solid | Dashed | Row only |
| Terrain labels | Active labels | Optional paired text | Hidden |
| Diagnostics | Full row | Compact row | Compact chip |
| Commands | Existing selected affordances | None | None |

## 5. Cognition lines

Approved line patterns:

```text
Session compare: selected s1 vs comparison s2 - visual only.
Layer delta: visibility corridor on in s1, off in s2.
Pull delta: s2 last pull 8s older than selected.
```

Forbidden line patterns:

- `s2 is less ready`
- `s2 is unhealthy`
- `auto-sync recommended`
- `import comparison`
- `operational picture`

## 6. Relationship to F8 and X2

| Adjacent wave | Boundary |
|---------------|----------|
| F8 advisory | Advisory cohorts and handoff blockers remain separate; V4 does not rank readiness |
| X2 experiments | V4 comparison visuals may inform future X3 planning but do not change experiment manifests |
| SA handoff | No import, auto-import, corpus write, or SA viewer mutation |

## 7. Explicit non-goals

- Distributed multi-bridge or cloud sessions
- Cross-session command execution
- Registry merge or command truth changes
- Browser->ROS authority
- SA viewer live hooks
- Readiness scoring

## Related

- [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md)
- [rt_visual_density_management_v1.md](rt_visual_density_management_v1.md)
