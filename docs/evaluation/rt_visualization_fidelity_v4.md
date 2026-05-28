# RT Visualization Fidelity V4 Contract (`rt_visualization_fidelity_v4`)

**Phase:** PLAN-RT-V4 - visualization fidelity planning  
**Authority:** [rt_v4_visualization_fidelity_plan.md](../platform/rt_v4_visualization_fidelity_plan.md)  
**Supplements:** [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md), [rt_cesium_workstation_visualization_v3_v1.md](rt_cesium_workstation_visualization_v3_v1.md), [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md), [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md)

Normative planning contract for the fourth RT visualization fidelity layer. V4 defines richer multi-session visualization, advanced visibility and terrain cognition, visual density management hooks, and session comparison visuals. All surfaces are explanatory unless an existing frozen contract explicitly labels a value otherwise.

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Scope | RT sandbox visualization planning only |
| Authority | Entity registry and bridge command handlers remain command truth |
| Transport | Loopback pull and local session model unchanged |
| SA | No SA viewer, replay bundle, import, or federation changes |
| Runtime | No ROS, Gazebo, bridge, parser, topic, or schema changes |
| Default posture | V4-new overlays default off until PLAT governance approves otherwise |

V4 never changes spawn, move, delete, reset, tactical, capture, handoff, or import semantics.

## 1. Visualization fidelity v4 layer classes

| Layer class | Purpose | Authority status |
|-------------|---------|------------------|
| `terrain_cognition_v4` | Terrain readability cues, ridge/valley relation, elevation context | Explanatory |
| `visibility_cognition_v4` | Visibility corridor, occlusion corridor, and LOS confidence display language | Explanatory |
| `session_compare_visual_v4` | Side-by-side or overlay comparison cues for local sessions | Explanatory |
| `diagnostic_visual_cohesion_v4` | Unified visual grammar for pull age, stale, paused, and background status | Explanatory |
| `density_policy_v1` | Declutter, priority, label, and legend rules | Explanatory policy |

## 2. Advanced visibility and terrain cognition

| Feature | V4 rule |
|---------|---------|
| Visibility corridors | Drawn as heuristic corridors derived from existing UI-local terrain and LOS cues; never sensor coverage |
| Occlusion confidence bands | Use qualitative labels only: `low`, `medium`, `high`; no probability or readiness score |
| Terrain relation labels | May label `ridge-side`, `valley ingress`, `screened`, `exposed` as fictional cognition |
| LOS stack refinement | May group V3 wedge/horizon/stacked LOS into a single visual explanation block |
| Sensor domes | Remain nominal; no coverage proof or detection assertion |

Required label pattern:

```text
Visibility cognition: heuristic sandbox cue - not sensor coverage or operational picture.
```

## 3. Multi-session visualization

| Topic | V4 rule |
|-------|---------|
| Session cap | Local cap=3 unchanged |
| Active session | Remains primary globe authority for display focus |
| Background sessions | May appear in comparison surfaces only; never as command targets without explicit tab activation |
| Comparison visuals | Must distinguish `selected`, `comparison`, and `background` states |
| Session deltas | Visual-only differences in pose, layer state, pull age, and terrain context; no merge/reconcile command |

Comparison visuals may support:

- aligned camera snapshots
- ghosted bounds or tracks for comparison sessions
- split legend rows
- per-session density summaries

They must not support:

- cross-session batch commands
- auto-reset or auto-sync
- distributed session routing
- federation writes

## 4. Session comparison visual contract

| Visual element | Required behavior |
|----------------|-------------------|
| Comparison ghost | Dashed or muted; label includes `comparison` |
| Active command target | Only selected session; uses existing editing lock semantics |
| Delta badges | `pose delta`, `layer delta`, `pull age delta`; no readiness/severity score |
| Camera compare | Viewpoint aid only; no camera command persistence |
| Export | No SA export or import action from comparison UI |

## 5. Diagnostic visual cohesion

V4 may consolidate background diagnostic signals into a common visual grammar:

| Signal | Display rule |
|--------|--------------|
| `last_pull` | Time-age chip; no health score |
| `stale` | Warning chip; no auto-remediation |
| `paused` | Neutral paused chip |
| `session_dirty` | Local mirror warning only; registry remains truth |
| `fidelity_truth` | Preserve F5b labels; never merge with explanatory terrain values |

## 6. Cesium/workstation evolution

PLAN-RT-V4 authorizes planning for:

- multi-session visual ergonomics
- cognition layout refinement
- layer-density strategy
- background diagnostic visual cohesion
- comparison-aware legends and density summaries

PLAN-RT-V4 does not authorize code changes in Cesium or workstation components.

## 7. Governance

| Boundary | V4 requirement |
|----------|----------------|
| Explanatory != authority | Required on all V4 visual surfaces |
| Advisory != authority | F8 advisory surfaces remain separate from V4 visualization |
| Registry truth | Entity registry and command handlers unchanged |
| Import semantics | No import, auto-import, or corpus commit actions |
| Browser authority | No browser->ROS or browser subprocess authority |
| SA | No SA contamination or live replay viewer hooks |

## 8. Explicit non-goals

- Bridge/runtime implementation
- Cesium code changes in PLAN
- SA viewer changes
- Tactical redesign
- Auto-import or federation writes
- Distributed runtime or multi-bridge
- Readiness scoring or operational picture language

## Related

- [rt_visual_density_management_v1.md](rt_visual_density_management_v1.md)
- [rt_visual_multi_session_cognition_v1.md](rt_visual_multi_session_cognition_v1.md)
- [rt_v4_visualization_realism_review_r1.md](rt_v4_visualization_realism_review_r1.md)
