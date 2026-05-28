# RT Visual Density Management Contract (`rt_visual_density_management_v1`)

**Phase:** PLAN-RT-V4 - visualization density planning  
**Authority:** [rt_v4_visualization_fidelity_plan.md](../platform/rt_v4_visualization_fidelity_plan.md)  
**Companion:** [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md)

Planning contract for density management in RT visualization. Density policy is a display policy only; it does not change telemetry, registry, command, capture, or import semantics.

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Scope | Future RT UI display declutter and layer budget |
| Authority | None - display policy only |
| Default | Prefer hiding explanatory decoration before hiding selected entity context |
| Persistence | Local UI preference only; no bridge persistence |
| Import/export | No SA import/export semantics |

## 1. Density classes

| Class | Examples | Priority |
|-------|----------|----------|
| `authority_context` | selected session id, command target marker, editing lock label | Highest display priority, but still not new authority |
| `entity_context` | selected entity marker, bounds, command ghost labels | High |
| `terrain_context` | ridges, contours, elevation bands, terrain labels | Medium |
| `visibility_context` | wedge, horizon, LOS corridors, occlusion corridors | Medium-low |
| `diagnostic_context` | pull age, stale, paused, background rows | Medium |
| `decorative_context` | nonessential labels, legends, secondary swatches | Lowest |

## 2. Declutter rules

| Condition | Required policy |
|-----------|-----------------|
| Entity labels overlap | Preserve selected entity label; compress or hide nonselected labels |
| Terrain labels overlap | Prefer clustered legend row over map labels |
| Visibility overlays exceed budget | Collapse into one summary corridor or disable lowest-priority overlay |
| Background sessions exceed visual budget | Show summary rows before globe overlays |
| Diagnostic chips exceed width | Collapse to count + worst non-authoritative state |

## 3. Layer budget

| Budget | PLAN-RT-V4 target |
|--------|-------------------|
| Active overlay groups | 5 visible groups before warning |
| Entity labels | Selected + nearest 12 before label thinning |
| Terrain labels | 8 visible labels before clustering |
| Visibility corridors | 3 per active session before summary mode |
| Comparison sessions | Active + 2 comparison rows; no distributed sessions |

Budget warnings must use wording such as:

```text
Layer density high - some explanatory labels are summarized.
```

They must not use readiness, health, operational, or failure scoring language.

## 4. Legend strategy

| Legend type | Rule |
|-------------|------|
| Layer legend | Group by terrain, visibility, sensor, diagnostics, comparison |
| Session legend | Use V1/M3 session accents; active session first |
| Comparison legend | Show `selected`, `comparison`, `background`; no command affordance |
| Fidelity legend | Preserve F5b `truth_attested` and `explanatory` labels |
| Advisory legend | Do not merge F8 advisory cohorts into V4 visual density |

## 5. Background diagnostic cohesion

Background diagnostics may be summarized visually when the workstation is dense:

| Summary | Meaning |
|---------|---------|
| `2 background` | Count of nonselected local sessions |
| `1 stale` | Pull staleness display only |
| `paused` | User-paused background pull state |
| `dirty mirror` | Local mirror differs from latest observed state; registry remains authority |

No summary may trigger reset, import, capture, or command execution.

## 6. PLAT advisory anchors

Future implementation may add helpers under existing RT UI modules only after PLAT authorization:

| Advisory anchor | Purpose |
|-----------------|---------|
| `visualDensityPolicy` | Priority and budget calculation |
| `LayerDensitySummary` | Text/legend rollup |
| `ComparisonLegendStrip` | Session compare legend |
| `BackgroundDiagnosticCohesionStrip` | Compact diagnostic summary |

These anchors are not created by PLAN-RT-V4.

## 7. Explicit non-goals

- Bridge or telemetry changes
- SA viewer density policy
- Parser/schema changes
- Auto-import or federation writes
- Distributed runtime support
- Operational readiness or health scoring

## Related

- [rt_visualization_fidelity_v4.md](rt_visualization_fidelity_v4.md)
- [rt_visual_multi_session_cognition_v1.md](rt_visual_multi_session_cognition_v1.md)
