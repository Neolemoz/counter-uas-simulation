# Situational Awareness UI Planning R1

## Scope

`AGENTS.md` remains the primary authority. This document is **plan-only**. It does not authorize implementation of live UI, rosbridge dashboards, runtime topic changes, HITL workflows, or engagement authority.

Purpose: define governance-safe concepts for **replay-derived situational awareness** before any interactive UI code, and to bound future work relative to frozen static replay visualization.

References:

- [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md)
- [replay_static_visualization_comprehension_r1_freeze_audit.md](replay_static_visualization_comprehension_r1_freeze_audit.md)
- [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md)
- [replay_static_visualization_r2_plan.md](replay_static_visualization_r2_plan.md) — deferred trajectory/paired/plotly candidates
- Legacy [web/](../../web/) Cesium stack — **out of scope** until explicitly re-governed; treat as experimental, not aligned with frozen eval viz

## Goal

Specify how a future reviewer-facing SA presentation could layer **radar/map concepts**, **synchronized views**, and **entity boundaries** on top of existing replay artifacts — while preserving:

- mirrors ≠ authority
- replay-only default mode
- no operational command semantics
- no readiness or robustness scoring

## Non-goals

- Operational command and control (C2)
- Real-time battle management
- Live weapon-control or engagement authority
- Approval chains or autonomous engagement authority
- Runtime architecture redesign
- Military operational system breadth
- Causal AI explanation engines
- Readiness / certification / composite scoring systems

## Mode separation (required)

| Mode | Description | Default |
|------|-------------|---------|
| `replay_static` | All data from frozen logs, narratives, manifests, optional bags | **Yes — only mode authorized for R0 prototype** |
| `live_observation` | Would subscribe to ROS/live feeds | **Forbidden until separate HITL boundary wave** |

UI chrome must make mode explicit (banner, color band, or separate application). No toggle that blends replay-derived positions with live topics into one “current picture.”

## Authority boundary design

| Layer | UI may | UI must not |
|-------|--------|-------------|
| Parser-visible summaries | Display as labeled parser-local fields | Rewrite or extend parser contract |
| Replay narrative / observability JSON | Render sequence, incidents, windows | Treat as tactical orders or truth |
| Static viz manifest + comprehension | Reuse headline, scan_guide, figure order | Re-score or rank runs |
| Log-evidenced positions | Show sparse/dashed paths | Imply continuous track truth |
| Rosbag (future R2) | Overlay labeled bag-derived geometry | Present as live state |
| Engagement / selection | Show replay-local selection id timeline | Offer engage/disengage controls |

**Auto / manual separation (conceptual only):** Document that any future manual override UI belongs to a **future HITL boundary wave**, not SA Planning R1. SA R1 defines read-only mirrors only.

## View concepts

### Radar / map visualization

| Option | Use case | Governance default |
|--------|----------|-------------------|
| 2D top-down (ENU) | Primary replay map; aligns with `sparse_topdown` semantics | Sparse samples; dashed segments; “not continuous path” label |
| Range–bearing strip | Sensor-centric review | Derived from log/bag allowlist; no operational radar claim |
| Altitude panel | Secondary axis | Optional; not required for R0 |

**Planning decision for R0 prototype:** 2D top-down only, same frame as static viz sparse topdown, no geodetic datums or mission map tiles that imply deployed geography.

### Global / local synchronized views

- **Time index:** monotonic from log timestamps or narrative `key_windows` — single master clock.
- **Global:** full scenario extent, incident markers, divergence bands.
- **Local:** zoom to selected incident window; linked crosshair on timeline.
- **No** “unified battle state” object — views are projections of the same derived artifacts.

### Interceptor / target visualization

| Entity | Source | Visual rules |
|--------|--------|--------------|
| Target | Log/bag position samples | Neutral color; no threat score glyph |
| Interceptor | Log/bag + engagement metrics series | Distinct layer; non-authoritative |
| Selection id | Replay-local tactical evidence | Label “replay-local”; no assign command |

Do not imply identification confidence, classification authority, or WEZ validity.

### Camera feed synchronization

| Status | Notes |
|--------|-------|
| **Not in R0** | No media contract in `replay_narrative_v1` or comprehension manifest |
| Future | Optional thumbnail strip synced to master clock from recorded image topics |
| Risk | Camera lag must not be read as engagement truth |

Document gap explicitly in any prototype: “camera sync not implemented.”

## Governance-safe UI layering

Recommended stack (bottom to top):

1. **Governance banner** — derived replay summary; not authority (reuse comprehension `scan_guide` text where possible)
2. **At-a-glance strip** — taxonomy labels only
3. **Timeline scrubber** — master clock; incident markers
4. **Map panel** — 2D replay_static geometry
5. **Figure strip** — thumbnails linking to static PNG assets or inline comprehension panel
6. **Lineage drawer** — paths, seed, profile_id, warnings
7. **Caveats footer** — causal, readiness, HITL negations

Optional R2 additions (see [replay_static_visualization_r2_plan.md](replay_static_visualization_r2_plan.md)): bag trajectories, paired-run split map, Plotly scrubbing — each with its own sub-banner.

## Data contract (planning)

R0 prototype should consume **only**:

- `replay_static_visualization.json` manifest + comprehension block
- `replay_narrative_v1` JSON
- `single_run_replay_observability_report` JSON (optional, for divergence/lifecycle detail)
- PNG assets referenced by manifest paths

No new runtime topics. No websocket to ROS.

## Relationship to existing web/

The [web/](../../web/) directory contains Cesium + rosbridge experiments. SA Planning R1 does **not** extend that stack. A future implementation wave must either:

- fork a new replay-only static viewer under evaluation governance, or
- explicitly re-govern `web/` with a freeze audit — not assumed here.

## Future HITL boundary concepts (plan-only pointers)

Reserved for **HITL Boundary Concepts R0** (separate plan wave):

- Manual vs automated engagement lanes (display separation only)
- No approval-chain semantics in SA R1
- No operator workload or readiness metrics

## Suggested implementation phases (after this plan freezes)

| Phase | Content | Type |
|-------|---------|------|
| SA-R0 | Static HTML/JS mock reading manifest JSON only | optional spike |
| VIZ-R2 | Bag trajectory, paired composite, Plotly | per [replay_static_visualization_r2_plan.md](replay_static_visualization_r2_plan.md) |
| SA-R1 impl | Replay-only map + timeline sync | implementation + audit |
| HITL-R0 | Boundary concepts doc | plan only |

## Freeze boundaries for future UI implementation

When an implementation wave is authorized, it must freeze:

| Boundary | Intent |
|----------|--------|
| Data source | Replay artifacts + parser summaries only in R0 |
| Authority | Read-only; no writes to engagement/selection |
| Modes | `replay_static` vs `live_observation` mutually exclusive chrome |
| Sync | Log timestamp index only |
| Geospatial | Local ENU / scenario frame; no operational map datums |
| Entities | Log-evidenced positions; dashed = non-continuous |
| Engagement viz | `[METRICS]` / markers — non-authoritative |
| Comprehension | Reuse digest fields; no derived scoring |
| Tech stack | Plotly/WebGL opt-in; no default rosbridge in eval CI |
| Governance | Banner + scan_guide on every page |

## “Do not cross yet” (SA context)

- Live ROS subscriptions in eval or demo tooling
- Websockets to runtime
- Multi-user C2 layouts
- Weapon release or approval UI
- Readiness / field performance claims
- Causal mechanism narratives beyond localization language
- Merging manifest into runtime node parameters
- CI gates on SA HTML without non-certification labels

## Validation strategy (for future implementation)

- Governance lint on source JSON before render
- Deterministic render tests (fixed manifest fixture)
- Visual regression optional; not required for R0 mock
- Manual review against [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) checklist

## Plan status

**Plan-only — not implemented.** Freeze of this planning document is recorded in [situational_awareness_ui_planning_r1_freeze_audit.md](situational_awareness_ui_planning_r1_freeze_audit.md).
