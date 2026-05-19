# Static Replay Visualization R2 Plan

## Scope

`AGENTS.md` remains the primary authority. This document is **planning only**. It does not authorize implementation until a separate scoped wave is approved and frozen.

R2 extends frozen [Static Replay Visualization R1](replay_static_visualization_r1_plan.md) and [Comprehension R1](replay_static_visualization_comprehension_r1_freeze_audit.md). It does **not** modify frozen `replay_observability.py` narrative/observability builder contracts without an explicit observability wave.

**Prerequisite:** [Replay Demo & Review Workflow R1](replay_demo_review_workflow_r1.md) and [freeze_registry_r1.md](freeze_registry_r1.md) in use so deferred R1 items have a governed consumption context.

## Goal

Add optional, governance-contained visualization capabilities deferred from Comprehension R1:

1. Rosbag-derived trajectory overlays (continuous path semantics explicitly labeled)
2. Paired-run composite figures/HTML (matched-seed A vs B, localization-only)
3. Plotly opt-in for interactive static export (no live ROS)

## Allowed scope (when implemented)

- Evaluation-side Python under `scripts/evaluation/replay_viz_*` and `replay_static_visualization.py`
- Additive manifest fields on `replay_static_visualization_v1` (new optional blocks only)
- New `render_profile` value for R2 outputs (do not mutate Comprehension R1 profile in place)
- Tests in `test_replay_static_visualization.py`
- README and R2 freeze audit

## Forbidden scope

- Live dashboards, websockets, ROS subscriptions at runtime
- Runtime, launch, topic, schema, parser-contract changes
- Operator/HITL/tactical command UI, readiness scoring, causal AI explanations
- CI publishing of HTML as pass/fail gates without explicit non-certification labeling
- Replacing matplotlib defaults for all users (Plotly must remain opt-in)
- Unified “replay battle state” merging narrative + bag + live topics

## Feature specifications (planning)

### 1. Rosbag trajectory overlays

| Aspect | Requirement |
|--------|-------------|
| Input | Recorded bag paths referenced in sidecar meta or CLI `--bag-path` |
| Topics | Documented allowlist (e.g. `/drone/position`, `/tracks`) — no new subscriptions in runtime nodes |
| Geometry | Local scenario frame; dashed vs solid semantics for sparse log samples vs dense bag samples |
| Labeling | Banner: bag-derived, not continuous authority; not parser contract |
| Determinism | Fixed decimation/resample policy; seed not required for bag read order |
| Fallback | Skip figure with `skipped_figures` reason if bag missing |

**Planning questions for implementation wave:**

- Max points per figure for repo CI time bounds?
- Align bag timestamps to narrative `key_windows` or independent axis?

### 2. Paired-run composite

| Aspect | Requirement |
|--------|-------------|
| Input | Two `replay_narrative_v1` + observability JSON with matched-seed lineage |
| Layout | Side-by-side or stacked panels — same figure scale, no winner annotation |
| Copy | “Matched-seed comparison — localization only” in scan guide |
| Lint | Run governance-lint on both inputs before composite generation |
| Schema | New optional manifest block `paired_comparison` referencing left/right lineage |

### 3. Plotly opt-in

| Aspect | Requirement |
|--------|-------------|
| Default | Matplotlib PNG pipeline remains default |
| Flag | `--plotly` or manifest `interactive_export: plotly_html` |
| Output | Standalone HTML file with embedded data — no server |
| Governance | Same banner/caveats as composite HTML; no animation autoplay |
| Dependency | Optional extra; CI must pass without Plotly installed |

## Architecture (target)

```
replay_narrative_v1 (+ optional second narrative for paired)
  → replay_viz_comprehension.py (unchanged contract for C1)
  → replay_viz_figures.py (+ bag trajectory, paired panels)
  → replay_static_visualization.py (manifest R2 blocks)
  → replay_viz_html.py (+ optional plotly branch)
```

## CLI (proposed)

```bash
# Rosbag overlay (implementation wave)
python3 scripts/evaluation/replay_static_visualization.py composite \
  --narrative-json runs/evaluation/RUN.replay_narrative.json \
  --observability-json runs/evaluation/RUN.replay_observability.json \
  --bag-path runs/evaluation/RUN.mcap \
  --out-dir runs/evaluation/RUN.replay_viz/

# Paired composite (implementation wave)
python3 scripts/evaluation/replay_static_visualization.py paired-composite \
  --narrative-json-a ... --narrative-json-b ... \
  --observability-json-a ... --observability-json-b ... \
  --out-dir ...
```

Exact subcommand names are not frozen in this plan document.

## Freeze criteria (for future R2 implementation freeze)

R2 may be marked freeze-ready when:

- No runtime, launch, topic, schema, or parser-contract changes
- Comprehension R1 `render_profile` and manifest shape remain valid for non-R2 invocations
- New `render_profile` documents R2-only fields
- Rosbag and paired figures include non-authoritative labeling per [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md)
- Determinism tests pass for all new figure types
- `governance-lint` profile accepts R2 manifest strings
- `git diff --check` clean; scoped pytest green
- Freeze audit written; [freeze_registry_r1.md](freeze_registry_r1.md) row updated

## Phase ordering

| Step | Action |
|------|--------|
| 1 | This plan (documentation) |
| 2 | Optional: spike on bag decimation policy with one fixture bag under `runs/` (not committed) |
| 3 | Implementation wave per feature flag (bag → paired → plotly) |
| 4 | R2 freeze audit + registry update |

Do not start implementation before [situational_awareness_ui_planning_r1.md](situational_awareness_ui_planning_r1.md) authority boundaries are read for geospatial overlap.

## Related documents

- [replay_static_visualization_comprehension_r1_freeze_audit.md](replay_static_visualization_comprehension_r1_freeze_audit.md) — deferred items list
- [situational_awareness_ui_planning_r1.md](situational_awareness_ui_planning_r1.md) — geospatial/UI boundaries
- [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md) — demo consumption

## Plan status

**Planning only — not implemented.** No `render_profile` or code changes are authorized by this file alone.
