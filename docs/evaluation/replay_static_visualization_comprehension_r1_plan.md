# Static Replay Visualization Comprehension R1 Plan

## Scope

`AGENTS.md` remains the primary authority. This phase extends the frozen [Static Replay Visualization R1](replay_static_visualization_r1_plan.md) layer with reviewer-facing comprehension improvements only. It does not modify `replay_observability.py`, parser contracts, runtime behavior, or the R1 freeze audit.

Comprehension R1 improves how quickly a non-author reviewer can understand a replay report (~30 seconds) through derived digest blocks, clearer figure labels, reordered HTML sections, and an optional stacked comprehension panel. Outputs remain deterministic, static, replay-derived, and non-authoritative.

Reference boundaries:

- [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md)
- [replay_static_visualization_r1_freeze_audit.md](replay_static_visualization_r1_freeze_audit.md)

## Goal

Make composite static replay HTML and figures scannable without internal architecture knowledge, while preserving governance banners, caveats, and anti-claims.

## Allowed scope

- `scripts/evaluation/replay_viz_comprehension.py` (new)
- Additive changes to `replay_viz_figures.py`, `replay_viz_html.py`, `replay_static_visualization.py`
- Optional top-level `comprehension` object on `replay_static_visualization_v1` manifests
- `render_profile: static_viz_comprehension_r1_v1`
- Tests, README, comprehension freeze audit

## Forbidden scope

- Live dashboards, websockets, ROS subscriptions, runtime/launch/topic/schema changes
- Operator/HITL/tactical command surfaces, readiness scoring, AI causal explanations
- Edits to frozen narrative/observability builder contracts in `replay_observability.py`
- Frontend framework expansion (React/Vue), animations, rosbag continuous trajectories

## Deliverables

| Wave | Deliverable |
|------|-------------|
| 1 | `build_comprehension_digest()` — headline, scan guide, incidents, timeline rows |
| 2 | Figure polish + `comprehension_panel` stacked PNG |
| 3 | Restructured composite HTML (read-first → figures → lineage appendix) |
| 4 | Auto-enable optional figures when `log_path` exists; `--no-optional-figures` |
| 5 | Tests, freeze audit, README pointer |

## Architecture

```
replay_narrative_v1 JSON
  → replay_viz_comprehension.py (digest)
  → replay_viz_figures.py (PNG)
  → replay_static_visualization.py (manifest)
  → replay_viz_html.py (composite HTML)
```

## CLI

Same as R1; optional figures auto-enable when lineage `log_path` is readable unless `--no-optional-figures`:

```bash
python3 scripts/evaluation/replay_static_visualization.py composite \
  --narrative-json runs/evaluation/RUN.replay_narrative.json \
  --observability-json runs/evaluation/RUN.replay_observability.json \
  --out-dir runs/evaluation/RUN.replay_viz/
```

## Freeze boundaries

Frozen at Comprehension R1: `comprehension` manifest block shape, `render_profile`, HTML section order, figure display order, governance lint profile string.

Deferred: rosbag trajectories, paired-run composites, Plotly default, animations, CI publishing.
