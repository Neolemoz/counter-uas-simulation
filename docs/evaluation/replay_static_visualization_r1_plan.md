# Static Replay Visualization R1 Plan

## Scope

`AGENTS.md` remains the primary authority. This phase implements the deferred Phase 2 Wave 4 static visual bands/overlays ([replay_narrative_tooling_r1_freeze_audit.md](replay_narrative_tooling_r1_freeze_audit.md)) as a separate, consume-only evaluation-side visualization module.

Static Replay Visualization R1 improves replay readability and reviewer comprehension through deterministic static figures and HTML built from frozen `replay_narrative_v1` and `replay_observability_v1` artifacts. It is not a live dashboard, runtime visualization, operator/HITL surface, or frontend platform.

Reference boundaries:

- [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md)
- [replay_observability_freeze_audit.md](replay_observability_freeze_audit.md)
- [replay_narrative_tooling_r1_freeze_audit.md](replay_narrative_tooling_r1_freeze_audit.md)

## Goal

Emit deterministic static timeline bands, divergence overlays, lifecycle strips, summary cards, and composite HTML that help reviewers localize events and divergence windows without collapsing derived artifacts into authority.

## Allowed scope

- Evaluation-side rendering from `replay_narrative_v1` + `replay_observability_v1` JSON
- Optional log re-read via copied lineage only (sparse `[METRICS]`, position debug lines)
- Static PNG/SVG figures and standalone HTML panels
- Opt-in Plotly HTML (CDN-pinned, file-only)
- Governance block, lineage, warnings, interpretation caveats on every manifest and HTML page

## Forbidden scope

- Live dashboards, websockets, ROS subscriptions, bag-playback UI
- Runtime/launch/topic/schema/parser changes
- Operator/HITL/tactical command surfaces, readiness scoring, AI causal explanations
- RViz / runtime visualization package changes
- Edits to frozen narrative/observability builder contracts in `replay_observability.py`

## Static Viz waves

| Wave | Name | Deliverable |
|------|------|-------------|
| 0 | Documentation scope | This plan + README pointer |
| 1 | Manifest contract | `replay_static_visualization_v1` manifest builder |
| 2 | Timeline bands | Horizontal categorical event-band SVG |
| 3 | Divergence + lifecycle | Shaded mismatch windows; lifecycle strip |
| 4 | Engagement series (opt-in) | dist/t_go series from `[METRICS]` |
| 5 | Sparse top-down (opt-in) | XY scatter when log-evidenced |
| 6 | Composite + freeze | Composite HTML, governance lint, freeze audit |

R1 default ship: Waves 0–3 + 6. Waves 4–5 are evidence-gated opt-in.

## Architecture

```
scripts/evaluation/
  replay_static_visualization.py   # manifest builder + CLI
  replay_viz_figures.py            # pure figure functions
  replay_viz_html.py               # composite HTML assembler
```

Consumes frozen JSON only; does not modify `replay_observability.py` narrative builder.

## CLI example

```bash
python3 scripts/evaluation/replay_observability.py narrative \
  --single-run-json runs/evaluation/RUN.replay_observability.json \
  --out-json runs/evaluation/RUN.replay_narrative.json

python3 scripts/evaluation/replay_static_visualization.py composite \
  --narrative-json runs/evaluation/RUN.replay_narrative.json \
  --observability-json runs/evaluation/RUN.replay_observability.json \
  --out-dir runs/evaluation/RUN.replay_viz/
```

Optional engagement and sparse top-down figures:

```bash
python3 scripts/evaluation/replay_static_visualization.py composite \
  --narrative-json runs/evaluation/RUN.replay_narrative.json \
  --out-dir runs/evaluation/RUN.replay_viz/ \
  --engagement-series \
  --sparse-topdown
```

## Freeze boundaries

Freeze when Waves 0–3 + 6 are stable with determinism tests green.

Frozen at R1: manifest schema, figure categories, CLI subcommands, governance/anti-claims, test fixtures.

Deferred past R1: rosbag trajectories, paired-run composites, Plotly as default, animations/GIF, CI artifact publishing.
