# Static Replay Visualization R1 Freeze Audit

## Scope

This audit covers the Static Replay Visualization R1 implementation:

- [`scripts/evaluation/replay_static_visualization.py`](../../scripts/evaluation/replay_static_visualization.py) — manifest builder, figure orchestration, governance lint, CLI
- [`scripts/evaluation/replay_viz_figures.py`](../../scripts/evaluation/replay_viz_figures.py) — deterministic figure rendering
- [`scripts/evaluation/replay_viz_html.py`](../../scripts/evaluation/replay_viz_html.py) — composite static HTML assembler
- [`src/counter_uas/test/test_replay_static_visualization.py`](../../src/counter_uas/test/test_replay_static_visualization.py) — determinism and governance tests
- [`src/counter_uas/test/fixtures/replay_narrative_minimal.json`](../../src/counter_uas/test/fixtures/replay_narrative_minimal.json) — minimal narrative fixture
- [`scripts/evaluation/README.md`](../../scripts/evaluation/README.md) — CLI examples (visualization section only)
- [`docs/evaluation/replay_static_visualization_r1_plan.md`](replay_static_visualization_r1_plan.md)

`AGENTS.md` remains the primary governance authority. Frozen replay observability and narrative tooling R1 contracts were not modified.

## Governance Result

**Verdict: freeze-ready** for the scoped Static Replay Visualization R1 layer.

The tooling adds `replay_static_visualization_v1` (`artifact_type: replay_static_visualization_manifest`) as a derived, non-authoritative evaluation artifact built from frozen `replay_narrative_v1` and optional `replay_observability_v1` JSON. Outputs are deterministic, static, and replay-derived. Figures are explanatory visualization layers only; they are not causal proof, parser contracts, tactical authority, lifecycle truth, governance approval, or readiness evidence.

## Boundary Checks

- **Authority creep:** no new authority surface. Manifest and HTML reject unified authoritative replay-state interpretation.
- **Parser safety:** no parser-visible fields, topics, schemas, or parser contracts changed. Consumes frozen JSON only.
- **Runtime isolation:** evaluation-side Python only; optional log re-read via copied lineage; no ROS/Gazebo runtime coupling, websockets, or live dashboards.
- **Lifecycle semantics:** lifecycle strip remains an explanatory overlay; not tracker lifecycle truth or robustness proof.
- **Divergence semantics:** divergence shading localizes replay-side disagreement; explicitly non-causal.
- **Trajectory semantics:** sparse top-down and engagement series are evidence-gated; skipped when log evidence absent.
- **Readiness / operator:** no operational-readiness, hardware-readiness, HITL/operator, certification, or composite robustness scoring semantics.

## Artifact Contract (frozen at R1)

- `artifact_type: replay_static_visualization_manifest`
- `visualization_schema_version: replay_static_visualization_v1`
- `render_profile: static_viz_r1_v1`
- governance block, copied lineage, `source_artifacts`
- `figures[]` with stable `figure_id`, category, relative path, caveat
- `skipped_figures[]` when optional evidence is absent
- `summary`, `warnings`, `interpretation_caveats`

Figure categories at R1 freeze:

- `timeline_band`
- `divergence_overlay`
- `lifecycle_strip`
- `engagement_series` (opt-in)
- `sparse_topdown` (opt-in)
- composite HTML report

## Deferred (out of R1 freeze scope)

- Rosbag-derived continuous trajectories
- Paired-run side-by-side composite layouts
- Plotly as default output
- Animation/GIF replay playback
- CI artifact publishing integration

## Regression Evidence

Commands run during R1 isolation:

```bash
python3 -m compileall -q scripts/evaluation/replay_static_visualization.py scripts/evaluation/replay_viz_figures.py scripts/evaluation/replay_viz_html.py
python3 -m pytest src/counter_uas/test/test_replay_static_visualization.py -q
python3 -m pytest src/counter_uas/test/test_replay_observability.py -q
```

Results:

- compile check: passed
- static visualization focused tests: 6 passed
- replay observability regression: 11 passed

## Freeze Conditions Met

- no runtime, launch, topic, schema, parser-contract, tracker, fusion, or tactical authority changes
- visualization tooling remains evaluation-side, additive, deterministic, and non-authoritative
- composite HTML has no live runtime coupling
- governance lint enforces visualization schema and anti-claim caveats
- optional figures skip cleanly when evidence is missing

## Post-Freeze Continuation

Permitted without reopening R1: rosbag trajectory overlays, paired composites, Plotly opt-in expansion, CI publishing hooks as separate scoped waves.

This audit does **not** authorize runtime redesign, parser/schema/topic changes, live dashboards, operator/HITL semantics, readiness scoring, AI causal explanation systems, governance automation expansion, or realism/runtime capability expansion.
