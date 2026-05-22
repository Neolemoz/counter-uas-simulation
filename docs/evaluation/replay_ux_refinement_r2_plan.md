# Replay UX Refinement R2 Plan

## Scope

`AGENTS.md` remains the primary authority. This wave refines **reviewer comprehension and replay-review usability** on the frozen Comprehension R1 platform. It is **not** [Static Replay Visualization R2](replay_static_visualization_r2_plan.md) (rosbag overlays, paired-run composites, Plotly).

**Prerequisite:** frozen [Comprehension R1](replay_static_visualization_comprehension_r1_freeze_audit.md) and [Replay Demo & Review Workflow R1](replay_demo_review_workflow_r1.md).

## Goal

Address validated usability friction from replay demo review:

1. Outcome wording consistency (hit vs miss)
2. Near-miss visibility (`min_miss_m`, threshold margin)
3. Selection-event flooding reduction
4. Long-log readability (salient-first timeline)
5. Divergence wording (D5 vs localized mismatch)

## Allowed scope

- `scripts/evaluation/replay_viz_comprehension.py`
- `scripts/evaluation/replay_viz_html.py`
- `scripts/evaluation/replay_viz_figures.py`
- `scripts/evaluation/replay_static_visualization.py`
- `src/counter_uas/test/test_replay_static_visualization.py`
- Fixture and evaluation docs

## Forbidden scope

- `replay_observability.py` / `replay_narrative_v1` builder changes
- Parser/schema/topic/runtime changes
- Live dashboards, websockets, HITL, readiness scoring, causal AI
- New artifact schema versions (additive `comprehension` keys only)
- Merging with `PLAN-VIZ-R2` feature work

## Contract

- `render_profile: static_viz_ux_refinement_r2_v1`
- Additive comprehension fields: `outcome_context`, `selection_summary`, `divergence_context`, `timeline_rows_salient`, `timeline_rows_full`, `timeline_overflow`, `selection_detail_rows`
- Frozen narrative JSON labels on disk unchanged; display normalization at digest render time

## Validation

```bash
python3 -m compileall -q scripts/evaluation/replay_static_visualization.py scripts/evaluation/replay_viz_*.py
python3 -m pytest src/counter_uas/test/test_replay_static_visualization.py -q
git diff --check
```

Manual: re-run demo cases `clean_hit`, `near_miss`, `selection_oracle_divergence`, `governance_warnings` per [demo_cases/](demo_cases/).
