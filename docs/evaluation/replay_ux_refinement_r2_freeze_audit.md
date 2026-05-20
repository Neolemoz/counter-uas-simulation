# Replay UX Refinement R2 Freeze Audit

## Scope

Display-layer polish on frozen Comprehension R1:

- [`scripts/evaluation/replay_viz_comprehension.py`](../../scripts/evaluation/replay_viz_comprehension.py)
- [`scripts/evaluation/replay_viz_html.py`](../../scripts/evaluation/replay_viz_html.py)
- [`scripts/evaluation/replay_viz_figures.py`](../../scripts/evaluation/replay_viz_figures.py)
- [`scripts/evaluation/replay_static_visualization.py`](../../scripts/evaluation/replay_static_visualization.py)
- [`src/counter_uas/test/test_replay_static_visualization.py`](../../src/counter_uas/test/test_replay_static_visualization.py)

Frozen: `replay_observability.py`, `replay_narrative_v1`, parser contracts, runtime.

## Governance Result

**Verdict: freeze-ready** for Replay UX Refinement R2.

`render_profile` is `static_viz_ux_refinement_r2_v1`. Outputs remain derived, static, deterministic, and non-authoritative.

## Usability fix matrix

| Finding | Change | Demo case |
|---------|--------|-----------|
| Outcome wording ambiguity | `display_outcome_label()` normalizes hit/miss across at_a_glance, incidents, salient timeline | `clean_hit`, `near_miss` |
| Near-miss visibility | `outcome_context` + at_a_glance margin cards; log parse for threshold; engagement figure threshold line | `near_miss` |
| Selection flooding | `selection_summary` + collapsed selection incidents; `selection_detail_rows` appendix; capped divergence markers | `selection_oracle_divergence` |
| Long-log readability | `timeline_rows_salient` + `timeline_overflow`; salient row CSS | all long logs |
| Divergence confusion | `DIVERGENCE_CLASS_LABELS`, `divergence_context.note`, deduped D5 warning row | `selection_oracle_divergence`, `governance_warnings` |

## Remaining reviewer friction

- Frozen narrative JSON on disk may still contain legacy labels until re-exported; composites normalize at digest time.
- Richest mismatch counts when `--observability-json` is passed to composite (trace summary fields).
- `hit_threshold_m` appears when present in evaluation row or log parse; not added to `CANONICAL_SUMMARY_FIELDS`.

## Readiness (post-R2)

| Level | Assessment |
|-------|------------|
| Mentor-ready | Yes — summary cards, collapsed selection, D5 coexistence note |
| Onboarding-ready | Yes with demo runbook; first-time reviewers still need D-class glossary in scan_guide |
| Self-serve-ready | Partial — pass `--observability-json` for full mismatch aggregation |

**Next wave:** only if demo review finds gaps; do not merge with `PLAN-VIZ-R2`.

## Regression Evidence

```bash
python3 -m compileall -q scripts/evaluation/replay_static_visualization.py scripts/evaluation/replay_viz_*.py
python3 -m pytest src/counter_uas/test/test_replay_static_visualization.py -q
git diff --check
```

Results:

- compile check: passed
- static visualization tests: 14 passed
- `git diff --check`: clean

## Additive comprehension fields

- `outcome_context`, `selection_summary`, `divergence_context`
- `timeline_rows_salient`, `timeline_rows_full`, `timeline_overflow`
- `selection_detail_rows`
- `timeline_rows` aliases salient rows for backward compatibility

## Boundary checks

- No parser/schema/topic/runtime changes
- No operational failure framing for near-miss margin
- No HITL, readiness, or causal mechanism claims
