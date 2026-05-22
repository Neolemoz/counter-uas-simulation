# Demo case: clean_hit

## What this demonstrates

A straightforward parser-visible success path with low ambiguity and no selection/oracle divergence (typically D0). Use as the **first** demo case for new reviewers.

## What reviewers should look for

- Parser-visible `success` / hit outcome in observability summary
- D0 or low D-class divergence label in at_a_glance
- Low or empty incident_groups for divergence and ambiguity
- Clean lineage: seed, cohort, git state, launch args present
- Lifecycle strip — note whether counters are active or dormant

## What NOT to conclude

- Field performance or operational effectiveness
- Robustness under other seeds, profiles, or realism overlays
- Tracker maturity from absent lifecycle churn
- That absence of mismatch proves selection/oracle agreement everywhere

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| at_a_glance | Outcome labels only |
| comprehension_panel | Confirm low incident density |
| timeline_band | Baseline event sequence |
| divergence_overlay | Expect empty or D0 |
| lifecycle_strip | State dormancy explicitly if flat |
| lineage | Seed and launch args for reproducibility |

## Governance caveats

- Derived replay summary only — see [reviewer_interpretation_guide.md](../reviewer_interpretation_guide.md).
- Dormant lifecycle counters are not proof of tracker robustness ([AGENTS.md](../../AGENTS.md)).

## Suggested run source

- `ros2 launch counter_uas bringup.launch.py` with default `config.yaml`
- Low noise / baseline MC seed from standard capture workflow
- See [research/phase0_baseline_metrics.md](../../../research/phase0_baseline_metrics.md) for pass semantics

## Fixture hints

- No realism overlay required
- Optional: baseline row from standard Monte Carlo CSV

## README_case.md (copy to demo bundle)

1. This run illustrates a **parser-visible hit** under baseline conditions — not field performance.
2. Read **at_a_glance** for outcome labels only; they are derived summaries.
3. Expect low or no divergence incidents; absence of mismatch is not proof of robustness elsewhere.
4. If lifecycle counters are zero, state the dormancy caveat — do not claim tracker maturity.
5. Close: derived replay summary; not readiness or certification.
