# Demo case: paired_seed_delta

## What this demonstrates

Matched-seed comparability: same `noise_seed`, different profiles or conditions. G-buckets from `pair_mc_seed_outcomes.py` are descriptive only.

## What reviewers should look for

- Identical seed in lineage for both runs (meta notes or observability bundle)
- Outcome label delta, divergence class delta, lifecycle churn delta
- `paired-comparison` JSON or two observability reports side by side
- Warnings if seed lineage is missing on either side — **stop if absent**
- G-bucket labels as cohort shorthand, not scores

## What NOT to conclude

- Statistical superiority or a “winning” profile
- Robustness certification from one seed pair
- Causal explanation for outcome differences
- That G-buckets imply ranking or readiness

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| at_a_glance (both runs) | Outcome and taxonomy delta |
| divergence_overlay (both) | Mismatch timing differences |
| lifecycle_strip (both) | Churn comparison |
| incident_groups | Block-level localization |
| paired-comparison JSON | Structured delta view |
| lineage (both) | Seed match verification — mandatory |

## Governance caveats

- Matched-seed buckets are comparability aids — [reviewer_interpretation_guide.md](../reviewer_interpretation_guide.md).
- Never present hit-rate deltas without seed context and dormancy caveats.

## Suggested run source

- Two MC rows or two single-run observability JSONs with documented same seed
- `replay_observability.py paired-comparison`

## Fixture hints

- Join CSVs with `pair_mc_seed_outcomes.py` before observability if building cohort view
- Lineage must show seed source and warnings

## README_case.md (copy to demo bundle)

1. Comparison is **matched-seed localization** — not statistical superiority.
2. Describe outcome/divergence differences; do not name a winner.
3. Confirm seed appears in lineage for both sides; if missing, stop and warn.
4. G-buckets are cohort shorthand — not robustness scores.
5. Close: comparability aid only; not certification.
