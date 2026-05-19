# Demo case: topology_spread

## What this demonstrates

How outcome and divergence labels vary across seeds under one profile — spread description, not ranking.

## What reviewers should look for

- Shared `profile_id` across multiple single-run reports or topology-index rows
- Distribution of success, D-class, A-class, and F-class labels across seeds
- Launch args, geometry, fragmentation fields preserved per row
- Missing provenance rows — surface warnings, do not impute
- Index output treated as derived evaluation artifact

## What NOT to conclude

- “Best seed” or optimal operating point
- Certified operating region from index shorthand
- Runtime topology truth from profile labels
- Robustness proof from spread width alone

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| topology-index JSON | Cohort-level spread table |
| at_a_glance (sample runs) | Anchor high/low spread examples |
| lineage per row | profile id, geometry, notes |
| Optional: 2–3 replay_viz HTML samples | Illustrate spread endpoints only |
| governance-lint on index JSON | Wording check before share |

## Governance caveats

- Topology index entries are lineage-linked shorthand — not runtime topology semantics.
- Describe distributions; avoid league-table framing.

## Suggested run source

- `replay_observability.py topology-index` over a fixture CSV (Wave 6 surface)
- Multiple single-run reports sharing `profile_id`

## Fixture hints

- `scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave6_transferability.csv` or adjacent wave CSVs
- Local summary: `runs/evaluation/ambiguity_sweep_wave6_transferability.csv` if present

## README_case.md (copy to demo bundle)

1. Topology index entries are **derived lineage-linked shorthand** — not runtime topology truth.
2. Describe distribution of labels across seeds; avoid “best seed” language.
3. Preserve profile id, launch args, geometry, and notes in every cited row.
4. Missing provenance in any row → surface warning, do not impute.
5. Close: exploratory spread view; not operating region certification.
