# Demo case: selection_oracle_divergence

## What this demonstrates

Replay-local disagreement between selection evidence and oracle expectation (D1–D5). Wave 7 frozen fields localize mismatch blocks — they do not assign tactical authority.

## What reviewers should look for

- Non-D0 `selection_oracle_divergence_class` in at_a_glance
- Mismatch block indices in incident_groups and divergence_overlay
- `first_selection_mismatch_block`, `selection_mismatch_count` in observability (frozen fields)
- Whether mismatch is adjacent to fragmented-gap windows — **association language only**
- Selection narrative events — mirrors, not `/interceptor/selected_id` replacement

## What NOT to conclude

- Operator should have selected differently
- Causal mechanism (“the gap caused the mismatch”)
- Robustness ranking between profiles or seeds
- That divergence class replaces tactical topic semantics

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| at_a_glance | D-class label |
| incident_groups | Block indices for log re-read |
| divergence_overlay | Primary teaching figure |
| timeline_band | Temporal context for mismatch blocks |
| lifecycle_strip | Co-occurring churn — not causal |
| observability JSON | Frozen divergence field values |

## Governance caveats

- Divergence taxonomy is evidence-only — see Wave 7 notes in [README.md](../../../README.md).
- Fragmented-gap adjacency is “associated with,” not “caused by.”

## Suggested run source

- Wave 7 scenarios documented in [README.md](../../../README.md) and [scripts/evaluation/README.md](../../../scripts/evaluation/README.md)
- Runs with non-D0 `selection_oracle_divergence_class`

## Fixture hints

- `classify_selection_oracle_divergence.py` on log before observability if validating taxonomy
- Frozen fields: `first_selection_mismatch_block`, `selection_mismatch_count`, etc.

## README_case.md (copy to demo bundle)

1. Divergence class labels are **replay-local taxonomy** — not operator commands or causal proof.
2. Use **incident_groups** and divergence overlay to localize timing only.
3. Fragmented-gap adjacency is association language, not “the gap caused the mismatch.”
4. Do not reinterpret `/interceptor/selected_id` beyond documented replay scope.
5. Close: evidence-side localization; not robustness ranking.
