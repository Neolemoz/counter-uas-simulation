# Demo case: near_miss

## What this demonstrates

Threshold sensitivity: outcome near the Phase 0 evaluation gate without framing the run as a dramatic “failure.”

## What reviewers should look for

- `min_miss_m` or equivalent parser-visible miss distance near `hit_threshold_m`
- F-class or near-threshold outcome wording in at_a_glance
- Outcome labels in observability parser-visible summary — not re-derived drama
- Whether divergence or ambiguity incidents co-occur (association only)

## What NOT to conclude

- Operational ineffectiveness or “system failure”
- Root cause in guidance, sensor, fusion, or tracker without separate governed analysis
- That a near-miss under one seed predicts behavior under other seeds
- Causal link between any single incident group and the miss distance

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| at_a_glance | Miss distance / outcome taxonomy |
| incident_groups | Co-occurring events — localization only |
| timeline_band | Where threshold pressure appears in sequence |
| divergence_overlay | Check for late-stage mismatch if present |
| lifecycle_strip | Churn near outcome window |
| lineage | Confirm threshold-relevant launch args |

## Governance caveats

- Prefer parser-visible miss fields over narrative re-interpretation.
- Compare to `clean_hit` only with matched seed — never rank systems.

## Suggested run source

- Capture run with `min_miss_m` close to `hit_threshold_m` per [research/phase0_baseline_metrics.md](../../../research/phase0_baseline_metrics.md)
- May show F-class taxonomy in evaluation row

## Fixture hints

- Filter MC CSV for runs with small margin to threshold
- Narrative should highlight miss distance, not causal mechanism

## README_case.md (copy to demo bundle)

1. This run is **near the evaluation threshold** — not proof of operational ineffectiveness.
2. Use **min_miss** / outcome wording from parser-visible fields only.
3. Do not infer guidance, sensor, or tracker root cause from replay artifacts alone.
4. Compare to `clean_hit` only with matched seed if discussing sensitivity — never rank “better system.”
5. Close: localization of outcome labels; not causal proof.
