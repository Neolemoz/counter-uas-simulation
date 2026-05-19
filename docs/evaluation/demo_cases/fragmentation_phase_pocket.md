# Demo case: fragmentation_phase_pocket

## What this demonstrates

Cadence and **phase** sensitivity (Wave 5). The `cycle=7 gap=4 phase=1` region is an instability pocket in tested geometry — not a certified operating boundary.

## What reviewers should look for

- Profile id and `fragmentation_stagger_phase_ticks` in lineage
- Fragmentation / gap windows on timeline_band
- Outcome and churn differences vs phase=3 variant under matched seed
- Lifecycle strip activity — counters may still be dormant
- Topology labels as lineage shorthand only

## What NOT to conclude

- Certified fielded operating limits or deployment recommendations
- Monotonic “stronger silence is better” from Wave 5 evidence
- That phase pocket generalizes outside tested geometry (`target_start_x_m:=-1500`, etc.)
- Causal proof that phase alone determined outcome

## Recommended replay sections and figures

| Section / figure | Focus |
|------------------|-------|
| timeline_band | Gap/cadence windows |
| lifecycle_strip | Phase-sensitive churn |
| at_a_glance | Outcome label under pocket profile |
| incident_groups | Fragmentation event localization |
| lineage | phase ticks and profile id — required |
| Optional paired HTML | Side-by-side with phase=3 under same seed |

## Governance caveats

- Wave 5 safe bounded region is documented in [scripts/evaluation/README.md](../../../scripts/evaluation/README.md) — cite as tested envelope, not certification.
- Phase comparisons require matched seed and explicit profile ids in cohort names.

## Suggested run source

- Wave 5 fixture profiles in `scripts/evaluation/fixtures/` (see [docs/scenarios/realism/README.md](../../scenarios/realism/README.md))
- Explicit `fragmentation_stagger_phase_ticks` in profile id / launch args

## Fixture hints

- Compare phase variants only with matched seed and explicit profile ids in lineage
- Reference `ambiguity_sweep_wave5_phase_spacing.csv` summary if available locally

## README_case.md (copy to demo bundle)

1. Timing labels describe **tested geometry and cadence** — not fielded operating limits.
2. Stronger silence windows are not monotonically “better” in Wave 5 evidence.
3. Use lifecycle strip for churn localization; counters may still be dormant.
4. Do not export phase pocket as a deployment recommendation.
5. Close: empirical timing envelope only; not readiness.
