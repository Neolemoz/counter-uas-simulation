# Statistical validation freeze audit (EVAL-STAT-VAL-R1)

**Wave:** Monte Carlo / statistical validation phase (Steps 1–5)  
**Status:** frozen stable (harness + docs; no new MC campaigns authorized)

## Allowed

- Re-run **existing** profile drivers with same governance (descriptive only).
- Sensor **reachability smoke** (N≤10, separate seed base) — descriptive reachability only; Step 5 smoke **did not** confirm non-empty tracks.
- Read `runs/evaluation/statistical_validation_pilot_report.json` for review.

## Forbidden

- Parser / topic / schema changes for MC aggregates.
- Superiority or operational effectiveness claims from pilot or N=40 tables.
- Full seven-arm MC sweeps without new scoped plan.
- Autonomy or tactical logic changes justified only by multi_defender pilot SR.

## Frozen surfaces

- `scripts/monte_carlo.py` aggregate contract (via `parse_run_to_result`).
- `scripts/evaluation/run_statistical_validation_sweep.py`
- `scripts/evaluation/summarize_statistical_validation.py`
- `scripts/evaluation/fixtures/statistical_validation_profiles.csv`
- `scripts/evaluation/fixtures/statistical_validation_focused_n40_profiles.csv`
- `docs/evaluation/statistical_validation_phase_plan.md`
- `docs/evaluation/statistical_validation_multi_defender_audit.md`

## Evidence artifacts (local, gitignored runs/)

- `runs/evaluation/statistical_validation_summary.csv` (N=15)
- `runs/evaluation/statistical_validation_focused_n40_summary.csv`
- `runs/evaluation/statistical_validation_pilot_report.json`

## Regression

- `src/counter_uas/test/test_statistical_validation_*.py`
- `src/counter_uas/test/test_summarize_statistical_validation.py`
