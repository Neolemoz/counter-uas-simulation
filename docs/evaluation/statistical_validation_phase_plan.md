# Statistical validation phase plan (frozen)

Additive matched-seed Monte Carlo harness for guidance and realism arms. Reuses `scripts/monte_carlo.py`; **no parser or topic changes**.

**Status:** Phase closed (Steps 1–5). Descriptive statistics only.

## Tooling

| Artifact | Role |
|----------|------|
| `scripts/evaluation/fixtures/statistical_validation_profiles.csv` | Seven-arm pilot matrix (N=2 default in CSV) |
| `scripts/evaluation/fixtures/statistical_validation_focused_n40_profiles.csv` | Four-arm focused pairs (N=40) |
| `scripts/evaluation/run_statistical_validation_sweep.py` | Profile loop → `monte_carlo run` |
| `scripts/evaluation/summarize_statistical_validation.py` | Pilot report builder |
| `scripts/evaluation/pair_mc_seed_outcomes.py` | Matched-seed G0–G4 buckets |
| `runs/evaluation/statistical_validation_pilot_report.json` | Consolidated N=15 + N=40 + triage |

## N=15 pilot findings (all arms, seed_base=6201)

| Arm | SR | Miss P95 | Notes |
|-----|-----|----------|--------|
| predictive_baseline | 86.7% | 0.010 m | Gate off |
| predictive_intercept | 60.0% | 39.419 m | Gate on — **worse than baseline at N=15** |
| hysteresis_off | 66.7% | 5.360 m | |
| hysteresis_on | 80.0% | 23.270 m | SR up; P95 tail worse |
| multi_defender | 6.7% | 1.637 m | See multi audit — HIT vs min_miss |
| aero_realism_on | 100.0% | 0.093 m | Engagement caps only (single) |
| sensor_realism_on | 0.0% | nan | **No tracks** — geometry/overlay (pre-fix profile) |

Paired N=15 buckets: predictive G1=6 / G3=4; hysteresis mixed (G2=5, G3=5).

**Do not** treat N=15 predictive pair as stable — superseded in direction by N=40 (see below).

## N=40 focused findings (pairs only, seed_base=6201)

| Arm | SR | Miss P95 |
|-----|-----|----------|
| predictive_baseline | 90.0% | 2.076 m |
| predictive_intercept | 95.0% | 0.251 m |
| hysteresis_off | 80.0% | 0.306 m |
| hysteresis_on | 85.0% | 18.213 m |

Paired N=40: predictive G0=23, G1=2, G2=4, G3=11; hysteresis G0=19, G1=4, G2=6, G3=9, G4=2.

**Interpretation (descriptive only):** At N=40, intercept arm shows higher SR and lower P95 than baseline; hysteresis-on improves SR vs off but tail P95 is worse. **Not** superiority, operational effectiveness, or hardware readiness.

## Sensor arm limitation

**Pilot (pre-reachability profile):** bringup with overlay but default km ingress → **Active tracks: 0** for full timeout → no intercept metrics.

**Profile fix (additive):** `target_start_x_m:=-1500.0`, `target_start_y_m:=0.0`, `target_start_z_m:=300.0` in `statistical_validation_profiles.csv`.

**Step 5 smoke (N=5, `seed_base=6301`, reachability launch args):** bringup with overlay + `target_start_*` at (−1500, 0, 300). **Outcome: not validated** — all five runs logged **Active tracks: 0** for full 120 s timeout; MC SR 0%, no miss samples. Target motion and occasional radar “distance=0.00 m” lines do **not** imply fused/track pipeline activity. Artifacts: `runs/evaluation/sensor_realism_smoke.log`, `runs/mc/sensor_realism_smoke_n5_s6301.{json,csv}`, per-run `runs/logs/2026-06-02T12-*_bringup.log`.

**Frozen:** `sensor_realism_on` remains **excluded** from statistical claims. Profile CSV reachability args are **documented but unproven** for track formation; a future governed campaign must show non-zero `Active tracks` before any sensor arm metrics.

## Multi-defender limitation

See [statistical_validation_multi_defender_audit.md](statistical_validation_multi_defender_audit.md).

**Summary:** Low SR reflects **`[HIT]` registration** vs ~1.6 m min_miss under 4.5 m threshold — **not** proven assignment failure. Arm frozen as **non-claiming** for statistical validation.

## Explicit non-claims

- No confidence intervals, p-values, or superiority statements.
- No operational readiness, hardware readiness, or tactical authority.
- No sensor performance validation from overlay MC until reachability + track confirmation campaigns exist.
- No multi-defender autonomy regression claims from pilot SR.
- N=15 and N=40 predictive comparisons **disagree in direction** — do not merge cohorts without explicit governance.

## Matched-seed rule

Same `seed_base` and `n` within a cohort; distinct `--cohort` per label. Reruns are not bitwise-identical.

## Governance

Registry: **EVAL-STAT-VAL-R1** in [freeze_registry_r1.md](freeze_registry_r1.md).  
Audit: [statistical_validation_freeze_audit.md](statistical_validation_freeze_audit.md).
