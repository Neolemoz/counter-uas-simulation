# Layer B Realism Validation Notes

Layer B keeps the simulator explicitly **kinematic**, **research-grade**, and
**validated within stated assumptions**.  It does not claim aerodynamic,
field-ready, or operational realism.

## Local Validation Completed

The following local checks passed after the shared plant/timing/measurement
changes:

```bash
python3 -m pytest -q --tb=short
```

Result:

```text
66 passed
```

The changed Python modules and launch files also compile with `py_compile`.

Heatmap-vs-Gazebo cell selection was smoke-tested without launching Gazebo:

```bash
python3 scripts/validate_heatmap_vs_gazebo.py \
  --heatmap-csv scripts/evaluation/fixtures/sample_heatmap_for_validate.csv \
  --dry-run --seed 1
```

Result: selected 9 validation cells across high/mid/low probability bands.

Existing log aggregation was smoke-tested:

```bash
python3 scripts/monte_carlo.py aggregate \
  --logs-dir runs/logs \
  --pattern '*.log' \
  --label layer_b_local_aggregate_smoke
```

Result: aggregate summary written under `runs/mc/`.

## Required Gazebo Campaign Gate

The full matched-seed Layer B campaign still requires a built ROS/Gazebo runtime
and should be run as an evaluation gate, not as a unit-test substitute:

```bash
python3 scripts/monte_carlo.py run \
  --n 40 \
  --seed-base 2001 \
  --label layer_b_tracks_state_baseline \
  --cohort layer_b_tracks_state_baseline \
  --launch-args 'use_gazebo_gui:=false intercept_measurement_source:=tracks_state eng_rollout_feasibility_gate:=true'

python3 scripts/monte_carlo.py run \
  --n 40 \
  --seed-base 2001 \
  --label layer_b_delay_tau \
  --cohort layer_b_delay_tau \
  --launch-args 'use_gazebo_gui:=false intercept_measurement_source:=tracks_state eng_rollout_feasibility_gate:=true interceptor_autopilot_tau_s:=0.25 interceptor_cmd_delay_s:=0.15'

python3 scripts/monte_carlo.py compare \
  --inputs runs/mc/layer_b_tracks_state_baseline.json runs/mc/layer_b_delay_tau.json
```

Acceptance should be based on matched-seed deltas in success rate, miss-distance
P50/P95, and failure-class histogram, with the launch argument envelope preserved
in each run's `.meta.json`.
