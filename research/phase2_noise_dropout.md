## Phase 2 — Perception realism: noise/dropout (no perfect ground truth)

Goal: stop feeding perfect ground truth directly into interception by inserting a **measurement layer**.

### Deliverables in this repo
- **ROS2 node**: `gazebo_target_sim/noisy_measurement_node`
  - Subscribes: `/drone/position` (ground truth, `geometry_msgs/Point`)
  - Publishes: `/drone/position_noisy` (noisy measurement, `geometry_msgs/Point`)
  - Params:
    - `noise_std_m` (float): Gaussian std-dev (meters) applied to x/y/z
    - `dropout_prob` (float): probability of dropping a publish tick (0..1)
    - `seed` (int): deterministic noise/dropout
    - `rate_hz` (float): output publish rate
- **Launch wiring** (enable/disable):
  - `src/gazebo_target_sim/launch/gazebo_target.launch.py`
  - `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py`
  - Both accept `use_noisy_measurement:=true` and route interception input through the noisy topic.

### How to run (single-target)

Baseline (ground truth):

```bash
python3 scripts/run_capture.py --scenario single --timeout-s 14
```

Noise enabled:

```bash
python3 scripts/run_capture.py --scenario single --timeout-s 14 --launch-args "use_noisy_measurement:=true noise_std_m:=0.5 dropout_prob:=0.0 noise_seed:=1 noise_rate_hz:=10.0"
```

### How to run (multi-target)

```bash
python3 scripts/run_capture.py --scenario multi --timeout-s 14 --launch-args "use_noisy_measurement:=true noise_std_m:=0.5 dropout_prob:=0.0 noise_seed:=1 noise_rate_hz:=10.0"
```

### Summarize and compare

```bash
python3 scripts/summarize_run.py runs/logs/<run>.log --format json
```

### Small results table (fill by running 10–30 trials each)

- Scenario: `single`
- Timeout per run: 14s

| setting | noise_std_m | dropout_prob | hit_rate | median(min_miss_m) |
|---|---:|---:|---:|---:|
| baseline | 0.0 | 0.0 | TBD | TBD |
| noise A | 0.3 | 0.0 | TBD | TBD |
| noise B | 0.8 | 0.0 | TBD | TBD |

