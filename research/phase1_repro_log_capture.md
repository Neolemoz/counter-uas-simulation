## Phase 1 — Reproducibility + standardized log capture

Goal: make each run **repeatable** and ensure outputs are captured as **one log file + machine-parsable summary**.

### Standard run commands (baseline)

Prereq: workspace already built (so `install/setup.bash` exists).

- Single-target Gazebo scenario:

```bash
python3 scripts/run_capture.py --scenario single --timeout-s 14
```

- Multi-target Gazebo scenario:

```bash
python3 scripts/run_capture.py --scenario multi --timeout-s 14
```

Outputs:
- Log: `runs/logs/<timestamp>_<scenario>.log`
- Metadata: `runs/logs/<timestamp>_<scenario>.meta.json`

### What is stored (per run)
- **stdout/stderr** merged into the `.log`
- **metadata JSON** with:
  - `run_id`, `created_utc`, `cmd`, `timeout_s`
  - `git_commit`, `git_dirty` (so we know exactly what code produced the log)

### How to compare runs (grep/parse + summarize)

#### Quick grep

```bash
rg "\\[HIT\\]|\\[min_miss\\]|\\[LAYER\\]|hit_threshold\\s*=" runs/logs/<your-run>.log
```

#### Summarize one run into JSON

```bash
python3 scripts/summarize_run.py runs/logs/<your-run>.log --format json
```

#### Summarize one run into CSV row

```bash
python3 scripts/summarize_run.py runs/logs/<your-run>.log --format csv
```

Parsed fields (Phase 0 schema):
- `hit` from `[HIT]`
- `min_miss_m` from `[min_miss]` (and/or `min_miss=` in `[HIT]`)
- `hit_threshold_m` from `hit_threshold = ... m`
- `layer_last` from `[LAYER]`
- `layer_at_hit` from `layer=` in `[HIT]` (else `unknown`)

