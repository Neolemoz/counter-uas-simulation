# GNC tuning playbook — Phase 0 frozen reference + operator checklist

Companion to the Cursor plan **Systematic guidance tuning playbook (physics-aware)**. Implements **Phase 0** snapshot and forwards **Phases 1–4** to [`run_gnc_tuning_playbook.sh`](../../scripts/evaluation/run_gnc_tuning_playbook.sh).

## Phase 0 — Frozen reference (no guidance tweaks)

| Field | Value |
|-------|--------|
| **Spine `launch-args`** | `use_gazebo_gui:=false eng_rollout_feasibility_gate:=true` |
| **Cohort template** | `gnc_tune_p0_reference_<git_short>` |
| **Role** | Every later phase holds this identical except the **≤2** tuned knobs per arm. |

Committed snapshot (regenerate with `bash scripts/evaluation/run_gnc_tuning_playbook.sh reference`):

- [`scripts/evaluation/fixtures/gnc_tuning_phase0_reference.txt`](../../scripts/evaluation/fixtures/gnc_tuning_phase0_reference.txt)

## Phases 1–4 (automation)

From repo root:

```bash
# Write / refresh Phase 0 fixture
bash scripts/evaluation/run_gnc_tuning_playbook.sh reference

# Pilot bracket (default N=15 per arm; override TUNING_PILOT_N=40)
bash scripts/evaluation/run_gnc_tuning_playbook.sh phase1

# After picking Phase 1 winner, export for next phases (or rely on script defaults)
export PHASE1_APPEND='guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35'
bash scripts/evaluation/run_gnc_tuning_playbook.sh phase2

# PHASE2_APPEND / PHASE3_APPEND must repeat the full prior launch-args prefix (script does not merge a delta-only fragment).
export PHASE2_APPEND='guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35 t_go_filter_max_step_s:=0.0'
bash scripts/evaluation/run_gnc_tuning_playbook.sh phase3

PI=$(python3 -c 'import math; print(math.pi)')
export PHASE3_APPEND="guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35 t_go_filter_max_step_s:=0.0 align_speed_use_smooth_hit_range:=false guidance_u_max_step_rad:=$PI"
bash scripts/evaluation/run_gnc_tuning_playbook.sh phase4
```

Adjust values after each phase from `runs/mc/tune_p*.json` / `monte_carlo.py compare`. For `t_go_filter_alpha`, add it to the same exported fragment (not varied by the script).

**Confirm run (N=40)** for a promoted arm (example):

```bash
TUNING_CONFIRM_N=40 TUNING_CONFIRM_SEED=9511 \
  bash scripts/evaluation/run_gnc_tuning_playbook.sh confirm_phase1_arm \
    'guidance_terminal_range_m:=800 guidance_terminal_pursuit_blend:=0.35' \
    gnc_tune_p1_confirm_r800
```

## Quick smoke (CI / no long batches)

```bash
TUNING_QUICK=1 bash scripts/evaluation/run_gnc_tuning_playbook.sh all
```

Uses `TUNING_PILOT_N=2` and short timeout. **Not** statistically sufficient for decisions.

## Analysis

```bash
python3 scripts/monte_carlo.py aggregate --logs-dir runs/logs --pattern '*.log' \
  --meta-cohort 'gnc_tune_p1_r400_b035_<git_short>' --label tune_p1_r400 --out-dir runs/mc
python3 scripts/monte_carlo.py compare --inputs runs/mc/tune_p1_r400.json runs/mc/tune_p1_r650.json
```

Replace `<git_short>` with the value in `gnc_tuning_phase0_reference.txt`.
