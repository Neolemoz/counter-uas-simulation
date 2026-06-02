# Multi-defender statistical validation audit (Step 5)

**Scope:** Interpretation of pilot `multi_defender` arm only (`gazebo_target_multi`, N=15, `seed_base=6201`).  
**Not in scope:** autonomy tuning, tactical logic changes, or operational effectiveness claims.

## Observed behavior (pilot N=15)

- Scenario: `gazebo_target_multi.launch.py` with `eng_rollout_feasibility_gate:=true`.
- Aggregate: **6.7%** success rate (1/15), miss mean **~1.60 m**, intercept time mean **~1.61 s**.
- Per-run CSV shows consistent **min_miss ~1.57–1.66 m** on failures; one run registered `[HIT]` / `success=true` (`layer=engage`).
- Logs show multi-target assignment (`=== Assignment ===`, `[TACTICAL_MULTI_ASSIGN]`), threat ranking, and predict-mode metrics at **~1.66 m** distance.
- End-of-run samples often show targets at origin with zero velocity and interceptors **idle**; assignment blocks may read `unassigned`.

## Hit-registration uncertainty

- Terminal log lines include `[min_miss] = 1.6605 m` and `hit_threshold = 4.5000 m` — geometrically inside threshold.
- Kinetic success in `parse_run_to_result` requires **`[HIT]`** (or equivalent hit chain), not proximity alone.
- Many seeds therefore classify as **miss** despite small reported min_miss — **registration / timing**, not necessarily poor guidance geometry.

## Evaluation-path uncertainty

- Parser contract (`scripts/analyze_run.py`) is **unchanged** and authoritative for MC aggregates.
- Multi scenario uses **ground_truth** target topics (`/drone_*/position`), not bringup `tracks_state` — appropriate for `gazebo_target_multi`, but **not comparable** to sensor/bringup arms.
- Low SR at N=15 is **insufficient** to conclude assignment logic fault; it is sufficient to flag **evaluation mismatch** between proximity metrics and HIT-tagged success.

## Limitation statement (freeze)

The `multi_defender` profile is **frozen as non-claiming** for statistical validation until:

1. HIT registration behavior is reconciled with min_miss / hit_threshold logging, or  
2. A separate **explanatory** near-miss metric is approved under governance (not implemented in this phase).

Do **not** use pilot multi_defender SR for superiority, readiness, or autonomy regression claims.
