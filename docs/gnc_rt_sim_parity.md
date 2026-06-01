# RT sandbox ↔ offline simulation engagement parity

**Step 4 (aero realism):** single limit tuple for kinematic plant and offline MC harness.

## Canonical source

`src/rt_sandbox_gz/config/rt_engagement_limits.yaml`

| Field | Default | Notes |
|-------|---------|--------|
| `max_speed_mps` | 25.0 | Interceptor cap (RT tactical geometry uses same order) |
| `max_accel_mps2` | 30.0 | Matches km-scale `gazebo_target` launch contracts |
| `max_turn_rate_deg_s` | 16.0 | Conservative; aligns offline `realtime_sim` |
| `max_climb_mps` | 8.0 | Vertical rate cap |
| `drag_decel_per_mps` | 0.12 | Light linear drag (coast decay) |
| `wind_*_mps` | 0.0 | Off unless scenario sets wind |

## Consumers

- **RT Gazebo bridge:** `rt_sandbox.launch.py` / `config/rt_kinematic.yaml`
- **Mock adapter:** `platform/rt-sandbox-bridge` attach defaults
- **Offline sim:** `simulation/realtime_sim.py` (`MAX_TURN_RATE_DEG_S`)
- **Shared integrator:** `simulation/rt_plant_adapter.py` → `rt_sandbox_gz.kinematic_plant`

## Parity tests

`src/counter_uas/test/test_rt_offline_engagement_parity.py`

Offline guidance loops still use `simulation/core/dynamics.py` for legacy MC; RT pose integration uses the shared plant. Compare plants only via `integrate_toward_pose` / `rt_plant_adapter`.
