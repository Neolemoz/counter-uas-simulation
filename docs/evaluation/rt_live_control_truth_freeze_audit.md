# RT Live Control Truth Freeze Audit (PLAT-RT-LIVE-CONTROL-TRUTH1)

**Status:** frozen  
**Contract:** [rt_live_control_truth_v1.md](rt_live_control_truth_v1.md)  
**Prerequisite:** PLAT-RT-G6, PLAT-RT-LIVE-GZ1 frozen

## Architecture summary

Live-mode **control truth** aligns Gazebo Sim physics pause/resume/reset with adapter session state and bridge-node kinematic integration.

| Layer | Role |
|-------|------|
| `adapter_worker` | IPC pause/resume/reset; Gazebo WorldControl via `gz_tools.py`; session `clock` publish |
| Gazebo WorldControl | `/world/{world}/control` — pause, resume, `reset: {all: true}` |
| `LiveRosClient` | Subscribes bridged `/clock`; publishes session `clock` + `entity_pose_cmd` |
| `rt_sandbox_gz_bridge_node` | Gates timer integration on session `clock.paused`; handles `reset_world` op |
| `clock_mirror` telemetry | Explanatory pause + optional additive sim-time fields |

## Implemented scope

| Step | Deliverable |
|------|-------------|
| 3 | `gz_world_pause` / `resume` / `reset_all` helpers; live `_pause` / `_resume` → WorldControl |
| 4 | Session `clock` pause signal; bridge-node integration gate; mock plant pause parity |
| 5 | Live `_reset_world` → entity deletes, bridge `reset_world` op, `gz_world_reset_all`, map clears |
| 6 | `/clock` → additive `sim_time_sec` / `sim_time_nsec` / `sim_time_source`; post-reset pause re-apply |

### Files

- `src/rt_sandbox_gz/rt_sandbox_gz/gz_tools.py`
- `src/rt_sandbox_gz/rt_sandbox_gz/gz_bridge_node.py`
- `platform/rt-sandbox-bridge/rt_sandbox/adapter_worker.py`
- `platform/rt-sandbox-bridge/rt_sandbox/live_ros_client.py`
- `docs/evaluation/rt_live_control_truth_v1.md`
- `docs/evaluation/rt_adapter_live_sync_v1.md` (additive cross-ref)
- `src/counter_uas/test/test_rt_gz_tools.py`
- `src/counter_uas/test/test_rt_gz_bridge_pause.py`
- `src/counter_uas/test/test_rt_gz_reset.py`
- `src/counter_uas/test/test_rt_gz_clock_truth.py`

## Explicit exclusions

- No telemetry schema version change (`rt_adapter_telemetry_v1` unchanged)
- No parser contract or ROS evaluation topic changes
- No autonomy, tactical assignment, or engagement authority
- No SA viewer coupling or live replay merge
- No bridge-node redesign beyond pause/reset truth
- No live-default adapter flag change

## Governance verification

| Check | Result |
|-------|--------|
| Telemetry schema version unchanged | Pass |
| Parser contracts unchanged | Pass |
| No autonomy additions | Pass |
| No engagement authority | Pass |
| No assignment authority | Pass |
| No SA viewer coupling | Pass |
| Additive `entity_pose_cmd` op only | Pass |
| Additive `clock_mirror` fields only | Pass |

## Validation results

```bash
python3 -m pytest src/counter_uas/test/test_rt_gz_tools.py -q
python3 -m pytest src/counter_uas/test/test_rt_gz_bridge_pause.py -q
python3 -m pytest src/counter_uas/test/test_rt_gz_reset.py -q
python3 -m pytest src/counter_uas/test/test_rt_gz_clock_truth.py -q
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
git diff --check
```

All TRUTH1 unit tests and existing RT sandbox bridge tests pass at freeze time.

## Remaining live-smoke recommendation

Maintainer validation with colcon-sourced stack (not CI-gated):

```bash
# Requires ROS 2 + Gazebo + rt_sandbox_gz on PATH
python3 scripts/rt/rt_live_smoke.py
```

Verify manually: pause freezes Gazebo + bridge integration; resume continues without dt jump; reset clears models and sim time; reset-while-paused keeps world paused; `/clock` sim-time appears in `clock_mirror` when bridged.

## Freeze commit

`a8252c0`

## Frozen verdict

**PLAT-RT-LIVE-CONTROL-TRUTH1** is frozen as additive live Gazebo control truth on the existing RT adapter / bridge-node path. Further work (live smoke hardening, sim-clock consumers) requires a new scoped wave.
