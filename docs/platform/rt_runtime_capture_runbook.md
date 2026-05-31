# RT Runtime Capture Runbook

Phase 6 runtime capture is a transient RT-side artifact flow. Captures are explanatory runtime records under `runs/rt_sandbox/captures/`; they are not SA replay authority and do not imply analytics derivation.

## Start Bridge

From the repository root:

```bash
python3 scripts/rt/run_rt_bridge.py
```

For a live Gazebo session, source the ROS workspace first when needed by the local environment:

```bash
source install/setup.bash
python3 scripts/rt/run_rt_bridge.py
```

## Run Sim

Open a session through the maintainer CLI:

```bash
python3 scripts/rt/rt_bridge_client.py start_sim
```

Use the returned `session_id` for subsequent commands. Spawn or apply runtime-only entities through existing commands such as `spawn_attacker`, `spawn_defender`, or `apply_scenario`.

## Capture

Start capture after the session is running:

```bash
python3 scripts/rt/rt_bridge_client.py start_capture --session-id <session_id>
```

Stop capture to persist the artifact:

```bash
python3 scripts/rt/rt_bridge_client.py stop_capture --session-id <session_id>
```

The response includes `artifact_ref`, normally:

```text
runs/rt_sandbox/captures/<capture_id>/runtime_run.json
```

## Inspect Artifact

Validate a specific runtime run:

```bash
python3 scripts/rt/rt_capture_inspect.py runtime-validate runs/rt_sandbox/captures/<capture_id>/runtime_run.json --json
```

List captured runtime runs:

```bash
python3 scripts/rt/rt_capture_inspect.py runtime-list --json
```

Show the latest captured runtime run:

```bash
python3 scripts/rt/rt_capture_inspect.py runtime-latest --json
```

Minimum validated fields are `session_id`, `capture_id`, timestamps, `entities`, `telemetry_frames`, `assignments`, and `lifecycle_transitions`.
