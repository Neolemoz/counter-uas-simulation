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

## Replay Export

Preview the runtime capture to replay mapping without writing files:

```bash
python3 scripts/rt/rt_runtime_replay_import_dry_run.py \
  runs/rt_sandbox/captures/<capture_id>/runtime_run.json \
  --json
```

Export a derived `replay_sa_bundle_v1` artifact:

```bash
python3 scripts/evaluation/rt_runtime_replay_bundle.py \
  runs/rt_sandbox/captures/<capture_id>/runtime_run.json \
  --out fixtures/rt_visualization/<bundle_name>.json
```

The exported bundle is an explanatory evaluation artifact only. It is not parser truth,
not SA corpus import, and not operational evidence.

## Runtime Replay Compare

Compare two exported runtime replay bundles:

```bash
python3 scripts/evaluation/rt_runtime_replay_compare.py \
  fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json \
  fixtures/rt_visualization/runtime_capture_replay_bundle_variant_golden_v1.json \
  --json
```

The wired SA-R0 compare demo can be reviewed through the existing viewer path:

```text
http://localhost:5173?pair=rt_runtime_capture_replay
```

For the complete maintainer sequence, see
[rt_runtime_maintainer_workflow_v1.md](../evaluation/rt_runtime_maintainer_workflow_v1.md).
