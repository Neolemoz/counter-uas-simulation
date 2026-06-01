# RT runtime maintainer workflow v1

**Status:** doc-only maintainer workflow. **Not** runtime authority, replay authority, or
SA corpus import automation.

## Purpose

This runbook connects the existing RT sandbox capture path to the D1 replay import and D2
compare tools for an end-to-end maintainer review. It preserves the frozen boundary:
`capture_session` and runtime capture artifacts do not automatically become SA replay
authority or corpus lineage.

## Prerequisites

- Run commands from the repository root unless a step says otherwise.
- Use a local loopback bridge only.
- Keep the RT browser and SA viewer separate: RT UI is runtime-only; SA-R0 viewer is
  read-only replay review.

## 1. Start RT bridge

Default local bridge:

```bash
python3 scripts/rt/run_rt_bridge.py
```

When the local live Gazebo path requires a sourced ROS workspace:

```bash
source install/setup.bash
python3 scripts/rt/run_rt_bridge.py
```

## 2. Open RT web UI

```bash
cd platform/rt-sandbox-ui
npm install
npm run dev
```

Open:

```text
http://127.0.0.1:5174
```

Use the browser workflow to connect to the loopback bridge and configure the runtime
scenario. The UI may apply scenarios and edit runtime entities through bridge commands;
it must not call `capture_session` or perform SA import.

## 3. Apply scenario

From the RT web UI, use the configured scenario controls and Apply action.

Equivalent maintainer CLI form:

```bash
python3 scripts/rt/rt_bridge_client.py apply_scenario \
  --session-id <session_id> \
  --payload-file <scenario_payload.json>
```

The scenario payload is RT-local runtime configuration. It is not a `fixtures/scenarios/`
SA scenario pack and does not create SA lineage.

## 4. Start sim

If starting from the CLI:

```bash
python3 scripts/rt/rt_bridge_client.py start_sim
```

Record the returned `session_id` for the remaining RT commands.

## 5. Spawn / override

Spawn an RT entity:

```bash
python3 scripts/rt/rt_bridge_client.py spawn_entity \
  --session-id <session_id> \
  --payload '{"entity_type":"drone","pose":{"x":0,"y":0,"z":20,"yaw_deg":0}}'
```

Move an existing RT entity:

```bash
python3 scripts/rt/rt_bridge_client.py move_entity \
  --session-id <session_id> \
  --payload '{"entity_id":"<entity_id>","pose":{"x":10,"y":5,"z":20,"yaw_deg":0}}'
```

The web UI can perform equivalent spawn, drag, delete, and override actions through the
same bridge command path.

## 6. Start capture

```bash
python3 scripts/rt/rt_bridge_client.py start_capture --session-id <session_id>
```

## 7. Stop capture

```bash
python3 scripts/rt/rt_bridge_client.py stop_capture --session-id <session_id>
```

The response includes an `artifact_ref` similar to:

```text
runs/rt_sandbox/captures/<capture_id>/runtime_run.json
```

## 8. Validate runtime_run.json

```bash
python3 scripts/rt/rt_capture_inspect.py runtime-validate \
  runs/rt_sandbox/captures/<capture_id>/runtime_run.json \
  --json
```

Optional discovery commands:

```bash
python3 scripts/rt/rt_capture_inspect.py runtime-list --json
python3 scripts/rt/rt_capture_inspect.py runtime-latest --json
```

## 9. D1 replay import dry-run

```bash
python3 scripts/rt/rt_runtime_replay_import_dry_run.py \
  runs/rt_sandbox/captures/<capture_id>/runtime_run.json \
  --json
```

Review `mapping_ok`, missing-field warnings, entity count, track count, lifecycle markers,
and assignment summary before exporting a bundle.

## 10. Export replay bundle

```bash
python3 scripts/evaluation/rt_runtime_replay_bundle.py \
  runs/rt_sandbox/captures/<capture_id>/runtime_run.json \
  --out fixtures/rt_visualization/<bundle_name>.json
```

The output is a derived `replay_sa_bundle_v1` evaluation artifact. It is not parser truth
and does not update the SA corpus by itself.

Golden fixture example:

```bash
python3 scripts/evaluation/rt_runtime_replay_bundle.py \
  fixtures/rt_sandbox/runtime_run_capture_golden_v1.json \
  --out fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json
```

## 11. D2 compare

Compare two exported runtime replay bundles:

```bash
python3 scripts/evaluation/rt_runtime_replay_compare.py \
  fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json \
  fixtures/rt_visualization/runtime_capture_replay_bundle_variant_golden_v1.json \
  --json
```

Optional output file:

```bash
python3 scripts/evaluation/rt_runtime_replay_compare.py \
  <bundle_a.json> \
  <bundle_b.json> \
  --out fixtures/rt_visualization/<compare_name>.json
```

## 12. Open SA viewer

```bash
cd platform/sa-r0-viewer
npm install
npm run dev
```

Open the wired runtime compare fixture:

```text
http://localhost:5173?pair=rt_runtime_capture_replay
```

Custom generated bundles must be served by the SA viewer dev server before they can be
loaded with viewer URLs. Do not add live RT bridge hooks to the SA viewer.

## Review checklist

- RT bridge was local-only and loopback-bound.
- RT web UI performed runtime-only scenario/editing actions.
- Browser did not call `capture_session`, `rt_sa_import`, or corpus write tools.
- `runtime_run.json` exists under `runs/rt_sandbox/captures/<capture_id>/`.
- `rt_capture_inspect.py runtime-validate ... --json` passed.
- D1 dry-run reported expected entities, tracks, lifecycle markers, and warnings.
- D1 bundle export produced `replay_sa_bundle_v1`.
- D2 compare produced `rt_runtime_replay_compare_v1`.
- SA viewer loaded the runtime compare pair through existing compare path.
- Review language stayed explanatory: not operational readiness, not tactical superiority,
  not parser authority, not causal proof.
- No runtime bridge, Gazebo launch, UI logic, SA viewer code, parser contract, topic, or
  schema change was made for the workflow.

## Related

- [rt_runtime_capture_runbook.md](../platform/rt_runtime_capture_runbook.md)
- [rt_runtime_replay_import_v1.md](rt_runtime_replay_import_v1.md)
- [rt_runtime_replay_compare_v1.md](rt_runtime_replay_compare_v1.md)
- [rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md)
- [sa_r0_reviewer_quickstart.md](sa_r0_reviewer_quickstart.md)
