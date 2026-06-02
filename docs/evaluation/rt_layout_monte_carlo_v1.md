# RT Layout Monte Carlo Translator V1

## Scope

`rt_layout_scenario_v1` is an offline RT layout artifact for Monte Carlo launch-profile preview. It is not an SA scenario pack, not a bridge protocol payload, and not replay authority.

Allowed storage for this artifact is RT-local or run-local, for example:

- `fixtures/rt_sandbox/`
- `runs/rt_sandbox/layouts/`

Do not write generated RT layouts into `fixtures/scenarios/`.

## Artifact Shape

Required fields:

- `schema_version`: `rt_layout_scenario_v1`
- `layout_id`: stable layout identifier
- `terrain_preset`: RT terrain preset such as `rt_sandbox_flat`
- `entities`: ordered RT entity list
- `source`: provenance object

Optional fields:

- `created_utc`
- `notes`

Entity types remain RT-local:

- `radar`
- `interceptor`
- `drone`
- `waypoint_marker`

Each entity has:

```json
{
  "entity_type": "drone",
  "pose": { "x": -1500.0, "y": 0.0, "z": 300.0, "yaw_deg": 0.0 }
}
```

## Translator

The dry-run translator is:

```bash
python3 scripts/evaluation/rt_layout_mc_profile.py \
  fixtures/rt_sandbox/rt_layout_scenario_golden_v1.json \
  --out-json runs/rt_sandbox/layouts/rt_layout_mc_profile.preview.json \
  --out-csv runs/rt_sandbox/layouts/rt_layout_mc_profile.preview.csv
```

It emits:

- `geometry_id`: stable geometry fingerprint for matched-seed pairing
- `scenario_suggestion`: current MC scenario label suggestion
- `launch_args`: MC-compatible launch argument string
- `warnings`: supported-boundary caveats
- `unsupported_fields`: layout content with no launch mapping today

The translator does not run Monte Carlo.

## Current Mapping

Current executable simulation entry points are launch-argument driven.

Supported mappings:

- exactly one `drone` maps to `target_start_x_m`, `target_start_y_m`, `target_start_z_m`
- exactly three `interceptor` entities map to `interceptor_ic_layout:=custom:x0,y0,x1,y1,x2,y2`

Metadata-only today:

- `radar`
- `waypoint_marker`
- `yaw_deg`
- `terrain_preset`

Unsupported today:

- multiple drones as direct target launch args
- one or two interceptor layout overrides
- RT radar or waypoint behavior changes

## Boundaries

This path is additive-only and offline:

- no RT Web UI implementation
- no RT bridge command or protocol change
- no Gazebo launch behavior change
- no autonomy, tactical, tracker, or fusion logic change
- no replay viewer change
- no automatic Monte Carlo run
- no automatic SA import

Future promotion into SA replay artifacts must remain maintainer-gated and use existing replay/capture/import boundaries.
