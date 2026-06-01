# RT runtime capture → replay import (v1)

**Status:** evaluation-side mapping + viewer-compatible replay fixture (D1 Step 3). **Not** authoritative replay state.

## Purpose

Map frozen `rt_runtime_run_capture_v1` artifacts (`runtime_run.json` from RT capture) into
candidate `replay_sa_bundle_v1` payloads for SA-R0 replay review. This path is additive,
read-only at import time, and does not modify the RT bridge, Gazebo, ROS topics, or the
SA viewer.

## Tools

| Tool | Role |
|------|------|
| `scripts/rt/rt_runtime_replay_import_dry_run.py` | Read-only JSON preview (no writes) |
| `scripts/evaluation/rt_runtime_replay_bundle.py` | Bundle transformer library |
| `fixtures/rt_sandbox/runtime_run_capture_golden_v1.json` | Source capture golden |
| `fixtures/rt_sandbox/runtime_run_replay_mapping_golden_v1.json` | Expected mapping golden |
| `fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json` | Exported `replay_sa_bundle_v1` golden from runtime capture |

### Dry-run (stdout only)

```bash
python3 scripts/rt/rt_runtime_replay_import_dry_run.py \
  fixtures/rt_sandbox/runtime_run_capture_golden_v1.json --json
```

### Replay bundle export

```bash
python3 scripts/evaluation/rt_runtime_replay_bundle.py \
  fixtures/rt_sandbox/runtime_run_capture_golden_v1.json \
  --out fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json
```

### Programmatic bundle build

```python
from pathlib import Path
from rt_runtime_replay_bundle import build_replay_sa_bundle_from_runtime_run, load_runtime_run

artifact = load_runtime_run(Path("runtime_run.json"))
bundle = build_replay_sa_bundle_from_runtime_run(artifact, source_path="runtime_run.json")
```

## Field mapping

| Runtime capture (`rt_runtime_run_capture_v1`) | Replay bundle (`replay_sa_bundle_v1`) |
|-----------------------------------------------|----------------------------------------|
| `capture_id`, `session_id`, `started_utc`, `stopped_utc` | `lineage.*`, `comprehension.at_a_glance` cards |
| `entities[]` (`entity_id`, `entity_type`, `pose`) | `entities_static[]` (`kind`, `position_enu_m`, `authoritative: false`) |
| `entities[]` + `telemetry_frames[entity_pose_mirror]` | `tracks[]` (`role` threat/interceptor, `samples[]` with `t` = frame index) |
| `telemetry_frames[entity_pose_mirror]` (interceptor rows) | `panels.telemetry_series[]` |
| `lifecycle_transitions[]` | `clock.markers[]` + dry-run `lifecycle_summary` |
| `assignments` | dry-run `assignment_summary`; telemetry `target_id` / `assignment_state` |
| `schema`, `governance_banner`, `world_revision` | `source_artifacts`, `rt_runtime_import.mapping_warnings`, `lineage.world_revision` |

### Entity / track roles

| `entity_type` (RT) | `entities_static.kind` | `tracks.role` |
|--------------------|------------------------|---------------|
| `waypoint_marker` | `waypoint` | — |
| `radar` | `radar` | — |
| `interceptor` | `interceptor` | `interceptor` |
| `drone` | `threat` | `threat` |

### Clock domain

- Bundle `clock.domain`: `runtime_capture_frame_index`
- `clock.duration`: `0 .. max(frame_index)` from telemetry frames and track samples
- **Not** ROS log line indices — do not compare directly to log-derived replay bundles

## Warnings

Dry-run and bundle `rt_runtime_import.mapping_warnings` may include:

- Missing required capture fields (`session_id`, `entities`, `telemetry_frames`, …)
- Unexpected `schema` (expected `rt_runtime_run_capture_v1`)
- No `entity_pose_mirror` frames (tracks fall back to `t=0` entity poses only)
- Empty `entities` list
- Per-entity pose missing `x`/`y`/`z`
- Absent `world_revision` or capture `governance_banner`

Warnings are explanatory only; they do not block dry-run output but set `mapping_ok: false`
when required fields are missing.

## Authority boundaries

| Surface | Authority |
|---------|-----------|
| RT bridge registry / live session | Command-authoritative for spawn/move/assign |
| `runtime_run.json` capture | Explanatory RT mirror at capture time — **not** replay truth |
| `replay_sa_bundle_v1` from this mapper | Derived evaluation artifact — **not** parser contracts |
| SA-R0 viewer | Read-only visualization of bundle — **not** operational C2 |

**Frozen:**

- No automatic corpus commit (use existing `rt_sa_import` handoff separately).
- No viewer changes in this wave.
- No changes to `replay_sa_bundle.py` log-derived pack path.
- Mirrors != authority; replay logs != parser contracts.

## Viewer compatibility

The exported golden fixture is validated through the existing SA-R0 viewer load path:

- `platform/sa-r0-viewer/src/replay/loadBundle.ts` parses bundle JSON with `parseBundleJson`.
- `platform/sa-r0-viewer/src/replay/bundleSchema.ts` accepts the exported `replay_sa_bundle_v1` shape.
- Timeline consumers use `clock.domain = runtime_capture_frame_index` and `clock.markers`.
- Telemetry consumers use `panels.telemetry_series` as sparse explanatory rows.

No UI rendering changes are required for this compatibility step.

## Governance lint

Candidate bundles should pass `replay_sa_bundle.lint_replay_sa_bundle` (notice, mode,
`georef_display.caveat`, prohibited operational fields). Bundles intentionally omit
scenario packs, LOS segments, and log-derived narrative unless added in a later wave.
