# RT runtime replay compare (`rt_runtime_replay_compare_v1`)

**Status:** D2 Step 2 evaluation-side adapter. **Not** authoritative replay state.

## Purpose

Compare two D1 runtime replay bundles after they have already been exported as
`replay_sa_bundle_v1`. The adapter is read-only and does not call the runtime bridge,
Gazebo, ROS, or viewer UI code.

## Tool

```bash
python3 scripts/evaluation/rt_runtime_replay_compare.py \
  fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json \
  fixtures/rt_visualization/runtime_capture_replay_bundle_variant_golden_v1.json \
  --json
```

Golden fixtures:

| Path | Purpose |
|------|---------|
| `fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json` | D1 runtime replay bundle A |
| `fixtures/rt_visualization/runtime_capture_replay_bundle_variant_golden_v1.json` | Minimal runtime replay bundle B variant |
| `fixtures/rt_visualization/runtime_capture_replay_compare_golden_v1.json` | Expected compare adapter output |
| `platform/sa-r0-viewer/public/demo/rt_runtime_compare/base/index.json` | Viewer-loadable copy of bundle A |
| `platform/sa-r0-viewer/public/demo/rt_runtime_compare/variant/index.json` | Viewer-loadable copy of bundle B |
| `platform/sa-r0-viewer/public/demo/compare_pairs.json` | Includes `rt_runtime_capture_replay` pair |

## Compare schema

Top-level identity:

| Field | Value |
|-------|-------|
| `artifact_type` | `rt_runtime_replay_compare_v1` |
| `schema_version` | `rt_runtime_replay_compare_v1` |

Output fields:

| Field | Meaning |
|-------|---------|
| `inputs` | Bundle A/B paths supplied to the CLI |
| `duration_delta` | A/B clock span and `b - a` span delta |
| `entity_count_delta` | A/B `entities_static` count and delta |
| `track_count_delta` | A/B `tracks` count and delta |
| `lifecycle_markers` | Lifecycle markers copied from each bundle clock |
| `topology_diff_summary` | Static entity, zone, overlay, and LOS-count differences |
| `warnings` | Weak/empty signal notes and schema/runtime-origin warnings |

## Supported fields

The adapter reads only existing `replay_sa_bundle_v1` fields:

- `clock.duration`
- `clock.markers` where `category == "lifecycle"`
- `entities_static[].entity_id`
- `entities_static[].position_enu_m`
- `tracks[]`
- `zones[].zone_id`
- `overlays[].overlay_id`
- `los_segments[]`
- optional `rt_runtime_import`

## Known weak / empty deltas

D1 runtime replay bundles are sparse. Compare output should be interpreted with these limits:

- Detection and selection deltas are unavailable unless future runtime replay bundles emit those markers.
- LOS deltas are weak/empty when `los_segments` is empty.
- Narrative event deltas are weak/empty when `narrative.events` is empty.
- Topology diffs are static replay-bundle geometry comparisons only.
- This adapter does not produce sweep manifests or viewer compare-pair catalog entries.

## Governance

Runtime replay compare is explanatory only. It must not imply operational readiness,
validated effectiveness, tactical superiority, or parser authority.

## Viewer compatibility

The existing SA-R0 compare path can load the runtime replay pair without UI changes:

- `loadComparePairs.ts` reads `/demo/compare_pairs.json` and validates the existing `scenario_compare_pairs_v1` shape.
- `resolveCompareUrl.ts` resolves `?pair=rt_runtime_capture_replay`, fetches both runtime replay bundle URLs, and calls `compareStore.enterCompare`.
- `compareStore.ts` stores both `replay_sa_bundle_v1` payloads using the same slot A/B state as existing replay pairs.

Demo URL shape:

```text
?pair=rt_runtime_capture_replay
```

No compare UI component changes are required for this fixture wiring.
