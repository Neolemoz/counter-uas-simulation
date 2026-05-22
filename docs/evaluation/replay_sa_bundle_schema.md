# Replay SA Bundle Schema (`replay_sa_bundle_v1`)

Normative schema for the SA-R0 viewer input. **Evaluation-side only** — not a parser contract.

## Top-level

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| `artifact_type` | string | yes | Must be `replay_sa_bundle` |
| `bundle_schema_version` | string | yes | Must be `replay_sa_bundle_v1` |
| `mode` | string | yes | Must be `replay_static` in SA-R0 |
| `governance` | object | yes | notice, constraints, anti_claims |
| `lineage` | object | yes | run_id, log_path, seed, etc. |
| `scenario` | object | yes | scenario_id, title, topology_tags, launch_geometry; C1b optional fields below |
| `comparison_hints` | object | no | Pre-D1 comparison anchors (see comparison_foundations.md) |
| `source_artifacts.scenario_pack` | string | no | Repo-relative path to `scenario_topology_v1` pack |
| `georef_display` | object | yes | frame, origin_enu_m, anchor (display-only) |
| `clock` | object | yes | domain, duration, markers |
| `tracks` | array | yes | May be empty; samples use `t` index |
| `entities_static` | array | yes | radar, eoir, fusion_center, interceptor_base, etc. |
| `zones` | array | yes | circles/polygons with caveats |
| `overlays` | array | yes | los_blocked, ridge_mask, degraded_visibility, etc. |
| `los_segments` | array | no | Explanatory sensor–target polylines + status |
| `narrative` | object | yes | events, bookmarks, annotations |
| `comprehension` | object | yes | scan_guide, headline, `at_a_glance` object |
| `comprehension.at_a_glance` | object | yes | `{ cards: [{label, value}], summary?: {...} }` — not an array |
| `source_artifacts` | object | yes | paths to upstream JSON |
| `views` | object | no | mock pane configs |
| `panels` | object | no | telemetry_series, threat_assessment |
| `corpus_ref` | object | no | F1a corpus index pointer (`corpus_id`, `entry_id`, `lineage_parent_ids`, `index_revision`) |

## Track sample

```json
{
  "track_id": "threat_uav_0",
  "role": "threat",
  "samples": [
    {"t": 3, "x_m": -1200.0, "y_m": 50.0, "z_m": 300.0, "source": "log_evidence"}
  ],
  "style": {"polyline": "dashed", "authoritative": false},
  "interpretation_caveat": "Log-evidenced samples only; not continuous path truth."
}
```

## LOS segment (optional)

```json
{
  "segment_id": "los_radar_01_threat_uav_0_t7",
  "from_entity_id": "radar_01",
  "from_kind": "site",
  "to_track_id": "threat_uav_0",
  "t": 7,
  "status": "partially_occluded",
  "polyline_enu_m": [[0, 0, 12], [-600, 40, 280]],
  "caveat": "Replay-local LOS association — not sensor truth.",
  "linked_event_ids": []
}
```

`status` must be one of: `visible`, `partially_occluded`, `terrain_blocked`.

## Overlay geometry

Overlays may include:

| Field | Notes |
|-------|-------|
| `kind` | `ridge_mask`, `los_blocked`, `degraded_visibility` |
| `active_t_range` | Optional `[start_t, end_t]` log-line window |
| `geometry.ridge_outline_enu_m` | Optional ridge crest polyline |

## Scenario terrain model (optional)

```json
{
  "terrain_model": {
    "type": "fictional_heightmap",
    "grid_enu_m": { "origin": [-2000, -2000], "spacing_m": 200, "size": 16, "heights_m": [[...]] },
    "caveat": "Exaggerated replay terrain — not deployed geography."
  }
}
```

## Zone geometry

```json
{
  "zone_id": "protected_core",
  "kind": "protected",
  "geometry": {
    "type": "circle",
    "center_enu_m": [0, 0, 0],
    "radius_m": 500
  },
  "caveat": "Scenario policy overlay — not validated doctrine."
}
```

## Prohibited fields

`command`, `engage`, `websocket_url`, `ros_topic_live`, `readiness_score`, `authority_state`, `live_mode`.

## Scenario topology input

Portable topology packs (`scenario_topology_v1`) are defined in [scenario_schema_v1.md](scenario_schema_v1.md). Pack with `--scenario-pack fixtures/scenarios/<id>` (preferred) or legacy `--scenario-overlay scenario_overlay.json`.

## Scenario block (C1b optional fields)

| Field | Notes |
|-------|-------|
| `replay_tags` | Replay experimentation facets |
| `ingress_archetype` | `open`, `valley`, `corridor`, `ridge` |
| `overlay_descriptors` | Summary of pack overlay kinds |
| `ambiguity_profile` | `{level, focus_tags?}` — explanatory only |
| `replay_duration_class` | `short`, `medium`, `long` |
| `terrain_profile` | Terrain/topology class |
| `narrative_focus` | Storytelling emphasis tags |
| `provenance` | Pack fictional disclaimer + fixture source |
| `catalog_pack_id` | Directory name under `fixtures/scenarios/` |

## `comparison_hints` (optional, C1b + D1)

See [comparison_foundations.md](comparison_foundations.md) and [replay_compare_v1.md](replay_compare_v1.md).

| Field | Notes |
|-------|-------|
| `topology_key`, `catalog_entry_id`, `scenario_id` | Pack anchors |
| `sensor_layout_id` | `sha256:` hash of static entity positions (D1) |
| `compare_mode` | `replay_ab`, `topology_ab`, `sensor_study` |
| `baseline_topology_key`, `paired_topology_key` | Sensor study pairing |

## Producer

`python3 scripts/evaluation/replay_sa_bundle.py pack ...`

## Consumer

`platform/sa-r0-viewer/` — validates with Zod at load time.
