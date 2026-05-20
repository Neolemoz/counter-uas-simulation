# Scenario Topology Schema (`scenario_topology_v1`)

Portable, deterministic, replay-safe topology packs for SA replay visualization. **Evaluation-side only** — not a parser contract, not runtime authority.

See also: [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md) (packed viewer input), [sa_b1_geometry_freeze_audit.md](sa_b1_geometry_freeze_audit.md) (LOS/masking semantics).

## Authority

| Layer | Role |
|-------|------|
| `scenario_topology_v1` pack | Source of truth for static sites, zones, overlays, annotations, fictional terrain flags |
| `replay_sa_bundle_v1` | Packed derived artifact for the SA-R0 viewer (tracks, clock, LOS segments computed at pack) |
| Legacy `scenario_overlay.json` | Deprecated monolithic input; still supported via `--scenario-overlay` |

## Pack layout

Each scenario lives under `fixtures/scenarios/<pack_id>/`:

| File | Required | Contents |
|------|----------|----------|
| `metadata.json` | yes | Identity, tags, governance, provenance |
| `topology.json` | yes | `georef_anchor`, `entities_static`, `zones` |
| `overlays.json` | yes | `overlays[]` |
| `annotations.json` | yes | `annotations[]` |
| `terrain.json` | no | `terrain_model` and/or `include_fictional_terrain` |
| `README.md` | recommended | Regen notes, fictional disclaimers |

Catalog (optional): `fixtures/scenarios/index.json` lists packs for future scenario selection UX.

## `metadata.json`

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| `artifact_type` | string | yes | Must be `scenario_topology_v1` |
| `schema_version` | string | yes | Must be `scenario_topology_v1` |
| `scenario_id` | string | yes | Stable id (e.g. `ridge_defense_demo`) |
| `title` | string | yes | Human title |
| `topology_tags` | string[] | yes | Geometry/topology identity tags (e.g. `ridge_defense`) |
| `replay_tags` | string[] | yes | Replay experimentation facets (e.g. `assignment_ambiguity`) |
| `ingress_archetype` | string | yes | `open`, `valley`, `corridor`, `ridge` |
| `overlay_descriptors` | object[] | recommended | Summary of overlay kinds; `{kind, count?, label?}` |
| `ambiguity_profile` | object | recommended | `{level, focus_tags?}` — `level`: `low\|moderate\|high\|saturation` |
| `replay_duration_class` | string | recommended | `short` (<120 log lines), `medium`, `long` |
| `terrain_profile` | string | yes | `none`, `flat`, `ridged`, `valley`, `urban_clutter` |
| `narrative_focus` | string[] | recommended | Storytelling emphasis tags |
| `governance` | object | yes | `notice`, `constraints[]`, `anti_claims[]` |
| `provenance` | object | yes | `fixture_source`, `fictional_disclaimer` |

Example:

```json
{
  "artifact_type": "scenario_topology_v1",
  "schema_version": "scenario_topology_v1",
  "scenario_id": "ridge_defense_demo",
  "title": "Ridge defense replay (demo fixture)",
  "topology_tags": ["ridge_defense", "protected_area", "terrain_aware"],
  "replay_tags": ["terrain_masking", "protected_area"],
  "ingress_archetype": "ridge",
  "terrain_profile": "ridged",
  "replay_duration_class": "medium",
  "narrative_focus": ["ridge_masking", "protected_area"],
  "overlay_descriptors": [{"kind": "ridge_mask", "count": 1}],
  "ambiguity_profile": {"level": "low"},
  "governance": {
    "notice": "Fictional scenario topology for replay visualization only.",
    "constraints": ["replay-safe", "explanatory-only", "non-authoritative"],
    "anti_claims": ["not deployed geography", "not operational effectiveness"]
  },
  "provenance": {
    "fixture_source": "fixtures/scenarios/ridge_defense",
    "fictional_disclaimer": "All coordinates and terrain are fictional replay fixtures."
  }
}
```

## `topology.json`

| Field | Type | Required |
|-------|------|----------|
| `georef_anchor` | `{lat_deg, lon_deg, h_m}` | yes |
| `entities_static` | array | yes |
| `zones` | array | no (packer defaults to dome zones if empty) |

### Entity (`entities_static[]`)

| Field | Required |
|-------|----------|
| `entity_id` | yes |
| `kind` | yes — see enum below |
| `position_enu_m` | yes — `[x, y, z]` ENU meters |
| `label` | yes |
| `authoritative` | recommended `false` |

**Entity `kind` enum:** `radar`, `eoir`, `passive_rf`, `interceptor_base`, `fusion_center`

### Zone (`zones[]`)

| Field | Required |
|-------|----------|
| `zone_id` | yes |
| `kind` | yes — `protected`, `threat_risk` |
| `geometry.type` | yes — `circle` (viewer renders circles only) |
| `geometry.center_enu_m` | yes for circle |
| `geometry.radius_m` | yes for circle |
| `caveat` | recommended |

## `overlays.json`

| Field | Required |
|-------|----------|
| `overlays` | yes — array (may be empty) |

### Overlay

| Field | Required |
|-------|----------|
| `overlay_id` | yes |
| `kind` | yes — see enum below |
| `geometry.type` | yes — `polygon` |
| `geometry.vertices_enu_m` | yes — ≥3 vertices `[x,y,z]` |
| `geometry.ridge_outline_enu_m` | optional — ridge crest polyline for `ridge_mask` |
| `caveat` | recommended |
| `linked_event_ids` | optional |
| `active_t_range` | optional — `[start_t, end_t]` log-line window |

**Overlay `kind` enum:** `ridge_mask`, `los_blocked`, `degraded_visibility`, `ingress_corridor`

`ingress_corridor` uses the same polygon geometry as `degraded_visibility`; it denotes a constrained ingress path in replay storytelling.

## `annotations.json`

| Field | Required |
|-------|----------|
| `annotations` | yes — array |

Each annotation: `annotation_id`, `kind`, `title`, `body`, optional `linked_event_ids`, optional `narrative_rank` (1–3, replay emphasis only — not operational severity).

Packed into bundle `narrative.annotations`.

## `terrain.json` (optional)

Either:

- `include_fictional_terrain`: `true` — packer synthesizes default fictional heightmap, or
- `terrain_model`: full `fictional_heightmap` object (see bundle schema)

## Derived at pack time (must NOT appear in pack files)

| Field | Producer |
|-------|----------|
| `los_segments` | `replay_sa_geometry.build_los_segments()` |
| `tracks`, `clock` | Log + narrative |
| `narrative.events` | Narrative fixture |

Validation errors if `los_segments` appears in any pack JSON file.

## Mapping: pack → `replay_sa_bundle_v1`

| Pack source | Bundle destination |
|-------------|-------------------|
| `metadata.scenario_id` | `scenario.scenario_id` |
| `metadata.title` | `scenario.title` |
| `metadata.topology_tags` | `scenario.topology_tags` |
| `metadata.replay_tags` | `scenario.replay_tags` |
| `metadata.ingress_archetype` | `scenario.ingress_archetype` |
| `metadata.overlay_descriptors` | `scenario.overlay_descriptors` |
| `metadata.ambiguity_profile` | `scenario.ambiguity_profile` |
| `metadata.replay_duration_class` | `scenario.replay_duration_class` (overridden at pack from clock if absent) |
| `metadata.terrain_profile` | `scenario.terrain_profile` |
| `metadata.narrative_focus` | `scenario.narrative_focus` |
| `metadata.provenance` | `scenario.provenance` |
| pack directory name / path | `scenario.scenario_pack_id`, `source_artifacts.scenario_pack` |
| `topology.georef_anchor` | `georef_display.anchor` |
| `topology.entities_static` | `entities_static` |
| `topology.zones` | `zones` (or default domes) |
| `overlays.overlays` | `overlays` |
| `annotations.annotations` | `narrative.annotations` |
| `terrain.*` | `scenario.terrain_model` / synthetic terrain |

## Governance caveats

- Topology packs are **explanatory** visualization inputs, not sensor coverage or doctrine.
- Overlays localize replay evidence; they do not prove causality or operational effectiveness.
- `georef_anchor` is a **fictional display anchor**, not deployed geography.
- Fictional terrain heightmaps are exaggerated replay aids, not DTED or mission terrain.
- Mirrors and replay UI are not authoritative state (see [AGENTS.md](../../AGENTS.md)).

## Prohibited fields

Same as bundle schema: `command`, `engage`, `websocket_url`, `ros_topic_live`, `readiness_score`, `authority_state`, `live_mode`, `los_segments` (in pack files).

## Multi-track log grammar (B2, additive)

Synthetic demo logs may tag samples for `replay_sa_bundle.py` track parsing (not a parser contract):

| Pattern | Effect |
|---------|--------|
| `[P_HEATMAP] threat_id=threat_uav_1 pos=(x, y, z)` | Threat sample bucket `threat_uav_1` |
| `[P_HEATMAP] pos=(x, y, z)` | Default bucket `threat_uav_0` |
| `interceptor_id=interceptor_1 interceptor_pos=(...) target_pos=(...) threat_id=threat_uav_0` | Interceptor/threat guidance buckets |
| `[METRICS] id=interceptor_1 \| ...` | Sets default interceptor for following guidance lines |

Regenerate B2 demos: `python3 scripts/evaluation/gen_b2_scenarios.py`

## Validation

```bash
python3 scripts/evaluation/validate_scenario.py fixtures/scenarios/ridge_defense
python3 scripts/evaluation/validate_scenario.py --strict fixtures/scenarios/multi_ridge
```

## Packing

```bash
python3 scripts/evaluation/replay_sa_bundle.py pack \
  --narrative-json src/counter_uas/test/fixtures/replay_narrative_minimal.json \
  --observability-json ... --viz-manifest-json ... \
  --scenario-pack fixtures/scenarios/ridge_defense \
  --out-dir fixtures/sa_r0/demo_ridge_defense
```

Legacy monolithic overlay:

```bash
--scenario-overlay fixtures/sa_r0/demo_ridge_defense/scenario_overlay.json
```

`--scenario-overlay` and `--scenario-pack` are mutually exclusive.

## Tag vocabulary (C1b appendix)

Controlled tokens validated by `lint_scenario_pack()` (unknown tokens warn; `--strict` errors). Authors may propose new tokens additively via schema doc updates.

**Topology tags:** `ridge_defense`, `valley_ingress`, `multi_ridge`, `corridor_defense`, `saturation_ingress`, `urban_masking`, `delayed_detection`, `long_range_ingress`, plus modifiers such as `terrain_masking`, `multi_threat`, `assignment_ambiguity`.

**Replay tags:** `assignment_ambiguity`, `compressed_intercept_window`, `multi_threat`, `chained_masking`, `urban_clutter`, `late_acquisition`, etc.

**Narrative focus:** `ridge_masking`, `assignment_ambiguity`, `launch_sequence`, `prioritization_pressure`, etc.

## Catalog

`fixtures/scenarios/index.json` (`scenario_topology_catalog_v1`) lists packs for viewer scenario selection. Synced to `platform/sa-r0-viewer/public/demo/catalog.json` via `gen_b2_scenarios.py`.

## Deprecation

`scenario_overlay.json` under `fixtures/sa_r0/` is deprecated in favor of `fixtures/scenarios/<id>/`. Remove monolithic overlays after migration to avoid dual source of truth.
