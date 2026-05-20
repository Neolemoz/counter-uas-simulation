# Replay Compare Schema (`replay_compare_v1`)

Normative semantics for governance-safe comparative replay in the SA-R0 viewer. **Explanatory comparability only** — not rankings, readiness, operational effectiveness, or deployment planning.

See also: [comparison_foundations.md](comparison_foundations.md), [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md), [scenario_schema_v1.md](scenario_schema_v1.md).

## Comparison modes

| Mode | `compare_mode` | When to use |
|------|----------------|-------------|
| Replay A/B | `replay_ab` | Two packed demo bundles (may differ in log, narrative, and topology) |
| Topology A/B | `topology_ab` | Different scenario packs; may share log when `shared_log_ref` is set |
| Sensor placement study | `sensor_study` | Experiment pack vs baseline; same log, different static layout |

## Identifiers

| Field | Role |
|-------|------|
| `pair_id` | Stable id in `scenario_compare_pairs_v1` manifest |
| `topology_key` / `catalog_entry_id` | Pack directory name |
| `sensor_layout_id` | `sha256:` hash of sorted static entity positions |
| `baseline_topology_key` | Baseline pack for sensor studies |
| `paired_topology_key` | Variant pack id |

## Pairing manifest (`scenario_compare_pairs_v1`)

Path: `fixtures/scenarios/compare_pairs_v1.json` (synced to `platform/sa-r0-viewer/public/demo/compare_pairs.json`).

Each pair entry:

| Field | Required | Notes |
|-------|----------|-------|
| `pair_id` | yes | URL `?pair=` anchor |
| `label` | yes | Human-readable, non-doctrinal |
| `mode` | yes | `replay_ab`, `topology_ab`, or `sensor_study` |
| `slot_a` | yes | `{ pack_id, demo_bundle_url }` |
| `slot_b` | yes | Same shape |
| `shared_log_ref` | no | Pack id when both slots use same `demo.log` |
| `governance_notice` | yes | Explanatory disclaimer |

## Deterministic topology diff (viewer)

Diff rules operate on packed bundle fields only:

1. **Static sites** — compare `entities_static` by `entity_id`; report position deltas in ENU.
2. **Zones** — compare `zone_id`, kind, circle center/radius.
3. **Overlays** — compare `overlay_id`, kind, `active_t_range`.
4. **LOS segments** — aggregate `status` counts per `(from_entity_id, to_track_id)`; explanatory only.

Output: ordered bullet strings; no winner/superiority language.

## Replay outcome observations (viewer)

Replay-side milestones extracted from bundle clock markers and narrative events:

- First detection `t`
- Ambiguity / reacquisition windows
- Selection / intercept-window timing (mock)
- LOS degradation span counts
- Clock duration span

Display as **replay observations** with Δt in log-line indices when comparing two bundles.

## Extended `comparison_hints` (bundle)

Optional on `replay_sa_bundle_v1` (additive to C1b):

```json
{
  "topology_key": "valley_ingress_radar_shifted_north",
  "sensor_layout_id": "sha256:abc…",
  "compare_mode": "sensor_study",
  "baseline_topology_key": "valley_ingress",
  "paired_topology_key": "valley_ingress_radar_shifted_north"
}
```

## Governance

**Do:**

- Label all compare UI as explanatory replay diff.
- Keep matched-seed pairing explicit in lineage when applicable.
- Document fictional topology when comparing geometry.

**Don't:**

- Imply tactical superiority, validated effectiveness, or deployment readiness.
- Treat viewer diff bullets as parser authority.
- Use green/red “winner” styling for outcomes.

## D2 extensions (PLAT-SA-D2)

- `comparison_hints.sweep_id`, `member_index`, `sweep_variant_id` on bundles in sweeps
- `replay_mc_sweep_v1` + `scenario_sweeps_index_v1` — see [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md)
- Export `replay_compare_report_v1` via `export_replay_analytics_report.py`
- Optional static `matched_seed_comparison_report` alongside sweep fixtures

## D3 implemented (PLAT-SA-D3)

- Sweep-scoped N-slot filmstrip (`?sweep=&filmstrip=` or `&cohort=`) — not global N-way compare
- `replay_narrative_summary`, `replay_cohorts`, `replay_pattern_tags` on sweep manifests
- Review exports: `sweep_narrative_summary.md`, `replay_cluster_report.md`, `replay_review_report_v1.json`
- Pack-time overlay `active_t_range` ⊆ log span lint (warnings in bundle lint)
