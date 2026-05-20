# Replay MC Sweep Schema (`replay_mc_sweep_v1`)

Normative semantics for deterministic Monte Carlo replay sweep manifests in the SA-R0 platform. **Explanatory replay experiment families only** — not operational planning, validated effectiveness, or deployment readiness.

See also: [comparison_foundations.md](comparison_foundations.md), [replay_spatial_analytics_v1.md](replay_spatial_analytics_v1.md), [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md).

## Artifact identity

| Field | Value |
|-------|-------|
| `artifact_type` | `replay_mc_sweep_v1` |
| `schema_version` | `replay_mc_sweep_v1` |
| Canonical path | `fixtures/sa_r0/sweeps/<sweep_id>/sweep.json` |
| Catalog index | `fixtures/scenarios/sweeps_index_v1.json` |

## Sweep kinds

| `sweep_kind` | Use |
|--------------|-----|
| `matched_seed` | Same topology family, explicit seed lineage across members |
| `topology_sweep` | Vary scenario packs / static layout within an archetype |
| `sensor_placement_sweep` | Shared log, different static sensor geometry (D1 experiment packs) |
| `ingress_variation` | Staggered ingress / detection timing variants on shared archetype |

## Required fields

| Field | Notes |
|-------|-------|
| `sweep_id` | Stable id; URL `?sweep=` anchor |
| `sweep_kind` | One of the kinds above |
| `title` | Human-readable, non-doctrinal |
| `baseline_topology_key` | Baseline `pack_id` for sensitivity summaries |
| `governance` | `notice` required; `anti_claims` recommended |
| `lineage` | `generator`, `seed_base`, `member_count` |
| `members` | Ordered list (see below) |
| `spatial_aggregate` | Precomputed grids per [replay_spatial_analytics_v1.md](replay_spatial_analytics_v1.md) |
| `replay_aggregation` | Histograms + `dominant_patterns` (descriptive strings only) |

## D3 extensions (PLAT-SA-D3, optional)

| Field | Notes |
|-------|-------|
| `replay_narrative_summary` | Sweep-level headline + bullets — see [replay_narrative_intelligence_v1.md](replay_narrative_intelligence_v1.md) |
| `replay_cohorts` | Rule-based member groupings for workstation / filmstrip |

## Member entry

| Field | Required | Notes |
|-------|----------|-------|
| `member_id` | yes | Stable within sweep |
| `pack_id` | yes | Catalog `pack_id` |
| `demo_bundle_url` | yes | Viewer load path |
| `seed` | no | Matched-seed lineage |
| `comparison_hints` | no | Must include `sweep_id`, `member_index` when packed |
| `replay_pattern_tags` | no | From [replay_pattern_taxonomy_v1.md](replay_pattern_taxonomy_v1.md) |
| `replay_pattern_summary` | no | Short replay-local pattern description |

## Topology linkage (optional block)

```json
{
  "shared_log_ref": "valley_ingress",
  "topology_keys": ["valley_ingress", "valley_ingress_radar_shifted_north"]
}
```

## Bundle extensions

Optional on `replay_sa_bundle_v1` `comparison_hints`:

- `sweep_id`, `member_index`, `sweep_variant_id`

Optional top-level `spatial_analytics` on bundle when member-local grids are shipped without sweep manifest.

## Governance

**Do:**

- Label sweeps as replay experiment families for reviewer exploration.
- Keep seed and topology lineage visible in manifest and bundles.
- Use fictional topology caveats when geometry-heavy.

**Don't:**

- Imply validated probability, P(kill), or deployment confidence.
- Treat sweep aggregates as parser authority.
- Use green/red winner styling across members.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `gen_d2_sweep_fixtures.py`, `gen_d3_sweep_enrichment.py`, `replay_mc_sweep.py`, `aggregate_spatial_analytics.py`, `replay_narrative_intelligence.py` |
| Validator | `replay_mc_sweep.validate_sweep`, `sync_sa_catalog.py` |
| Consumer | `platform/sa-r0-viewer` sweep picker, workstation, filmstrip, spatial layers |
