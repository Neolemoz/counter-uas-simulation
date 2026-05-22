# Replay Linkage Index (`replay_linkage_index_v1`)

Rule-based, deterministic linkage between replay sweep families. **Not** a knowledge graph database, ML clustering, or causal inference layer.

See also: [replay_cross_sweep_synthesis_v1.md](replay_cross_sweep_synthesis_v1.md), [replay_pattern_taxonomy_v1.md](replay_pattern_taxonomy_v1.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/synthesis/replay_linkage_index_v1.json` | Canonical linkage index |
| `fixtures/sa_r0/synthesis/storyline_linkage_overlay_v1.json` | Optional curated storyline edges |

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_linkage_index_v1` |
| `schema_version` | yes | `replay_linkage_index_v1` |
| `governance` | yes | Notice + anti-claims |
| `nodes` | yes | One node per sweep |
| `edges` | yes | Rule-derived edges |

## Node fields

| Field | Required | Notes |
|-------|----------|-------|
| `sweep_id` | yes | Stable sweep identifier |
| `baseline_topology_key` | yes | Baseline topology |
| `topology_keys` | yes | From `topology_linkage` |
| `dominant_patterns` | yes | Top pattern tags in sweep |
| `experiment_tags` | no | From sweeps index |

## Edge fields

| Field | Required | Notes |
|-------|----------|-------|
| `edge_id` | yes | Stable slug `{source}__{target}__{link_kind}` |
| `source` | yes | Source sweep_id |
| `target` | yes | Target sweep_id |
| `link_kind` | yes | See below |
| `evidence` | yes | Replay-derived evidence object |
| `copy` | yes | Explanatory-only human-readable sentence |

### `link_kind` values

| Kind | Rule |
|------|------|
| `shared_pattern` | ≥1 shared dominant pattern tag across sweeps |
| `shared_topology` | Non-empty `topology_keys` intersection or shared baseline |
| `metric_similarity` | Median first-detection or ambiguity within ratio 0.85–1.15 |
| `storyline_reference` | Curated overlay from storyboard decks |

## Governance

**Do:** Document edge rules in `evidence`; keep copy replay-local.

**Don't:** Imply causal certainty, operational recommendations, or validated doctrine.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_linkage.py` |
| Consumer | Viewer linkage panel, publication appendix |
