# Replay Corpus Navigation (`replay_corpus_navigation_v1`)

Additive navigation metadata and reviewer URL contract for `replay_corpus_index_v1` entries. **Not** operational routing or deployment catalog semantics.

See also: [replay_corpus_index_v1.md](replay_corpus_index_v1.md), [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md).

## Optional entry fields

| Field | Required | Notes |
|-------|----------|-------|
| `navigation_tags` | no | Sorted filter chips, e.g. `mc_sweep`, `synthesis`, `export` |
| `reviewer_category` | no | Stable grouping bucket (see below) |
| `replay_family` | no | Declutter / family header key |
| `chronology_group` | no | Timeline bucket for reviewer sort |
| `navigation_hint` | no | Short human-facing label |

### `reviewer_category` values

| Value | Typical `entry_kind` |
|-------|----------------------|
| `topology_lab` | `topology_experiment`, `demo_bundle` |
| `sweep_experiment` | `sweep_family` |
| `presentation` | `presentation_deck` |
| `synthesis` | `synthesis_report` |
| `export` | `publication_packet`, `replay_export` |
| `corpus_ops` | `research_bundle`, `corpus_release` |

## URL contract (sa-r0-viewer)

| Param | Meaning |
|-------|---------|
| `corpus_entry` | Stable `entry_id` — browser selection + lineage context |
| `corpus_release` | Release id (e.g. `sa_r0_corpus_r1_r1`) — release browser panel |
| `federation_id` | Active federation manifest (PLAT-SA-F2A) |
| `corpus_group_id` | Filters federation/corpus context to a registered group |
| `federation_lineage_ref` | Highlights edge in federation lineage panel |

Entry navigation resolves to existing viewer modes:

| `entry_kind` | Resolved params |
|--------------|-----------------|
| `demo_bundle` | `demo={pack_id}` |
| `topology_experiment` | `demo={pack_id}` from slug |
| `sweep_family` | `sweep={sweep_id}` |
| `presentation_deck` | `presentation={storyboard_id}` |
| `publication_packet`, `replay_export` | `sweep={sweep_id}` |
| `synthesis_report` | `sweep={first sweep_id}` |
| `research_bundle` | corpus browser info only |

## Viewer artifacts

| Path | Role |
|------|------|
| `/demo/synthesis/replay_corpus_index_v1.json` | Corpus inventory |
| `/demo/corpus_audits/replay_corpus_drift_report_v1.json` | Drift findings |
| `/demo/corpus_releases/{release_id}/replay_corpus_release_manifest_v1.json` | Release snapshot |

## Governance

**Do:** Keep navigation deterministic; treat drift badges as maintainer aids only.

**Don't:** Imply operational monitoring, readiness, or lifecycle management.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_index.py` (`_apply_navigation_metadata`) |
| Consumer | `CorpusBrowserPanel`, `resolveCorpusEntryUrl.ts`, `CorpusProvenancePanel` |
