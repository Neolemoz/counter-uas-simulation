# Replay Corpus Index (`replay_corpus_index_v1`)

Deterministic inventory of replay corpus artifacts with structural lineage and integrity hashes. **Not** a database, cloud registry, or operational deployment catalog.

See also: [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md), [replay_corpus_manifest_v1.md](replay_corpus_manifest_v1.md), [replay_corpus_provenance_rules_v1.md](replay_corpus_provenance_rules_v1.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/synthesis/replay_corpus_index_v1.json` | Canonical corpus index |
| `platform/sa-r0-viewer/public/demo/synthesis/replay_corpus_index_v1.json` | Viewer mirror |

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_corpus_index_v1` |
| `schema_version` | yes | `replay_corpus_index_v1` |
| `corpus_id` | yes | Stable corpus identifier (e.g. `sa_r0_corpus_r1`) |
| `generation_revision` | yes | Bumped when index builder logic changes (not wall-clock) |
| `governance` | yes | Notice + anti-claims |
| `entries` | yes | Corpus inventory entries |
| `lineage_edges` | no | Normalized derivation edges (optional duplicate of entry lineage) |

## Entry fields

| Field | Required | Notes |
|-------|----------|-------|
| `entry_id` | yes | Stable slug `{entry_kind}__{slug}` |
| `corpus_id` | yes | Parent corpus |
| `entry_kind` | yes | See below |
| `lineage_parent_ids` | yes | Parent entry IDs (may be empty) |
| `derived_from` | yes | Typed derivation refs |
| `source_artifacts` | yes | Upstream paths used to build this entry |
| `generation_tool` | yes | Producer script name |
| `generation_revision` | yes | SHA256 of primary artifact or content revision |
| `replay_scope` | no | Replay-local scope object |
| `sweep_scope` | no | Sweep scope when applicable |
| `presentation_scope` | no | Storyboard/presentation scope |
| `export_scope` | no | Static export scope |
| `primary_artifact_path` | yes | Repo-relative path to primary JSON/manifest |
| `sha256` | yes | Hash of primary artifact bytes |
| `sha256_manifest` | no | Multi-file entries: list of `{ path, sha256 }` |
| `content_revision` | yes | Deterministic revision fingerprint for staleness checks |
| `navigation_tags` | no | Filter chips for corpus browser (F1c) |
| `reviewer_category` | no | Stable reviewer grouping bucket |
| `replay_family` | no | Family header for declutter grouping |
| `chronology_group` | no | Timeline bucket for chronology sort |
| `navigation_hint` | no | Short reviewer-facing label |

See [replay_corpus_navigation_v1.md](replay_corpus_navigation_v1.md) for navigation semantics and URL contract.

### `entry_kind` values

| Kind | Primary artifact |
|------|------------------|
| `demo_bundle` | `fixtures/sa_r0/demo_*/index.json` |
| `sweep_family` | `fixtures/sa_r0/sweeps/*/sweep.json` |
| `topology_experiment` | `fixtures/scenarios/<pack>/metadata.json` |
| `presentation_deck` | `fixtures/sa_r0/presentations/<id>.json` |
| `publication_packet` | Per-sweep `replay_publication_report_v1.json` |
| `synthesis_report` | Corpus synthesis JSON (cross-sweep, linkage, assets index) |
| `replay_export` | Per-sweep static analytics/compare/review exports |
| `research_bundle` | `research_bundles/<corpus_id>/manifest.json` |
| `corpus_release` | `corpus_releases/<release_id>/replay_corpus_release_manifest_v1.json` |

## Relationship to other indexes

| Artifact | Role |
|----------|------|
| `scenario_sweeps_index_v1` | Catalog picker for sweeps |
| `replay_linkage_index_v1` | Semantic sweep-to-sweep similarity |
| **`replay_corpus_index_v1`** | Full corpus inventory + structural derivation DAG |

## Governance

**Do:** Use SHA256 for integrity; document derivation in `derived_from`; keep scopes replay-local.

**Don't:** Imply operational verification, deployment readiness, or scientific validity certification.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_index.py` |
| Validator | `validate_replay_corpus.py` |
| Auditor | `audit_replay_lineage.py` |
| Consumer | Corpus browser, lineage nav, provenance panel, research bundle inclusion |
