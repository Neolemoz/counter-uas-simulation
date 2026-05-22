# Replay Federation Manifest (`replay_federation_manifest_v1`)

Deterministic registry of corpus groups within a replay federation. **Not** a cloud registry, sync service, or operational deployment catalog.

See also: [replay_federation_lineage_v1.md](replay_federation_lineage_v1.md), [replay_corpus_index_v1.md](replay_corpus_index_v1.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/federation/replay_federation_manifest_v1.json` | Canonical manifest |
| `platform/sa-r0-viewer/public/demo/federation/replay_federation_manifest_v1.json` | Viewer mirror |

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_federation_manifest_v1` |
| `schema_version` | yes | `replay_federation_manifest_v1` |
| `federation_id` | yes | Stable federation id (e.g. `sa_replay_federation_r0_v1`) |
| `generation_revision` | yes | Bumped when builder logic changes |
| `governance` | yes | Notice + anti_claims |
| `corpus_groups` | yes | Member corpus partitions |
| `parent_federation_ref` | no | Prior federation generation for long-horizon continuity |
| `publication_collection_refs` | no | Paths to publication collection manifests |
| `federation_lineage_refs` | no | Stable refs into federation lineage graph |

## `corpus_groups[]` entry

| Field | Required | Notes |
|-------|----------|-------|
| `corpus_group_id` | yes | Partition key within federation |
| `corpus_id` | yes | Parent `replay_corpus_index_v1.corpus_id` |
| `index_artifact_path` | yes | Repo-relative path to corpus index JSON |
| `release_id` | no | When group is release-scoped |
| `study_label` | no | Reviewer-facing cross-study label |
| `orchestration_scope` | no | Optional orchestration fixture subtree hint |

## Governance

**Do:** Register only offline fixture indexes; document parent federation for reproducibility chains.

**Don't:** Imply live sync, multi-user collaboration, or cross-site authority.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_federation_index.py` |
| Validator | `validate_replay_federation.py` |
| Viewer | `loadFederationArtifacts.ts`, `FederationBrowserPanel.tsx` |
