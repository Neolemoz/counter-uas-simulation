# Replay Federation Publication Collection (`replay_federation_publication_collection_v1`)

Groups publication packets and research bundles across corpus groups for long-horizon replay research review.

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_federation_publication_collection_v1` |
| `schema_version` | yes | `replay_federation_publication_collection_v1` |
| `federation_id` | yes | Parent federation |
| `collection_id` | yes | Stable collection slug |
| `generation_revision` | yes | Builder revision |
| `governance` | yes | Notice + anti_claims |
| `members` | yes | Grouped publication refs |

## `members[]` entry

| Field | Required | Notes |
|-------|----------|-------|
| `corpus_group_id` | yes | Source partition |
| `artifact_path` | yes | Repo-relative publication or bundle path |
| `artifact_kind` | yes | `publication_packet`, `research_bundle`, `corpus_release` |
| `sha256` | yes | Primary artifact hash |
| `publication_revision` | no | F1d fingerprint when applicable |

## Governance

**Do:** Treat as offline research organization only.

**Don't:** Imply external distribution, deployment, or engagement certification.

## Producer

`build_replay_federation_index.py` — aggregates F1d `replay_corpus_publication_packet_v1` and research bundle manifests per group.
