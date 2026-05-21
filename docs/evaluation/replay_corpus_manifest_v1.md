# Replay Corpus Manifest (`replay_corpus_manifest_v1`)

Per-entry or per-release manifest describing source artifacts, scopes, and SHA256 integrity. Embedded in index entries or standalone release snapshots.

See also: [replay_corpus_index_v1.md](replay_corpus_index_v1.md), [replay_corpus_release_v1.md](replay_corpus_release_v1.md).

## Usage contexts

| Context | Location |
|---------|----------|
| Index entry | `sha256_manifest` on `replay_corpus_index_v1` entries with multiple files |
| Release snapshot | `replay_corpus_release_manifest_v1.json` under `corpus_releases/` |
| Research bundle | `included_files` on `replay_research_bundle_v1` (pre-existing pattern) |

## Manifest fields (per-entry)

| Field | Required | Notes |
|-------|----------|-------|
| `manifest_kind` | yes | `entry` or `release` |
| `corpus_id` | yes | Parent corpus |
| `entry_id` | yes | Index entry slug (release uses `release_id`) |
| `source_artifacts` | yes | List of repo-relative paths |
| `sha256_manifest` | yes | `{ path, sha256, size_bytes? }[]` |
| `replay_scope` | no | Optional replay bounds |
| `sweep_scope` | no | Optional sweep/member bounds |
| `presentation_scope` | no | Optional storyboard refs |
| `export_scope` | no | Optional export kinds included |
| `generation_tool` | yes | Producer script |
| `generation_revision` | yes | Content hash or fixed rev string |

## Scope objects (optional)

**`replay_scope`:** `{ pack_id?, member_id?, seed?, log_path? }`

**`sweep_scope`:** `{ sweep_id, member_ids[], baseline_topology_key? }`

**`presentation_scope`:** `{ storyboard_id?, chapter_ids[]? }`

**`export_scope`:** `{ export_kinds: string[] }` — e.g. `compare`, `review`, `publication`, `presentation`

## Governance

Manifests are **explanatory integrity records**, not authority grants. SHA256 mismatches indicate stale fixtures, not operational failure.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_index.py`, `build_replay_corpus_release.py` |
| Consumer | `validate_replay_corpus.py`, `export_research_bundle.py` |
