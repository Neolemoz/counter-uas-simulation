# Replay Federation Index (`replay_federation_index_v1`)

Deterministic federated inventory rollup across registered corpus groups.

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_federation_index_v1` |
| `schema_version` | yes | `replay_federation_index_v1` |
| `federation_id` | yes | Matches manifest |
| `generation_revision` | yes | `f2a_v1` |
| `governance` | yes | Notice + anti_claims |
| `manifest_ref` | yes | Path to federation manifest |
| `corpus_group_summaries` | yes | Per-group rollups |
| `federation_revision` | yes | SHA256 fingerprint of manifest + group index revisions |
| `continuity_index_ref` | no | Path to continuity index |
| `lineage_graph_ref` | no | Path to lineage graph |

## Related artifacts

| Artifact | Role |
|----------|------|
| `replay_federation_lineage_graph_v1` | Cross-group edges |
| `replay_federation_continuity_index_v1` | Canonical ↔ release continuity |
| `replay_federation_replay_summary_v1` | Reviewer cognition rollup |
| `replay_federation_reproducibility_v1` | Long-horizon fingerprint chain |
| `replay_federation_snapshot_v1` | Frozen federation generation snapshot |

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_federation_index.py` |
| Validator | `validate_replay_federation.py` |
| Viewer | `FederationBrowserPanel.tsx`, `useFederationStore.ts` |
