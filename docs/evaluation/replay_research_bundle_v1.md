# Replay Research Bundle (`replay_research_bundle_v1`)

Portable, reproducible archive of replay synthesis artifacts for mentor review and research documentation.

See also: [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md), [replay_cross_sweep_synthesis_v1.md](replay_cross_sweep_synthesis_v1.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/research_bundles/<corpus_id>/` | Directory bundle |
| `fixtures/sa_r0/research_bundles/<corpus_id>.zip` | Optional portable zip |

## Manifest fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_research_bundle_v1` |
| `schema_version` | yes | `replay_research_bundle_v1` |
| `corpus_id` | yes | Stable corpus identifier |
| `governance` | yes | Notice + anti-claims |
| `included_files` | yes | List of `{ path, sha256, size_bytes }` |
| `provenance` | yes | Generator versions and timestamps |
| `sweep_ids` | yes | Sweeps included |
| `corpus_ref` | no | F1a corpus index pointer for `research_bundle` entry |

## Bundle layout

```
<corpus_id>/
  manifest.json
  synthesis/cross_sweep_synthesis_v1.json
  synthesis/replay_linkage_index_v1.json
  synthesis/replay_corpus_index_v1.json
  synthesis/cognition_rollup_summary.md
  synthesis/linkage_summary.md
  synthesis/cross_sweep_summary.md
  figures/*
  assets/*
  sweeps/<sweep_id>/reports/*
  provenance/generator_versions.json
```

## Governance

**Do:** Include SHA256 for reproducibility checks; label as offline research archive.

**Don't:** Imply operational deployment or cloud service dependency.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `export_research_bundle.py`, `gen_e2_research_fixtures.py` |
| Consumer | Mentor handoff, internship demos, documentation workflows |
