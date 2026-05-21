# Replay Corpus Publication (`replay_corpus_publication_v1`)

Deterministic publication packet and offline release archive for replay corpus research operations.

See also: [replay_corpus_release_v1.md](replay_corpus_release_v1.md), [replay_research_bundle_v1.md](replay_research_bundle_v1.md).

## Artifacts

| Artifact | Path |
|----------|------|
| `replay_corpus_publication_packet_v1` | `fixtures/sa_r0/synthesis/replay_corpus_publication_packet_v1.json` |
| Release archive zip | `fixtures/sa_r0/corpus_releases/<release_id>/archive/sa_r0_corpus_release_archive.zip` |

## Publication packet fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_corpus_publication_packet_v1` |
| `corpus_id` | yes | Parent corpus |
| `release_id` | yes | Primary release snapshot id |
| `generation_revision` | yes | `f1d_v1` |
| `governance` | yes | Notice + anti-claims |
| `included_artifacts` | yes | `{ path, sha256, kind }[]` publication-ready paths |
| `publication_revision` | yes | SHA256 of sorted artifact list fingerprint |

## Release manifest extensions (optional)

| Field | Notes |
|-------|-------|
| `evolution_chain` | Release id chain with index revisions |
| `publication_revision` | Fingerprint at release build time |

## Workflow

```bash
python3 scripts/evaluation/build_replay_corpus_evolution.py
python3 scripts/evaluation/build_replay_corpus_publication.py
python3 scripts/evaluation/export_replay_corpus_release.py
python3 scripts/evaluation/verify_replay_corpus_release.py
```

## Governance

**Do:** Treat as offline research packaging only.

**Don't:** Imply deployment distribution, operational monitoring, or authority certification.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_publication.py`, `export_replay_corpus_release.py` |
| Verifier | `verify_replay_corpus_release.py` |
