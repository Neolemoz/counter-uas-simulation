# Replay Corpus Evolution (`replay_corpus_evolution_v1`)

Long-horizon replay evolution tracking across corpus releases and chronology tiers. **Not** operational lifecycle or deployment history.

See also: [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md), [replay_corpus_release_v1.md](replay_corpus_release_v1.md).

## Artifacts

| Artifact | Path |
|----------|------|
| `replay_corpus_evolution_manifest_v1` | `fixtures/sa_r0/synthesis/replay_corpus_evolution_manifest_v1.json` |
| `replay_corpus_evolution_summary_v1` | `fixtures/sa_r0/synthesis/replay_corpus_evolution_summary_v1.json` |

Viewer mirrors under `platform/sa-r0-viewer/public/demo/synthesis/`.

## Manifest fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_corpus_evolution_manifest_v1` |
| `corpus_id` | yes | Parent corpus |
| `generation_revision` | yes | `f1d_v1` (bumped when evolution logic changes) |
| `governance` | yes | Notice + anti-claims |
| `releases` | yes | Discovered release snapshots + canonical target |
| `chronology_tiers` | yes | Ordered tiers from index `chronology_group` |
| `cross_release_diffs` | yes | Refs to pairwise release diffs |

## Summary fields

| Field | Required | Notes |
|-------|----------|-------|
| `topology_sensitivity_evolution` | yes | Per-sweep topology rollup from synthesis |
| `ambiguity_trend_rollup` | yes | Tier-level ambiguity concentration comparison |
| `pattern_evolution_rollup` | yes | Pattern frequency by replay family |
| `divergence_chronology` | yes | Ordered explanatory bullets |
| `long_horizon_family_narratives` | yes | Per-family narrative blocks with caveats |

## Optional index entry fields (F1d)

| Field | Notes |
|-------|-------|
| `release_generation_id` | Stable generation bucket from chronology |
| `publication_revision_lineage` | Parent `content_revision` refs |
| `evolution_tags` | e.g. `first_generation`, `synthesis_rollup` |
| `replay_chronology_descriptor` | Human chronology label |

## Governance

**Do:** Use for mentor/reviewer long-horizon replay research documentation.

**Don't:** Imply causal doctrine, tactical superiority, or readiness certification.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_evolution.py` |
| Consumer | `CorpusEvolutionPanel`, publication packet builder |
