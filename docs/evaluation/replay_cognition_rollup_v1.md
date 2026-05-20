# Replay Cognition Rollup (`replay_cognition_rollup_v1`)

Deterministic, higher-level replay cognition summaries for large corpus review. **Explanatory only** — reduces reviewer fragmentation without scoring or recommendations.

See also: [replay_cross_sweep_synthesis_v1.md](replay_cross_sweep_synthesis_v1.md), [replay_narrative_intelligence_v1.md](replay_narrative_intelligence_v1.md).

## Artifact placement

Embedded in `cross_sweep_synthesis_v1.json` as `cognition_rollup`, plus standalone `cognition_rollup_summary.md`.

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_cognition_rollup_v1` |
| `schema_version` | yes | `replay_cognition_rollup_v1` |
| `bullets` | yes | Ordered cognition bullets |
| `groupings` | no | Topology-aware group labels |

## Bullet fields

| Field | Required | Notes |
|-------|----------|-------|
| `bullet_id` | yes | Stable slug |
| `observation` | yes | Descriptive replay-local statement |
| `caveat` | yes | Mandatory replay-local caveat |
| `related_sweep_ids` | no | Supporting sweep IDs |
| `topology_group` | no | e.g. `ridge_transition`, `delayed_detection_cohort` |

## Template pattern

Each bullet renders as: `{observation} — replay-local; {caveat}`

## Governance

**Do:** Lint bullets for forbidden operational phrasing.

**Don't:** Use prescriptive language ("should deploy", "recommend intercept").

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_cognition_rollup.py` |
| Consumer | Viewer synthesis panel, publication packets |
