# Replay Narrative Intelligence (`replay_narrative_intelligence_v1`)

Normative semantics for deterministic, sweep-level replay reasoning summaries in the SA-R0 platform. **Explanatory replay interpretation only** — not tactical advice, operational recommendations, or causal proof of runtime behavior.

See also: [replay_pattern_taxonomy_v1.md](replay_pattern_taxonomy_v1.md), [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md), [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md).

## Artifact placement

Sweep-level blocks are embedded in `replay_mc_sweep_v1` manifests (not a separate top-level artifact type).

| Block | Location |
|-------|----------|
| `replay_narrative_summary` | `sweep.json` root |
| `replay_cohorts` | `sweep.json` root |
| `replay_pattern_tags` | `members[]` entry |

## `replay_narrative_summary`

| Field | Required | Notes |
|-------|----------|-------|
| `headline` | yes | Single descriptive sentence; non-prescriptive |
| `bullets` | yes | Ordered strings; max 8 recommended |
| `importance_weights` | no | Keys: `ambiguity`, `los`, `topology`, `assignment`, `pacing` — viewer emphasis only |

**Themes** (deterministic rules in `replay_narrative_intelligence.py`):

- Replay outcome shifts across members
- Ambiguity escalation concentration
- LOS degradation concentration
- Topology-sensitive replay divergence
- Assignment instability (selection/divergence narrative events)
- Replay pacing / detection timing variance

## `replay_cohorts`

Ordered list of reviewer-facing groups (rule-based, not ML).

| Field | Required | Notes |
|-------|----------|-------|
| `cohort_id` | yes | Stable slug within sweep |
| `label` | yes | Human-readable, non-doctrinal |
| `pattern_tags` | yes | From [replay_pattern_taxonomy_v1.md](replay_pattern_taxonomy_v1.md) |
| `member_indices` | yes | Indices into `members[]` |
| `dominant_summary` | yes | One descriptive sentence |
| `anomaly_member_indices` | no | Members flagged as sweep outliers |

## Member extensions

| Field | Required | Notes |
|-------|----------|-------|
| `replay_pattern_tags` | no | Zero or more `pattern_id` values |
| `replay_pattern_summary` | no | Short replay-local description |

## Governance

**Do:**

- Prefix summaries with replay-local framing (“across replay variants”, “in this sweep family”).
- Lint markdown exports for forbidden operational phrasing (shared with D2 exporter).

**Don't:**

- Imply optimal deployment, validated effectiveness, or tactical superiority.
- Treat narrative bullets as parser authority or causal runtime proof.
- Use severity or readiness language for `importance_weights`.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `replay_narrative_intelligence.py`, `classify_replay_pattern.py`, `gen_d3_sweep_enrichment.py` |
| Consumer | `platform/sa-r0-viewer` workstation panels, review exports |
