# Replay Presentation (`replay_presentation_v1`)

Normative semantics for deterministic presentation chapters and sweep walkthroughs in the SA-R0 platform. **Explanatory replay interpretation only** — not tactical advice, operational recommendations, or causal proof of runtime behavior.

See also: [replay_narrative_intelligence_v1.md](replay_narrative_intelligence_v1.md), [replay_storyboard_v1.md](replay_storyboard_v1.md), [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md).

## Artifact placement

| Block | Location |
|-------|----------|
| `presentation` | `replay_sa_bundle_v1` root (optional) |
| `presentation_walkthrough` | `replay_mc_sweep_v1` root (optional) |

## Bundle `presentation`

| Field | Required | Notes |
|-------|----------|-------|
| `walkthrough_id` | yes | Stable slug within bundle |
| `chapters` | yes | Ordered chapter list |

### Chapter fields

| Field | Required | Notes |
|-------|----------|-------|
| `chapter_id` | yes | Stable slug within walkthrough |
| `title` | yes | Short chapter label (e.g. "Ingress", "LOS degradation") |
| `t_start` | yes | Log-line index (inclusive) |
| `t_end` | yes | Log-line index (inclusive) |
| `summary` | yes | Explanatory-only sentence for reviewer guidance |
| `focus_event_ids` | no | Narrative event IDs to emphasize |
| `spotlight_annotation_ids` | no | Annotation IDs for spotlight mode |
| `topology_highlight` | no | `ridge`, `valley`, `corridor`, `assignment`, or `none` |
| `visible_layers` | no | Partial layer visibility override |
| `los_scope` | no | `all` or `selected_track` |
| `spatial_declutter` | no | `top_k`, `threshold`, or `off` |
| `narrative_emphasis` | no | `ambiguity`, `los`, `topology`, `assignment`, or `pacing` |

## Sweep `presentation_walkthrough`

| Field | Required | Notes |
|-------|----------|-------|
| `walkthrough_id` | yes | Stable slug within sweep |
| `headline` | yes | Sweep-level walkthrough headline |
| `steps` | yes | Ordered guided steps |

### Step fields

| Field | Required | Notes |
|-------|----------|-------|
| `step_id` | yes | Stable slug within walkthrough |
| `label` | yes | Step label for chapter nav |
| `kind` | yes | `chapter`, `cohort_filmstrip`, `compare_pair`, or `analytics_panel` |
| `copy` | yes | Explanatory-only guidance text |
| `member_index` | no | Target member when `kind` is chapter-related |
| `cohort_id` | no | Target cohort for filmstrip steps |
| `filmstrip_indices` | no | Member indices for filmstrip view |
| `chapter_index` | no | Index into member bundle `presentation.chapters` |

## Governance

**Do:**

- Keep chapter copy replay-local and non-prescriptive.
- Lint presentation exports for forbidden operational phrasing.

**Don't:**

- Imply live monitoring, deployment readiness, or tactical superiority.
- Use battle-map styling semantics in schema labels.
- Treat presentation chapters as parser authority.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_presentation.py`, `gen_e1_presentation_fixtures.py` |
| Consumer | `platform/sa-r0-viewer` presentation mode, export scripts |
