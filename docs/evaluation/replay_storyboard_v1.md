# Replay Storyboard (`replay_storyboard_v1`)

Normative semantics for cross-demo presentation decks (mentor sequences) in the SA-R0 platform. **Explanatory replay interpretation only** — not tactical advice or operational recommendations.

See also: [replay_presentation_v1.md](replay_presentation_v1.md), [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md).

## Artifact type

Standalone JSON artifact with `artifact_type: replay_storyboard_v1`.

Stored under `fixtures/sa_r0/presentations/<storyboard_id>.json`, synced to `platform/sa-r0-viewer/public/demo/presentations/`.

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | Must be `replay_storyboard_v1` |
| `schema_version` | yes | Must be `replay_storyboard_v1` |
| `storyboard_id` | yes | Stable deck identifier |
| `title` | yes | Human-readable deck title |
| `estimated_minutes` | yes | Approximate mentor walkthrough duration |
| `governance` | yes | `notice` + optional `anti_claims` |
| `scenes` | yes | Ordered scene list |

## Scene fields

| Field | Required | Notes |
|-------|----------|-------|
| `scene_id` | yes | Stable slug within storyboard |
| `label` | yes | Scene label for nav |
| `target_url` | yes | Viewer-relative URL (e.g. `?demo=valley_ingress&presentation=walkthrough_valley_ingress_long&chapter=0`) |
| `copy` | yes | Explanatory-only mentor guidance |
| `chapter` | no | Default chapter index when scene loads |
| `importance_tags` | no | Emphasis tags: `ambiguity`, `los`, `topology`, `assignment`, `pacing` |

## Index catalog

`fixtures/sa_r0/presentations/index.json` lists available storyboards for catalog sync:

```json
{
  "artifact_type": "replay_storyboard_index_v1",
  "storyboards": [
    { "storyboard_id": "...", "title": "...", "storyboard_url": "..." }
  ]
}
```

## Governance

**Do:**

- Map 15–20 minute mentor sequences from replay demo workflow into deterministic URLs.
- Keep scene copy non-prescriptive and replay-derived.

**Don't:**

- Imply validated operational doctrine or deployment readiness.
- Reference live ROS or realtime collaboration.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `gen_e1_presentation_fixtures.py`, `export_presentation_pack.py` |
| Consumer | `platform/sa-r0-viewer` `resolvePresentationUrl.ts`, presentation mode |
