# RT-R2f — RT→SA Bridge Planning (PLAN-RT-R2f)

**Phase:** PLAN-RT-R2f — P1 SA bridge planning closure (R1-SA-05)  
**Prerequisite:** PLAT-RT-R2e frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Close R1-SA-05: define governance-safe RT→SA handoff for **manual maintainer import only** — docs and contracts only; no SA ingestion implementation.

## Allowed

| Item | Notes |
|------|-------|
| Contracts | [rt_rt_sa_bridge_handoff_v1.md](../evaluation/rt_rt_sa_bridge_handoff_v1.md), [rt_manual_sa_import_workflow_v1.md](../evaluation/rt_manual_sa_import_workflow_v1.md), [rt_sa_lineage_protection_v1.md](../evaluation/rt_sa_lineage_protection_v1.md) |
| Audit vocabulary | `handoff_*` export events — **specified only** in [rt_audit_event_vocabulary_v1.md](../evaluation/rt_audit_event_vocabulary_v1.md) |
| Authority lexicon | Additive §3 terms in [rt_authority_model_v1.md](../evaluation/rt_authority_model_v1.md) |
| Cross-links | `rt_sa_export_boundary_v1`, `rt_runtime_export_semantics_v1`, `rt_capture_continuity_v1` |
| Governance / freeze audits | R2f review + freeze audit |
| Registry / roadmap / AGENTS | Navigation updates |

## Forbidden

- SA import automation; `replay_sa_bundle` from bridge
- Federation / corpus writes from RT sessions
- SA viewer runtime hooks or live telemetry
- New bridge commands; `handoff_*` emission in `audit_vocabulary.py` or CLIs
- Parser/topic/schema changes
- Telemetry UI, Cesium, distributed/autonomous runtime

## Validation

- [rt_r2f_governance_review_r1.md](../evaluation/rt_r2f_governance_review_r1.md) — governance + RT↔SA boundary + lineage
- Zero diff under `platform/rt-sandbox-bridge/`, `scripts/rt/`, `platform/sa-r0-viewer/`

No pytest required for this wave.

## Stop line

Do not start SA import **implementation** or expansion waves until a separate PLAT-* plan, governance review, and freeze audit authorize it.

## Related

- [rt_r2f_governance_review_r1.md](../evaluation/rt_r2f_governance_review_r1.md)
- [rt_r2f_freeze_audit.md](../evaluation/rt_r2f_freeze_audit.md)
