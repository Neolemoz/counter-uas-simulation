# RT-R1 — Runtime Architecture Stabilization Review (PLAN-RT-R1)

**Phase:** PLAN-RT-R1 — architecture stabilization and consolidation review (docs only)  
**Prerequisite:** PLAN-RT-S1, PLAT-RT-S2–S6, PLAN-RT-G1, PLAT-RT-G2–G5 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); frozen RT contracts under `docs/evaluation/rt_*_v1.md`

## Goal

Perform a deep, read-only architecture stabilization review of the entire RT interactive sandbox line before further expansion (telemetry UI, Cesium runtime visualization, autonomous runtime, multi-runtime orchestration, distributed infra). Produce consolidation findings, a post-G5 roadmap, and a freeze verdict — **without** changing runtime behavior.

## Allowed

- Documentation: plan, master review, governance review, freeze audit, post-G5 roadmap
- Cross-reference matrix and audit-event taxonomy (appendix in master review)
- Revision vocabulary glossary (docs-only annex in master review)
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md), [sa_platform_maintainer_checklist.md](../evaluation/sa_platform_maintainer_checklist.md)
- Regression evidence: existing `test_rt_sandbox_bridge.py` pytest (no new tests in R1)

## Forbidden

- New runtime features, bridge commands, or contract `v2` schemas
- Telemetry UI, Cesium integration, SA viewer live hooks
- Automatic SA replay ingestion, federation publication, orchestration authority from RT
- Distributed runtime, multi-session orchestration, autonomous runtime behavior
- Operational dashboards, HITL/C2 semantics, production security infra
- `session_manager.py` refactor or new modules in `platform/rt-sandbox-bridge/`
- Parser/topic/schema changes; rosbridge / legacy `web/` extension
- Read-only audit automation scripts (deferred; contrast PLAT-SA-STAB)

## Review workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | Lifecycle consistency | [rt_session_lifecycle_v1.md](../evaluation/rt_session_lifecycle_v1.md), `lifecycle.py`, `session_manager.py` |
| 2 | Authority consistency | [rt_runtime_governance_v1.md](../evaluation/rt_runtime_governance_v1.md), registry, mirrors, capture |
| 3 | Sync vs telemetry semantics | [rt_runtime_synchronization_v1.md](../evaluation/rt_runtime_synchronization_v1.md), `pose_sync.py`, `telemetry_bridge.py` |
| 4 | Capture determinism | [rt_capture_normalization_v1.md](../evaluation/rt_capture_normalization_v1.md), `capture_normalize.py` |
| 5 | Runtime governance hardening | `governance.py`, `isolation.py`, `ros_allowlist.py` |
| 6 | Auditability consistency | `audit_log.py`, `export_audit_log.py`, audit vocabulary |
| 7 | Technical debt | `session_manager.py` concentration, duplicated stores |
| 8 | RT↔SA boundary re-validation | [rt_sa_export_boundary_v1.md](../evaluation/rt_sa_export_boundary_v1.md), `export_boundary.py` |

## Deliverables

| Artifact | Path |
|----------|------|
| Master review | [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md) |
| Post-G5 roadmap | [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md) |
| Governance review | [rt_r1_governance_review_r1.md](../evaluation/rt_r1_governance_review_r1.md) |
| Freeze audit | [rt_r1_freeze_audit.md](../evaluation/rt_r1_freeze_audit.md) |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

Documentation-only diff hygiene: no changes under `platform/rt-sandbox-bridge/`, `platform/sa-r0-viewer/`, `src/counter_uas/`, or parser paths.

## Stop line

Do not start telemetry UI, Cesium runtime viz, SA bridge ingestion, autonomous runtime, or distributed infra until:

1. PLAN-RT-R1 is frozen, and  
2. P0 findings in [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md) are addressed via **new scoped waves** (each with plan + governance review + freeze audit).

PLAT-RT-G5 stop line remains: no automatic SA replay ingestion, federation publication, or SA viewer runtime integration without a new wave audit.

## Related

- [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md)
- [rt_r1_governance_review_r1.md](../evaluation/rt_r1_governance_review_r1.md)
- [rt_r1_freeze_audit.md](../evaluation/rt_r1_freeze_audit.md)
- [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)
