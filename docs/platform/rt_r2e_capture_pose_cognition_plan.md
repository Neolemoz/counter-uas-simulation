# RT-R2e — Capture Pose Cognition & Runtime Export Semantics (PLAT-RT-R2e)

**Phase:** PLAT-RT-R2e — P1 capture cognition closure (R1-CAP-02)  
**Prerequisite:** PLAT-RT-R2d frozen; PLAT-RT-G5 frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Close R1-CAP-02: reviewer-facing contracts and additive audit/manifest surfacing for tri-source capture pose interpretation and RT export lineage — without SA bridge, UI, or capture authority changes.

## Allowed

| Item | Notes |
|------|-------|
| Contracts | [rt_capture_pose_cognition_v1.md](../evaluation/rt_capture_pose_cognition_v1.md), [rt_runtime_export_semantics_v1.md](../evaluation/rt_runtime_export_semantics_v1.md) |
| Module | `capture_pose_cognition.py` — assess, manifest block, audit helpers |
| Normalized manifest | Additive `capture_pose_cognition` block |
| Audit events | `capture_pose_authority`, `capture_pose_stale`, `capture_pose_mismatch`, `export_pose_normalized` |
| `audit_vocabulary.py` | `event_kind: capture` for session capture-pose events |
| Authority model / cross-links | Lexicon §3 additions |
| Additive tests | Stub + adapter drift paths |

## Forbidden

- Telemetry UI, Cesium, SA viewer / auto-import (R2f is separate)
- New bridge commands; parser/topic changes
- Changing normalization pass/fail or `command_pose` selection
- Gazebo feature expansion; distributed/autonomous runtime
- `session_manager` decomposition (R3a)

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

Do not start R2f (SA bridge planning) or expansion waves until PLAT-RT-R2e is frozen.

## Related

- [rt_r2e_governance_review_r1.md](../evaluation/rt_r2e_governance_review_r1.md)
- [rt_r2e_freeze_audit.md](../evaluation/rt_r2e_freeze_audit.md)
