# RT-R3d — World Revision Hint Policy (PLAT-RT-R3d)

**Phase:** PLAT-RT-R3d — P2 maintenance / sync governance (R1-SYNC-05)  
**Prerequisite:** PLAT-RT-R3c frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Clarify and harden revision hint semantics. Closes roadmap item **RT-R3d** / finding **R1-SYNC-05** — final P2 maintenance wave.

## Allowed

| Item | Notes |
|------|-------|
| Contract | [rt_world_revision_hint_policy_v1.md](../evaluation/rt_world_revision_hint_policy_v1.md) |
| Vocabulary extension | [rt_revision_vocabulary_v1.md](../evaluation/rt_revision_vocabulary_v1.md) §4 divergence policy |
| Sync/telemetry/capture/audit doc alignment | Cross-links and wording normalization |
| Read-only helper | `revision_hint_policy.py` — `revision_counter_roles()`, `expected_world_revision_hint_keys()` |
| Clarifying comments | `pose_sync.py`, `telemetry_bridge.py` |
| Additive policy tests | Revision hint shape, sync_revision equality, roles export |

## Forbidden

- Runtime behavior changes (unless required — none expected)
- Telemetry UI, Cesium, SA integration, Gazebo expansion
- New bridge commands or subcommands
- Expansion waves without new audit

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
scripts/ci_eval.sh tier0
```

## Stop line

P2 complete after PLAT-RT-R3d freeze. Do not start expansion waves (telemetry UI, Cesium, SA bridge implementation) without explicit new wave audit.

## Related

- [rt_r3d_governance_review_r1.md](../evaluation/rt_r3d_governance_review_r1.md)
- [rt_r3d_freeze_audit.md](../evaluation/rt_r3d_freeze_audit.md)
