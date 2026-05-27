# RT-R1a — Runtime Vocabulary & Authority Hardening (PLAT-RT-R1a)

**Phase:** PLAT-RT-R1a — P0 hardening closure  
**Prerequisite:** PLAN-RT-R1 frozen; PLAT-RT-S2–S6, PLAT-RT-G2–G5 frozen  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md); [rt_roadmap_post_g5_v1.md](../evaluation/rt_roadmap_post_g5_v1.md)

## Goal

Close RT-R1 P0 findings (R1-SYNC-01, R1-AUTH-02, R1-GOV-02, R1-AUDIT-02, R1-SA-03) via vocabulary contracts and **additive** code fields — no feature expansion.

## Allowed

- Contracts: [rt_revision_vocabulary_v1.md](../evaluation/rt_revision_vocabulary_v1.md), [rt_authority_model_v1.md](../evaluation/rt_authority_model_v1.md), [rt_audit_event_vocabulary_v1.md](../evaluation/rt_audit_event_vocabulary_v1.md)
- Modules: `authority_labels.py`, `audit_vocabulary.py`
- Additive audit `event_kind`; telemetry `source` + `authority_label`
- Normalized manifest `authority_model` block
- Docstrings / cross-links in existing contracts
- Additive tests

## Forbidden

- Telemetry UI, Cesium, SA integration, distributed/autonomous runtime
- New bridge commands; poll-path refactors (R2a)
- `session_manager` decomposition
- Parser/topic/schema changes
- Renaming existing `command_type` audit values

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

## Stop line

Do not start R2a (adapter poll unification) or expansion waves until PLAT-RT-R1a is frozen.

## Related

- [rt_r1a_governance_review_r1.md](../evaluation/rt_r1a_governance_review_r1.md)
- [rt_r1a_freeze_audit.md](../evaluation/rt_r1a_freeze_audit.md)
