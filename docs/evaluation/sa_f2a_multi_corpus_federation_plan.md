# SA F2A — Multi-Corpus Federation Foundations (PLAN-SA-F2A / PLAT-SA-F2A)

**Status:** implementation wave (additive to PLAT-SA-F1a–F1d, PLAT-SA-I3)

**Checkpoint:** PLAT-SA-R0–H5, PLAT-SA-A1/A2, PLAT-SA-I1/I2/I3, PLAT-SA-F1a–F1d frozen.

## Goal

Create governance-safe **offline-first multi-corpus federation** for replay research operations: federation manifests, federated indexing, cross-study lineage, publication collection organization, integrity auditing, reproducibility snapshots, and read-only viewer cognition — without cloud sync, collaborative editing, or browser orchestration authority.

## Allowed

- Federation schema docs (`replay_federation_*_v1`)
- `replay_federation_lineage.py`, federation build/validate/audit CLIs
- Federation fixtures under `fixtures/sa_r0/federation/`
- Read-only federation panels in sa-r0-viewer (corpus segment extension)
- Federation recovery continuity hooks (additive to I3)
- `gen_f2a_federation_fixtures.py`, integrity checks, `tier0-sa-r0`, tests, freeze audit

## Forbidden

- Distributed/cloud federation, live synchronization, collaborative editing
- Browser-triggered orchestration or recovery authority
- WebSocket / rosbridge / live ROS
- Parser/topic/schema redesign
- Operational readiness, HITL, ML recommendations
- Reopening frozen I2/I3 `operations_status` or promote semantics

## Deliverables

| ID | Deliverable |
|----|-------------|
| F2A.1 | Federation manifest + schema docs |
| F2A.2 | `build_replay_federation_index.py`, lineage graph, continuity index, publication collection, replay summary |
| F2A.3 | `audit_replay_federation_integrity.py` + integrity report fixture |
| F2A.4 | Read-only federation viewer panels + URL params |
| F2A.5 | Reproducibility + snapshot builders/verifiers |
| F2A.6 | `audit_federation_recovery_continuity.py` + recovery continuity report |
| F2A.7 | Freeze audit, governance review, registry, AGENTS.md |

## Validation

```bash
python3 scripts/evaluation/build_replay_federation_index.py --check
python3 scripts/evaluation/validate_replay_federation.py
python3 scripts/evaluation/audit_replay_federation_integrity.py --strict
python3 scripts/evaluation/verify_replay_federation_reproducibility.py
python3 scripts/evaluation/audit_federation_recovery_continuity.py --strict
python3 -m pytest src/counter_uas/test/test_replay_federation_index.py \
  src/counter_uas/test/test_replay_federation_integrity.py \
  src/counter_uas/test/test_replay_federation_recovery_continuity.py -q
scripts/ci_eval.sh tier0-sa-r0
```

## Related

- [replay_federation_manifest_v1.md](replay_federation_manifest_v1.md)
- [sa_f1a_corpus_indexing_plan.md](sa_f1a_corpus_indexing_plan.md)
- [sa_i3_async_recovery_plan.md](../platform/sa_i3_async_recovery_plan.md)

## Stop line after PLAT-SA-F2A freeze

Cloud federation, live sync, collaborative multi-user systems, browser orchestration, third+ corpora without registry wave.
