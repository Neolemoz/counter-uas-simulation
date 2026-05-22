# PHASE I1 — Orchestration Operations & Experiment Governance (PLAN-SA-I1)

**Phase:** I1 — Deterministic Offline Orchestration Operations Layer  
**Checkpoint:** PLAT-SA-H3, PLAT-SA-A1, PLAT-SA-A2 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); extends frozen H3 execution without reopening parser or queue runner semantics.

**Companion artifacts:**

- [experiment_orchestration_operations_v1.md](../evaluation/experiment_orchestration_operations_v1.md) — operations ladder, integrity commands (I1.4)
- [experiment_orchestration_ops_manifest_v1.md](../evaluation/experiment_orchestration_ops_manifest_v1.md) — additive ops sidecar (I1.2)
- [experiment_orchestration_continuity_v1.md](../evaluation/experiment_orchestration_continuity_v1.md) — 12-pack continuity (I1.6)
- [sa_i1_orchestration_operations_governance_review_r1.md](../evaluation/sa_i1_orchestration_operations_governance_review_r1.md)
- [sa_i1_orchestration_operations_freeze_audit.md](../evaluation/sa_i1_orchestration_operations_freeze_audit.md)

---

## 1. Purpose

Operationalize deterministic offline orchestration after H3 foundations: corpus-wide manifest coverage, operations lifecycle, integrity audits, promotion ergonomics, read-only viewer cognition, and scenario→replay continuity — **before** async orchestration, browser-triggered execution, or distributed workers.

**Core rule (unchanged):** *Viewer observes; CLI runs and promotes.*

---

## 2. Deliverables

| ID | Scope |
|----|--------|
| I1.1 | 12-pack job manifests + queues + audits; authoring handoff alignment |
| I1.2 | `experiment_orchestration_ops_manifest_v1` + `replay_sa_orchestration_ops.py` + `promote_experiment_manifest.py` |
| I1.3 | `replay_sa_orchestration_integrity.py` + `audit_orchestration_integrity.py`; platform auditor hook |
| I1.4 | Summaries, lineage reports, replay continuity, batch audit, repro-check |
| I1.5 | Read-only viewer: lifecycle, integrity, replay continuity panels; full mirror sync |
| I1.6 | Continuity doc + cross-plane integrity checks |
| I1.7 | Freeze audit, registry rows, tier0 hooks |

---

## 3. Non-goals

Browser queue launch, live orchestration, async/distributed workers, parser/topic changes, HITL/tactical UX, readiness scoring, implementing deferred H3 step types (`observability`, `narrative`, `bundle_pack`) in runner, rosbridge/WebSocket.

---

## 4. Depends on

PLAT-SA-H3, PLAT-SA-H4, PLAT-SA-A1, PLAT-SA-A2, PLAT-SA-C1a/C1b, PLAT-SA-R0, PLAT-SA-STAB, PLAT-SA-F1a–F1d.

---

## 5. Validation

See [sa_i1_orchestration_operations_freeze_audit.md](../evaluation/sa_i1_orchestration_operations_freeze_audit.md) regression block.

*End of PLAN-SA-I1.*
