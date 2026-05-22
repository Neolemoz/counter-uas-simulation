# PHASE A2 — Authoring Operations & Promotion Flow (PLAN-SA-A2)

**Phase:** A2 — Deterministic Authoring Operations Layer  
**Checkpoint:** PLAT-SA-A1 frozen (authoring workstation foundations)  
**Authority:** [AGENTS.md](../../AGENTS.md); extends frozen PLAN-SA-A1 / PLAT-SA-A1 without reopening parser or H3 execution semantics.

**Companion artifacts:**

- [scenario_authoring_operations_v1.md](../evaluation/scenario_authoring_operations_v1.md) — operations ladder, integrity commands, lifecycle groups (A2.3)
- [scenario_authoring_manifest_v1.md](../evaluation/scenario_authoring_manifest_v1.md) — additive enum (`deprecated`, `archived`)
- [authoring_workflow_continuity_v1.md](../evaluation/authoring_workflow_continuity_v1.md) — 11-pack continuity
- [sa_a2_authoring_operations_governance_review_r1.md](../evaluation/sa_a2_authoring_operations_governance_review_r1.md)
- [sa_a2_authoring_operations_freeze_audit.md](../evaluation/sa_a2_authoring_operations_freeze_audit.md)

---

## 1. Purpose

Operationalize the deterministic authoring lifecycle after A1 foundations: corpus-wide manifest consistency, integrity audits, promotion ergonomics, read-only viewer cognition, and orchestration handoff readiness — **before** async orchestration, browser editing, or multi-corpus federation.

**Core rule (unchanged):** *Viewer observes; CLI promotes.*

---

## 2. Deliverables

| ID | Scope |
|----|--------|
| A2.1 | Backfill `authoring_manifest.json` + validation mirrors for all 11 catalog packs |
| A2.2 | `replay_sa_authoring_integrity.py` + `audit_scenario_authoring_integrity.py`; platform auditor hook |
| A2.3 | Additive lifecycle: `deprecated`, `archived`; coarse lifecycle groups in ops doc |
| A2.4 | Promotion summaries, lineage reports, manifest diff, reproducibility check |
| A2.5 | Read-only viewer: integrity panel, ladder, multi-hop lineage, staleness hints |
| A2.6 | Handoff refs, validation-only orch manifests (B2), catalog promotion advisory |
| A2.7 | Freeze audit, registry rows, tier0 hooks |

---

## 3. Non-goals

Browser topology editing, live orchestration from UI, async workers, parser/topic changes, HITL/tactical UX, readiness scoring, corpus auto-release, rosbridge/WebSocket.

---

## 4. Depends on

PLAN-SA-A1, PLAT-SA-A1, PLAT-SA-C1a/C1b, PLAT-SA-H3, PLAT-SA-H4, PLAT-SA-R0, PLAT-SA-STAB.

---

## 5. Validation

See [sa_a2_authoring_operations_freeze_audit.md](../evaluation/sa_a2_authoring_operations_freeze_audit.md) regression block.

*End of PLAN-SA-A2.*
