# PHASE I3 — Async Recovery & Batch Review Workflows (PLAN-SA-I3 / PLAT-SA-I3)

**Phase:** I3 — Async Recovery & Batch Review Cognition  
**Checkpoint:** PLAT-SA-I2, PLAT-SA-I1, PLAT-SA-H3, PLAT-SA-A1, PLAT-SA-A2, PLAT-SA-R0–H5 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); extends frozen I2 async plane without reopening parser, queue runner dispatch, or `operations_status` semantics.

**Companion artifacts:**

- [experiment_orchestration_async_recovery_v1.md](../evaluation/experiment_orchestration_async_recovery_v1.md) — retry lineage, recovery lifecycle (I3.1)
- [experiment_orchestration_replay_reconciliation_v1.md](../evaluation/experiment_orchestration_replay_reconciliation_v1.md) — replay supersession, fingerprint reconciliation (I3.1, I3.5)
- [orchestration_async_batch_audit_v1.md](../evaluation/orchestration_async_batch_audit_v1.md) — batch review schema (I3.3)
- [orchestration_recovery_report_v1.md](../evaluation/orchestration_recovery_report_v1.md) — per-manifest recovery report (I3.2)
- [orchestration_reconciliation_lineage_index_v1.md](../evaluation/orchestration_reconciliation_lineage_index_v1.md) — corpus lineage index (I3.3)
- [experiment_orchestration_async_recovery_governance_v1.md](../evaluation/experiment_orchestration_async_recovery_governance_v1.md) — governance-safe recovery semantics (I3.6)
- [sa_i3_async_recovery_governance_review_r1.md](../evaluation/sa_i3_async_recovery_governance_review_r1.md)
- [sa_i3_async_recovery_freeze_audit.md](../evaluation/sa_i3_async_recovery_freeze_audit.md)

---

## 1. Purpose

Stabilize the human/reviewer **recovery and reconciliation cognition layer** for async orchestration before federation scaling, multi-corpus operations, or large async replay studies.

I2 delivered detect-and-hold async primitives (fingerprints, quarantine, integrity audits, read-only async panel). I3 adds reviewer-oriented recovery docs, recovery/reconciliation audit tooling, batch async review artifacts, and extended read-only viewer cognition — **without** live orchestration, browser execution authority, or distributed workers.

**Platform direction:** deterministic large-scale replay experimentation governance — **not** realtime orchestration management or operational recovery infrastructure.

**Core rule (unchanged):** *Viewer observes; CLI/workers run and promote; reconcile forward, never patch history.*

---

## 2. Deliverables

| ID | Scope | Artifact |
|----|--------|----------|
| I3.1 | Recovery/reconciliation review docs | `async_recovery_v1`, `replay_reconciliation_v1`, batch/recovery/lineage schemas |
| I3.2 | Recovery audit tooling | `replay_sa_orchestration_recovery.py`, `audit_orchestration_recovery.py` |
| I3.3 | Batch async review artifacts | `fixtures/orchestration/recovery/`, `reconciliation/`, `synthesis/` |
| I3.4 | Read-only viewer cognition | `OrchestrationRecoveryPanel`, `OrchestrationBatchReviewPanel` |
| I3.5 | Replay reproducibility review | Fingerprint reconciliation, replacement continuity, recovered-equivalence checks |
| I3.6 | Governance-safe recovery semantics | `async_recovery_governance_v1` |
| I3.7 | Freeze audit + governance review; registry `PLAN-SA-I3` / `PLAT-SA-I3` | governance review + freeze audit |

---

## 3. Non-goals

Distributed worker infrastructure, live retry orchestration, browser-triggered recovery, realtime dashboards, live ROS/WebSocket, collaborative orchestration, auto-reconcile CLIs that mutate historical snapshots, parser/topic/schema redesign, tactical/operator workflows, ML scheduling, deployment semantics, federation, multi-corpus operations, default CI async lane.

**Stop line after PLAT-SA-I3 freeze:** federation, multi-corpus, distributed workers, live scheduling UX.

---

## 4. Depends on

PLAT-SA-I2, PLAT-SA-I1, PLAT-SA-H3, PLAT-SA-H4, PLAT-SA-A1, PLAT-SA-A2, PLAT-SA-R0–H5, PLAT-SA-STAB, PLAN-SA-H1.

---

## 5. Validation

See [sa_i3_async_recovery_freeze_audit.md](../evaluation/sa_i3_async_recovery_freeze_audit.md) regression block.

*End of PLAN-SA-I3.*
