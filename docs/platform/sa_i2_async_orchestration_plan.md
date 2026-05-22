# PHASE I2 — Async Orchestration Foundations (PLAN-SA-I2 / PLAT-SA-I2)

**Phase:** I2 — Async Orchestration Safety & Determinism Model + foundations implementation  
**Checkpoint:** PLAT-SA-I1, PLAT-SA-H3, PLAT-SA-A1, PLAT-SA-A2 frozen; **PLAT-SA-I2 frozen**  
**Authority:** [AGENTS.md](../../AGENTS.md); extends frozen I1/H3 orchestration without reopening parser, queue runner dispatch, or `operations_status` semantics.

**Companion artifacts:**

- [experiment_orchestration_async_model_v1.md](../evaluation/experiment_orchestration_async_model_v1.md) — worker topology, determinism, lifecycle proposals, failure/recovery (I2.1, I2.3, I2.4)
- [experiment_orchestration_async_safety_v1.md](../evaluation/experiment_orchestration_async_safety_v1.md) — safety boundaries, viewer posture (I2.2, I2.7)
- [experiment_orchestration_async_governance_v1.md](../evaluation/experiment_orchestration_async_governance_v1.md) — CI constraints, async integrity model (I2.5, I2.6)
- [sa_i2_async_orchestration_governance_review_r1.md](../evaluation/sa_i2_async_orchestration_governance_review_r1.md)
- [sa_i2_async_orchestration_freeze_audit.md](../evaluation/sa_i2_async_orchestration_freeze_audit.md)

---

## 1. Purpose

Define the governance-safe **Async Orchestration Safety & Determinism Model** before async workers, CI orchestration scaling, distributed queue execution, large-scale replay generation, or orchestration federation.

I1 operationalized deterministic offline orchestration (ops sidecar, integrity audits, replay continuity). I2 closes the **planning gap** for how async execution may evolve without becoming a live orchestration service, mission scheduler, or operational command infrastructure.

**Platform direction:** deterministic large-scale experimentation infrastructure — **not** live orchestration, realtime mission scheduling, or deployment readiness semantics.

**Core rule (unchanged):** *Viewer observes; CLI/workers run and promote.*

---

## 2. Deliverables

| ID | Scope | Artifact |
|----|--------|----------|
| I2.1 | Async orchestration planning model: worker topology, queue isolation, replay-generation boundaries, deterministic execution bookkeeping, failure/recovery semantics, scaling boundaries | [experiment_orchestration_async_model_v1.md](../evaluation/experiment_orchestration_async_model_v1.md) |
| I2.2 | Async safety boundaries: no browser authority, no runtime streaming, no nondeterministic replay gen, no hidden queue mutation, no live operational semantics | [experiment_orchestration_async_safety_v1.md](../evaluation/experiment_orchestration_async_safety_v1.md) |
| I2.3 | Deterministic async constraints: replay fingerprint, execution reproducibility, queue replayability, audit continuity, worker provenance, artifact lineage | `async_model_v1` § determinism |
| I2.4 | Lifecycle expansion model (read-only semantics): proposed `async_execution_status` (`retrying`, `failed`, `quarantined`, `superseded`) — parallel to frozen `operations_status` | `async_model_v1` § lifecycle |
| I2.5 | CI orchestration boundaries: optional/offline capture, explicit flags, no automatic runtime authority escalation, bounded execution environments | [experiment_orchestration_async_governance_v1.md](../evaluation/experiment_orchestration_async_governance_v1.md) |
| I2.6 | Async integrity/audit model: orphan workers, stale replay, partial recovery, replay continuity auditing, queue reconciliation | `async_governance_v1` |
| I2.7 | Future-safe viewer posture: explanatory-only, read-only, replay-oriented; no orchestration control UI | `async_safety_v1` § viewer |
| I2.8 | Freeze audit + governance review; registry row `PLAN-SA-I2` | governance review + freeze audit |

---

## 3. Non-goals

Async worker implementation, distributed execution systems, browser-triggered orchestration, live ROS/WebSocket/rosbridge in eval tooling, runtime streaming into viewer, collaborative orchestration, realtime dashboards, parser/topic/schema redesign, tactical/operator workflows, ML scheduling or recommendation systems, deployment readiness semantics, default CI Gazebo capture lane, orchestration federation, implementing deferred H3 pipeline step types (`observability`, `narrative`, `bundle_pack`) as live runner stages, changes to frozen `operations_status` or H3 step dispatch.

**PLAT-SA-I2 implementation complete** — see [sa_i2_async_orchestration_freeze_audit.md](../evaluation/sa_i2_async_orchestration_freeze_audit.md). Stop line before distributed workers, browser authority, federation.

---

## 4. Depends on

PLAT-SA-H3, PLAT-SA-I1, PLAT-SA-H4, PLAT-SA-A1, PLAT-SA-A2, PLAT-SA-C1a/C1b, PLAT-SA-R0, PLAT-SA-STAB, PLAT-SA-F1a–F1d, PLAN-SA-H1.

---

## 5. Validation

See [sa_i2_async_orchestration_freeze_audit.md](../evaluation/sa_i2_async_orchestration_freeze_audit.md) regression block. Documentation-only wave: existing platform gates must remain green; no code, fixture, or viewer changes in PLAN-SA-I2.

*End of PLAN-SA-I2.*
