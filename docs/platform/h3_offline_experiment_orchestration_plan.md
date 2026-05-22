# PHASE H3 — Offline Experiment Orchestration Foundations (PLAT-SA-H3)

**Phase:** H3 — Offline Experiment Orchestration Foundations  
**Checkpoint:** post PLAT-SA-H2 (`75ea6d5` lineage)  
**Build recommendation:** scoped implementation — CLI orchestration + read-only viewer mirrors  
**Authority:** [AGENTS.md](../../AGENTS.md) primary; extends [h1_sandbox_ux_architecture_plan.md](h1_sandbox_ux_architecture_plan.md) §12

**Companion schemas:**

- [experiment_job_manifest_v1.md](../evaluation/experiment_job_manifest_v1.md)
- [experiment_run_queue_v1.md](../evaluation/experiment_run_queue_v1.md)
- [experiment_workflow_scenario_to_replay_v1.md](../evaluation/experiment_workflow_scenario_to_replay_v1.md)
- [sa_h3_offline_orchestration_freeze_audit.md](../evaluation/sa_h3_offline_orchestration_freeze_audit.md)

---

## 1. Purpose

Establish **deterministic offline orchestration foundations** so the web platform can present a coherent **experimentation sandbox** workflow:

```
scenario manifest → offline runner → [Gazebo/ROS2] → replay bundle → review workspace
```

The **browser never controls Gazebo**. Execution remains CLI/CI. The viewer consumes **frozen queue snapshots** only.

---

## 2. Goals and non-goals

### Goals

| ID | Goal |
|----|------|
| G1 | `experiment_job_manifest_v1` for declarative pipelines |
| G2 | `experiment_run_queue_v1` snapshot for status mirrors |
| G3 | `run_experiment_queue.py` sequential offline runner |
| G4 | Read-only orchestration UI (`discover.job_status`, `discover.validation_status`) |
| G5 | Provenance mirrors linking scenario, log, bundle, corpus |
| G6 | CI-safe synthetic default path |

### Non-goals (H3 / stop before H4)

- Live ROS/WebSocket in viewer  
- Browser-triggered sim or regen  
- Parallel workers / Redis / daemons  
- HITL, C2, readiness scoring, ML recommendations  
- Parser/topic changes  
- Default CI with live Gazebo capture  

---

## 3. Architecture

See [experiment_workflow_scenario_to_replay_v1.md](../evaluation/experiment_workflow_scenario_to_replay_v1.md).

**H1 hook realization:** `sandbox_orchestration_context_v0` is implemented as manifest ref + queue snapshot + governance banner.

---

## 4. Deliverables

| ID | Deliverable |
|----|-------------|
| D1 | Python: `experiment_orchestration.py`, `run_experiment_queue.py`, `lint_experiment_manifest.py` |
| D2 | Fixtures: `fixtures/orchestration/` |
| D3 | Viewer mirrors: `public/demo/orchestration/` |
| D4 | Viewer panels: `OrchestrationStatusPanel`, `ValidationStatusPanel` |
| D5 | Tests + `tier0-sa-r0` H3 checks |
| D6 | Freeze audit + registry row |

---

## 5. H4 boundary

- Real async workers  
- Capture automation in default CI  
- Orchestration promotion / multi-tenant queues  
- Authoring inline edit (still forbidden without new wave)  

---

*End of PLAT-SA-H3 plan.*
