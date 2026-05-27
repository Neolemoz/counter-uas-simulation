# RT-R2 — Platform Maturity Review R1

**Phase:** PLAN-RT-R2 — runtime platform maturity review (read-only)  
**Prerequisite:** PLAT-RT-X1 frozen; full primary RT roadmap delivered  
**Plan:** [rt_r2_runtime_platform_maturity_plan.md](../platform/rt_r2_runtime_platform_maturity_plan.md)  
**Governance review:** [rt_r2_platform_governance_review_r1.md](rt_r2_platform_governance_review_r1.md)  
**Technical debt:** [rt_r2_technical_debt_audit_r1.md](rt_r2_technical_debt_audit_r1.md)  
**Next frontiers:** [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md)  
**Freeze audit:** [rt_r2_platform_maturity_freeze_audit.md](rt_r2_platform_maturity_freeze_audit.md)  
**Baseline:** [rt_r1_architecture_stabilization_review_r1.md](rt_r1_architecture_stabilization_review_r1.md)

No runtime code was modified for this review wave.

---

## Executive summary

| Dimension | Verdict |
|-----------|---------|
| Platform completeness | **Pass** — primary RT roadmap implemented end-to-end |
| Architecture layering | **Pass** — browser → bridge → stub/adapter; mirrors non-authoritative |
| Governance / RT↔SA separation | **Pass** — re-validated; SA3 read-only replay only |
| Runtime consistency | **Pass-with-conditions** — lifecycle/tactical/telemetry/capture aligned; minor workflow template residual |
| UX maturity | **Pass** — research workbench, not operational C2 |
| Technical debt | **Pass-with-conditions** — R1 P0/P1 closed; `App.tsx` concentration residual |
| Expansion readiness | **Conditional** — see ranked next-frontier roadmap |

**Platform maturity:** The RT interactive sandbox has reached a **documented maturity plateau**. Multi-session sandbox, SVG + Cesium editing, Gazebo adapter path (mock default), terrain cognition, tactical modes (manual / assisted / autonomous), tactical capture continuity, RT→SA replay visibility (SA3), and experimentation workbench (X1) are delivered and frozen.

**Recommendation:** Freeze **PLAN-RT-R2** (docs only). **Recommended next major frontier** (advisory): **RT experiment analytics & template sweep catalog** — see [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md). Do not authorize implementation until a new scoped wave audit.

**Stop line:** No post-R2 implementation without plan + governance review + freeze audit.

---

## 1. Architecture review

### 1.1 Bridge and session layer

**Stack:** HTTP loopback → `BridgeSessionManager` facade ([session_manager.py](../../platform/rt-sandbox-bridge/rt_sandbox/session_manager.py) ~580 LOC) → `session_*_handlers.py` per concern ([rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md)).

| Check | Result |
|-------|--------|
| Deny-by-default command gate | **Pass** — `governance.py`, `classify_command` |
| Multi-session cap | **Pass** — `max_concurrent_sessions=3` (M2) |
| Session registry + list_sessions | **Pass** — `session_registry.py` |
| Writable roots isolation | **Pass** — `runs/rt_sandbox/` only |
| Facade delegates (post-R3a) | **Pass** — no ~1900-line monolith |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-ARCH-01 | Pass | Four-layer model intact: Browser → Bridge → Adapter/Stub → ROS/Gazebo (optional) |
| R2-ARCH-02 | Pass | `RuntimeStub` default when `enable_gazebo_adapter=false` |
| R2-ARCH-03 | Pass-with-conditions | Facade still central routing point — acceptable post-R3a |

### 1.2 Runtime adapter (G2–G6)

| Component | Role | Contract |
|-----------|------|----------|
| `runtime_adapter.py` | Attach/detach, IPC | [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md) |
| `adapter_poll.py` | Unified post-mutation tick | [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md) |
| `pose_sync.py` | Feedback mirror G3 | [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md) |
| `telemetry_bridge.py` | Telemetry mirror G4 | [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md) |
| `live_ros_client.py` + G6 stack | Live sync, visual fidelity | [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md) |
| `template_resync.py` | Template resync R2d | [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md) |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-ARCH-04 | Pass | R1b unified poll closure holds |
| R2-ARCH-05 | Pass | ROS allow-list when adapter enabled |
| R2-ARCH-06 | Pass-with-conditions | Live Gazebo path maintainer-heavy; not CI-gated end-to-end |

### 1.3 RT UI (`platform/rt-sandbox-ui`)

| Surface | Waves | Maturity |
|---------|-------|----------|
| Telemetry + banners | T1 | Frozen governance chrome |
| SVG world editing | T2 | Command-authoritative edits via bridge |
| Cesium runtime + interactive edit | T3, T5 | Fictional georef; dual-surface |
| Workstation / cognition | T4 | Session tabs, handoff guidance |
| Visualization fidelity | V1 | Markers, camera, accents |
| Terrain cognition | V2 | Fictional geometry; not sensor truth |
| Experiment workbench | X1 | Pin/compare; batch CLI only for capture |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-ARCH-07 | Pass | Loopback pull only; no rosbridge |
| R2-ARCH-08 | Pass-with-conditions | `App.tsx` (~628 LOC) concentrates integration — debt item R2-DEBT-UI-01 |
| R2-ARCH-09 | Pass | Isolation tests forbid browser `capture_session` |

### 1.4 Tactical controller (TAC1–TAC5)

| Mode | Bridge | UI | Capture |
|------|--------|-----|---------|
| Manual | `tactical_controller.py` | `TacticalManualPanel` | — |
| Assisted | `tactical_recommendation.py` | `TacticalAssistedPanel` | — |
| Autonomous | `tactical_autonomous.py` | `TacticalAutonomousPanel` | Bounded tick; pause/resume |
| Continuity | `tactical_capture_annex.py` | SA3 replay panel | At `capture_session` |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-ARCH-10 | Pass | Deny-by-default tactical commands; sandbox lexicon |
| R2-ARCH-11 | Pass | Autonomous loop controller-authoritative; not operational engage |
| R2-ARCH-12 | Pass | TAC5 annex embedded in normalized capture + SA3 read-only |

### 1.5 Experimentation stack (X1)

**Flow:** Pull telemetry → pin `rt_experiment_manifest_v1` → compare (live or pinned) → optional maintainer `rt_experiment_batch.py` for capture.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-ARCH-13 | Pass | No new bridge API; explanatory compare only |
| R2-ARCH-14 | Pass | Batch capture via `send_command` only; UI shows CLI |

---

## 2. Governance review (summary)

Full checklist: [rt_r2_platform_governance_review_r1.md](rt_r2_platform_governance_review_r1.md).

| Area | Verdict |
|------|---------|
| Authority boundaries | **Pass** |
| RT↔SA separation | **Pass** |
| Replay boundaries | **Pass** |
| Deny-by-default | **Pass** |
| Banner / lexicon discipline | **Pass** |

---

## 3. Runtime consistency review

### 3.1 Session lifecycle

Re-validated against [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md) and M2 multi-session rules.

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-CONS-01 | Pass | Per-session lifecycle; registry tracks ≤3 non-terminal |
| R2-CONS-02 | Pass | `capture_session` from stopped; normalization gate |
| R2-CONS-03 | Pass-with-conditions | R1-LIFE-04 residual: template apply vs workflow reset |

### 3.2 Tactical lifecycle

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-CONS-04 | Pass | Mode transitions manual → assisted → autonomous gated |
| R2-CONS-05 | Pass | `tactical_state` + `tactical_recommendation` telemetry channels |
| R2-CONS-06 | Pass | Autonomous pause/resume; no unbounded engage semantics |

### 3.3 Telemetry semantics

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-CONS-07 | Pass | Revision vocabulary in R1a; hint policy in R3d |
| R2-CONS-08 | Pass | Pull channels match T1 contract; mirrors labeled |
| R2-CONS-09 | Pass | Multi-session telemetry routing per M1/M2 |

### 3.4 Capture continuity

| Stage | Artifact |
|-------|----------|
| Staging | G5 normalization |
| Pose cognition | R2e tri-source interpretation |
| Tactical annex | TAC5 → staging `tactical_annex.json` |
| Handoff | SA1 maintainer import; SA2 mirror UI |
| Replay | SA3 `rt_tactical_replay_continuity` in bundle |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-CONS-10 | Pass | Capture ≠ SA import; lineage protection holds |
| R2-CONS-11 | Pass | X1 annex summary counts align with TAC5 contract |

### 3.5 Multi-session isolation

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-CONS-12 | Pass | Editing lock; session-scoped commands |
| R2-CONS-13 | Pass | No cross-session world mutation without explicit session_id |

---

## 4. UX maturity review

| Workstream | Maturity assessment |
|------------|---------------------|
| Runtime workstation (T4) | **Mature** for local research — workflow strip, cognition hub, capture/handoff panels |
| Cesium editing (T3/T5/V1) | **Mature** for sandbox prototyping — fictional georef, dual SVG/globe |
| Terrain cognition (V2) | **Mature** as explanatory overlay — not terrain truth |
| Tactical UX (TAC2–4) | **Mature** for sandbox assignment experiments — forbidden lexicon tested |
| Experimentation (X1) | **Mature** for A/B compare + maintainer batch — not distributed orchestration |
| SA replay visibility (SA3) | **Mature** read-only — timeline counts, no live hook |

| Finding ID | Verdict | Summary |
|------------|---------|---------|
| R2-UX-01 | Pass | Research workbench posture; banners consistent |
| R2-UX-02 | Pass | Multi-session tabs + background diagnostics usable |
| R2-UX-03 | Pass-with-conditions | Experiment manifest import manual — by design |

**UX verdict:** Suitable for **mentor/demo/runtime research** workflows. Not suitable as operational C2 or readiness dashboard (by governance design).

---

## 5. R1 re-validation summary

All R1 P0 findings closed by R1a–R3d, T*, M2, TAC*, X1 waves. Residuals documented in [rt_r2_technical_debt_audit_r1.md](rt_r2_technical_debt_audit_r1.md). No new P0 architecture blockers identified at plateau.

---

## 6. Platform maturity statement

The **primary RT interactive sandbox roadmap is complete**. The platform exhibits:

- **Coherent layering** with frozen contracts per wave
- **Governance-aware UX** that repeatedly states non-authority of mirrors and captures
- **Bounded tactical sandbox** without operational intercept semantics
- **Manual RT→SA bridge** with read-only replay cognition
- **Local experimentation** without distributed or auto-import scope creep

Future work should be **selective frontier expansion**, not ad-hoc feature accretion. See [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md).

---

## Appendix — Subsystem cross-reference

| Subsystem | Contracts | Implementation | Tests |
|-----------|-----------|----------------|-------|
| Bridge | `rt_bridge_contract_v1`, ownership v1 | `session_manager` + handlers | `test_rt_sandbox_bridge.py` |
| Adapter | G1–G6, poll semantics | `runtime_adapter`, `adapter_poll` | bridge pytest |
| UI | T1–T5, V1–V2, X1 | `platform/rt-sandbox-ui` | vitest + tier0-rt-ui |
| Tactical | TAC1–5 | `tactical_*` modules | bridge + panel tests |
| RT→SA | R2f, SA1–3 | handoff CLIs, SA viewer panel | bridge + packager |
| Experiment | `rt_experiment_workbench_v1` | `experiment/`, batch CLI | vitest + `test_rt_experiment_batch.py` |
