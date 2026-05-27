# RT-TAC1 — Architecture Review R1

**Phase:** PLAN-RT-TAC1 — tactical controller architecture (docs only)  
**Prerequisite:** PLAT-RT-M2, PLAT-RT-V1 frozen

Plan: [rt_tac1_tactical_controller_architecture_plan.md](../platform/rt_tac1_tactical_controller_architecture_plan.md)  
Governance review: [rt_tac1_governance_review_r1.md](rt_tac1_governance_review_r1.md)  
Freeze audit: [rt_tac1_freeze_audit.md](rt_tac1_freeze_audit.md)

---

## 1. Baseline vs proposal

### 1.1 RT Bridge (frozen baseline)

| Aspect | Today (PLAT-RT-M2) | After TAC waves (proposed) |
|--------|-------------------|----------------------------|
| Command surface | Entity + session + telemetry + workflow | Add tactical verbs per roadmap |
| Tactical module | Absent | `TacticalController` per `SessionRecord` |
| Allow-list | Deny `engage`/`intercept`/`strike` | Deny-by-default `tactical_*` until per-wave extension |
| Authority | `command_authoritative` registry | Mode-dependent tactical labels |
| Multi-session | cap=3, editing lock | Tactical state per session |

### 1.2 RT UI (frozen baseline)

| Aspect | Today | After TAC waves |
|--------|-------|-----------------|
| Entity editing | SVG + Cesium, `interceptor` type | Unchanged core |
| Tactical UX | None | Mode selector, candidate panel (TAC2+), approval (TAC3) |
| Telemetry | Adapter + session health | `tactical_state` channel (TAC2+) |

### 1.3 Runtime Adapter (frozen baseline)

| Aspect | Today | After TAC waves |
|--------|-------|-----------------|
| Role | Pose sync, telemetry mirrors | Unchanged — no selection policy in adapter |
| ROS boundary | Allow-list topics | No tactical topic additions without wave audit |

### 1.4 Engine stack (reference only)

| Aspect | Engine (`interception_logic_node`) | RT tactical controller |
|--------|-----------------------------------|------------------------|
| Runs in | Gazebo launch (optional) | Bridge process (future) |
| Authority | Engine/replay tooling topics | RT session sandbox modes |
| Logs | `[TACTICAL_*]` explanatory | RT audit `event_kind: tactical` |
| Coupling | **None required** for TAC2 manual | Optional read-only mirror |

---

## 2. Layer insertion review

| Criterion | Result | Evidence |
|-----------|--------|----------|
| Controller behind bridge allow-list | **Pass** | [rt_tac1_tactical_controller_layers_v1.md](rt_tac1_tactical_controller_layers_v1.md) §3 |
| Controller in front of adapter | **Pass** | Same — motion intents re-validated by bridge |
| No UI → ROS shortcut | **Pass** | Unchanged bridge contract |
| Per-session isolation | **Pass** | [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md) §9 |
| Teardown clears tactical state | **Pass** | Layers doc §3 + ownership alignment |

---

## 3. Mode architecture review

| Criterion | Result | Evidence |
|-----------|--------|----------|
| Manual default at session start | **Pass** | [rt_tac1_tactical_modes_v1.md](rt_tac1_tactical_modes_v1.md) §4 |
| Assisted approval gate | **Pass** | Modes §1.2, governance §7 |
| Autonomous bounded + revert | **Pass** | Modes §1.3, roadmap TAC4 |
| Mode ≠ SA authority | **Pass** | Modes §4 invariant 2 |

---

## 4. Logic reuse review

| Criterion | Result | Evidence |
|-----------|--------|----------|
| Pure-function solver reuse path | **Pass** | [rt_tac1_tactical_logic_reuse_v1.md](rt_tac1_tactical_logic_reuse_v1.md) §2 |
| TTI-at-cap consistency | **Pass** | Logic reuse §3 |
| Lock vs Assisted approval | **Pass** | Logic reuse §4 |
| `[TACTICAL_*]` non-authoritative | **Pass** | Logic reuse §6, reviewer guide |

---

## 5. Compatibility with frozen RT waves

| Frozen wave | Compatibility |
|-------------|---------------|
| PLAT-RT-M2 multi-session | Tactical state per session; editing lock applies |
| PLAT-RT-G6 live sync | Adapter remains non-policy; pose commands unchanged in TAC1 |
| PLAT-RT-T4/T5 workstation | Future tactical panels additive |
| PLAT-RT-SA1/SA2 handoff | Capture annex in TAC5 only; no live handoff change in TAC1 |
| PLAT-RT-R3a session decomposition | Controller module fits handler decomposition pattern |

---

## 6. Risks and mitigations (planning)

| Risk | Mitigation documented |
|------|----------------------|
| Operational lexicon drift | [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md) §5 |
| Engine/RT authority confusion | Layers §5, logic reuse §6 |
| Assisted auto-apply | Forbidden until TAC3; explicit approval verb |
| Cross-session assignment | Forbidden in governance §6, §9 |
| Parser contamination | No parser fields in any TAC contract |

---

## 7. Verdict

**Pass** — architecture is sufficiently specified for PLAN-RT-TAC1 docs freeze. Implementation deferred to PLAT-RT-TAC2+ per [rt_roadmap_tac1_tac5_v1.md](rt_roadmap_tac1_tac5_v1.md).

**Stop line:** Do not implement bridge or UI changes until PLAT-RT-TAC2 wave audit.

---

## Related

- [rt_tac1_governance_review_r1.md](rt_tac1_governance_review_r1.md)
- [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md)
- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)
