# RT Lifecycle Transitions (`rt_lifecycle_transitions_v1`)

**Phase:** PLAT-RT-R3b — lifecycle documentation hardening (R1-LIFE-02)  
**Authority:** [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md); [lifecycle.py](../../platform/rt-sandbox-bridge/rt_sandbox/lifecycle.py)

Implementation truth table for RT sandbox session lifecycle. Distinguishes **implemented** transitions from **reserved** states documented in RT-S1 but not yet wired in bridge handlers.

---

## 1. Command-gated transitions (implemented)

Source: `lifecycle.py` `_TRANSITIONS` and `can_transition()`. Handlers in `session_lifecycle_handlers.py`, `session_entity_handlers.py`, `session_workflow_handlers.py`, `session_capture_handler.py`.

| Command | Allowed source states | Handler owner |
|---------|----------------------|---------------|
| `start_session` | (no active session) | `session_lifecycle_handlers.start_session` |
| `pause_session` | `running` | `session_lifecycle_handlers.pause_session` |
| `resume` | `paused` | `session_lifecycle_handlers.resume_session` |
| `stop_session` | `running`, `paused` | `session_lifecycle_handlers.stop_session` |
| `reset_session` | `running`, `paused` | `session_lifecycle_handlers.reset_session` |
| `spawn_entity`, `move_entity`, `delete_entity` | `running`, `paused` | `session_entity_handlers.handle_entity` |
| `subscribe_telemetry` | `running`, `paused` | `session_telemetry_coordinator.handle_telemetry` |
| `unsubscribe_telemetry` | `running`, `paused`, `stopped` | `session_telemetry_coordinator.handle_telemetry` |
| `capture_session` | `stopped` only | `session_capture_handler.capture_session` |
| `discard_session` | `created`, `running`, `paused`, `stopped`, `captured`, `failed`, `runtime_crashed`, `bridge_disconnected`, `cleanup_pending` | `session_lifecycle_handlers.discard_session` |
| Template/workflow commands | `running`, `paused` (except `list_runtime_templates` — no session required) | `session_workflow_handlers` |
| `send_runtime_command` | `running`, `paused` (when adapter active) | `session_runtime_commands.handle_runtime_command` |

**Terminal states:** `captured`, `discarded` — all commands rejected (`can_transition` → false).

**Special rule:** `cleanup_pending` — only `discard_session` permitted (all other commands → `INVALID_STATE`).

**Not in `_TRANSITIONS`:** `send_runtime_command` — gated by governance + adapter state in runtime command handler, not `can_transition()`.

---

## 2. Timeout- and bridge-driven transitions (implemented)

Source: `session_lifecycle_handlers.tick_timeouts()`. Invoked at start of every `handle_command()`.

| Trigger | From | To | Cleanup | Audit `command_type` |
|---------|------|-----|---------|---------------------|
| `bridge_ready_timeout` | `created` (deadline elapsed) | `failed` | world + telemetry clear; runtime terminate; schedules `cleanup_after` | `bridge_ready_timeout` (result: `failed`) |
| `max_session_duration` | `running` | `stopped` | schedules `cleanup_after` | `max_session_duration` |
| Passive runtime death | `running`, `paused` | `runtime_crashed` | schedules `cleanup_after` (`cleanup_pending_max_age_s`) | `runtime_crashed` |
| Auto-cleanup (STOPPED) | `stopped` | `cleanup_pending` → `discarded` | full teardown via `teardown_stopped_auto_cleanup` | `auto_cleanup` |
| Auto-cleanup (FAILED/RUNTIME_CRASHED/CLEANUP_PENDING) | same | `discarded` | partial via `teardown_failed_auto_cleanup` | `auto_cleanup` |

**`CAPTURED` exemption:** `tick_timeouts` returns early when state is `captured` — no auto-cleanup until explicit `discard_session`.

**Note:** Happy-path `start_session` transitions `created → running` synchronously before client observes `created`; `bridge_ready_timeout` applies only if `created` persists past deadline (failure path).

---

## 3. Failure-state teardown comparison

| Path | Helper | World | Telemetry subs | Runtime | Adapter teardown audits |
|------|--------|-------|----------------|---------|------------------------|
| Discard / capture post | `teardown_discarded_session` / `teardown_capture_post_session` | clear + audit | clear + audit | `terminate_runtime_with_audit` | Yes (`adapter_teardown`, `orphan_cleanup`, feedback lost) |
| Auto-cleanup from STOPPED | `teardown_stopped_auto_cleanup` | clear + audit | clear + audit | `terminate_runtime_with_audit` | Yes |
| Auto-cleanup from FAILED/RUNTIME_CRASHED | `teardown_failed_auto_cleanup` | clear + audit | clear + audit | `runtime.terminate()` only | **No** adapter teardown audits |
| Bridge ready timeout | inline in `tick_timeouts` | clear + audit | clear + audit | `runtime.terminate()` | No (pre-adapter attach path) |
| Runtime crash on command | `lifecycle_runtime_crashed` | mirrors cleared | — | — | Adapter audits if adapter was active |

Partial teardown on FAILED/RUNTIME_CRASHED auto-cleanup is **intentional preserved behavior** (PLAT-RT-R3a); not a bug.

---

## 4. Reserved states

### 4.1 `bridge_disconnected`

| Aspect | Status |
|--------|--------|
| Enum in `SessionState` | Yes |
| `can_transition` rules (discard allowed) | Yes |
| Bridge handler sets this state | **No** (PLAT-RT-R3b) |
| Reconnect timeout in `GovernanceConfig` | **No** (governance constant documented only) |

Future wave may implement: transport-loss detection → `bridge_disconnected` → reconnect window → `failed`. **Not authorized in R3b.**

### 4.2 `bridge_disconnected_reconnect_timeout`

Documented in [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) as **30 s** governance constant. **Not wired** to `GovernanceConfig` or `tick_timeouts` in PLAT-RT-R3b.

---

## 5. Disconnect vocabulary

Three distinct concepts — do not conflate:

| Concept | Surface | Scope | Implemented? |
|---------|---------|-------|--------------|
| **HTTP transport loss** | Error code `BRIDGE_DISCONNECTED` | Client cannot reach loopback bridge ([rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §6) | Client-side only (`rt_bridge_client.py`) |
| **Session `bridge_disconnected`** | `SessionState` value | UI/bridge transport lost; runtime may continue | **Reserved** — enum + rules only |
| **Adapter IPC loss** | `runtime_crashed`, `adapter_feedback_lost`, `telemetry_feedback_lost` | Gazebo adapter worker exit or IPC failure | **Yes** — see [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md) |

Adapter IPC disconnect maps to **`runtime_crashed`** / **`failed`**, not session `bridge_disconnected`.

---

## 6. State precedence

1. **Cleanup wins over capture** — from `cleanup_pending`, only `discard_session`; `capture_session` → `INVALID_STATE`.
2. **Capture only from stable `stopped`** — not from failure states or `cleanup_pending`.
3. **Failure blocks capture** — `failed`, `runtime_crashed`, `bridge_disconnected` cannot reach `captured`.
4. **`captured` is terminal** for capture path — no return to `running`.
5. **`CLEANUP_PENDING` is transient** — set synchronously within discard and STOPPED auto-cleanup handlers; not observable across concurrent commands in single-threaded bridge.

---

## 7. Authority

| Actor | Role |
|-------|------|
| RT Backend Bridge | Sole authority for session state transitions |
| RT Browser UI | Requests via bridge commands only |
| Failure states | **Non-authoritative** — no SA replay, corpus, or federation writes |
| SA viewer / orchestration | **None** — read-only frozen artifacts |

All failure semantics are **explanatory only** — see [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md).

---

## 8. Related

- [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md) — conceptual lifecycle (RT-S1)
- [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) — module ownership
- [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md) — lifecycle audit events
- [rt_r3b_lifecycle_doc_hardening_plan.md](../platform/rt_r3b_lifecycle_doc_hardening_plan.md)
