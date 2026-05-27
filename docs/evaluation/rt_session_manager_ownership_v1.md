# RT Session Manager Module Ownership (`rt_session_manager_ownership_v1`)

**Phase:** PLAT-RT-R3a — module ownership contract  
**Authority:** [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md), [rt_r1_architecture_stabilization_review_r1.md](rt_r1_architecture_stabilization_review_r1.md)

This document defines ownership boundaries after PLAT-RT-R3a session manager decomposition. **Mirrors remain non-authoritative**; command routing remains in the facade.

---

## Facade

| Module | Role |
|--------|------|
| `session_manager.py` | Public `BridgeSessionManager` — command router, rate limit, session ownership (`_session`), telemetry pull API |
| `bridge_server.py` | HTTP transport wrapper (unchanged) |

The facade delegates to internal handler modules; it does **not** own domain logic.

---

## Ownership map

| Concern | Owner module(s) | Contract |
|---------|-----------------|----------|
| Session lifecycle state machine | `lifecycle.py`, `session_lifecycle_handlers.py` | [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md), [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md) |
| Per-session state bag | `session_record.py` | Additive R3a |
| Command response shaping | `session_response.py` | [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) |
| Teardown / cleanup sequencing | `session_teardown.py` | This document §3 |
| Adapter poll/resync result apply | `session_adapter_results.py` | [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md), [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md) |
| Runtime adapter subcommands | `session_runtime_commands.py`, `runtime_adapter.py`, `adapter_poll.py` | [rt_runtime_subcommand_registry_v1.md](rt_runtime_subcommand_registry_v1.md), [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md) |
| Entity commands + pose sync orchestration | `session_entity_handlers.py`, `adapter_sync.py`, `pose_sync.py` | [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md) |
| Telemetry publish + subscriptions | `session_telemetry_coordinator.py`, `telemetry_bridge.py`, `telemetry_subscriptions.py` | [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md) |
| Template / workflow hooks | `session_workflow_handlers.py`, `templates.py`, `workflow.py`, `template_resync.py` | [rt_workflow_contract_v1.md](rt_workflow_contract_v1.md) |
| Capture + normalization orchestration | `session_capture_handler.py`, `capture.py`, `capture_normalize.py`, `capture_pose_cognition.py` | [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md), [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md) |
| Audit append | `audit_log.py`, `export_audit_log.py` | [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md) |
| Governance / isolation | `governance.py`, `isolation.py`, `export_boundary.py`, `runtime_subcommand_governance.py` | [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md), [rt_runtime_subcommand_registry_v1.md](rt_runtime_subcommand_registry_v1.md) |

---

## Teardown sequencing (R3a)

Unified in `session_teardown.py`:

| Path | Sequence | Notes |
|------|----------|-------|
| Capture post-path | world → telemetry subs → runtime (with adapter audits) → null world/workflow | `teardown_capture_post_session` |
| Discard | same as capture teardown | `teardown_discarded_session` |
| Auto-cleanup (STOPPED) | world → telemetry → runtime (full adapter teardown) | `teardown_stopped_auto_cleanup` |
| Auto-cleanup (FAILED/RUNTIME_CRASHED/CLEANUP_PENDING) | world → telemetry → `runtime.terminate()` only | **No** adapter teardown audits — preserved behavior |
| Reset session | world reset inline; mirrors cleared via `clear_adapter_mirrors` pattern | Does not terminate runtime |
| Adapter detach | `runtime.terminate()` + `clear_adapter_mirrors` | Subcommand path |

`clear_adapter_mirrors(session)` centralizes pose + telemetry mirror nulling (G3/G4).

---

## Lifecycle transition ownership (PLAT-RT-R3b)

| Concern | Owner | Notes |
|---------|-------|-------|
| Command-gated transitions | `lifecycle.py` `can_transition()` | Matrix in [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md) §1 |
| Timeout-driven transitions | `session_lifecycle_handlers.tick_timeouts()` | §2 of transition contract |
| User lifecycle commands | `session_lifecycle_handlers.py` | start/pause/resume/stop/discard/reset |
| Reserved `bridge_disconnected` | `lifecycle.py` enum only | No handler sets state in R3b |

Maintainers: update transition contract when adding lifecycle states or changing `can_transition` rules.

---

## Maintainer guidance

1. **Add new bridge commands** in the facade router only after governance review; implement handler logic in the appropriate module above — not in `session_manager.py`.
2. **Cleanup changes** must go through `session_teardown.py` composed paths to avoid drift between discard/capture/auto-cleanup.
3. **Do not** move export-boundary or normalization pass/fail logic out of `capture.py` / `capture_normalize.py`.
4. **Tests** remain integration-style via `BridgeSessionManager.handle_command`; decomposition does not require handler-level unit tests unless a future wave adds them.

---

## RT↔SA boundary (unchanged)

- No SA viewer writes from RT modules
- No automatic replay ingestion
- Capture orchestration remains RT-local staging only

See [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md).

---

## 11. PLAN-RT-M1 multi-session extension (docs only — PLAT-RT-M2)

**Phase:** PLAN-RT-M1 — local single-bridge multi-session  
**Authority:** [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md)

After PLAT-RT-M2, the facade owns a **session registry** (`dict[str, SessionRecord]`) instead of sole `_session: SessionRecord | None`.

| Concern | Owner (M2) |
|---------|------------|
| Registry map + capacity | `session_registry.py` (new) via facade |
| Per-session lifecycle | `session_lifecycle_handlers.py` — parameterized by session lookup |
| Per-session teardown | `session_teardown.py` — unchanged composed paths |
| Per-session rate limit | Per-session limiter on facade (replaces global) |
| `editing_session_id` | Facade field — set via `set_editing_session` |
| `tick_timeouts` | Per non-terminal session |

Maintainer guidance additions:

5. **Registry mutations** must hold registry mutex — no direct dict access from handlers.
6. **Do not** share `WorldStateStore`, mirrors, or `RuntimeHandle` across sessions.
7. **Eviction** only after teardown completes — never delete mid-lifecycle.

Frozen single-session code paths remain until PLAT-RT-M2 replaces `_session` slot.
