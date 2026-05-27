# RT Multi-Session Registry Contract (`rt_multi_session_registry_v1`)

**Phase:** PLAN-RT-M1 — multi-session architecture (docs only)  
**Authority:** [rt_m1_multi_session_architecture_plan.md](../platform/rt_m1_multi_session_architecture_plan.md); [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md); [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)

Defines bridge session registry semantics for local single-bridge multi-session support. **No implementation in PLAN-RT-M1.**

---

## 1. Registry model

| Field | Rule |
|-------|------|
| Owner | `BridgeSessionManager` facade (future `session_registry.py` in PLAT-RT-M2) |
| Structure | `dict[str, SessionRecord]` keyed by `session_id` |
| Capacity | `non_terminal_count ≤ max_concurrent_sessions` (3) |
| Eviction | Terminal sessions removed after teardown completes |

Replaces current single-slot `_session: SessionRecord | None`.

---

## 2. Session states in registry

| Category | Lifecycle states | Registry presence |
|----------|------------------|-------------------|
| **Non-terminal active** | `created`, `running`, `paused`, `stopped`, `cleanup_pending`, `failed`, `runtime_crashed` | Retained |
| **Terminal** | `discarded`, `captured` | Evicted after teardown |

Lifecycle transitions unchanged — see [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md).

**Active vs background** is a **UI designation only** — not a lifecycle state.

---

## 3. Session creation

### 3.1 `start_session` (revised precondition)

| Precondition | Current (frozen) | M1 (future M2) |
|--------------|------------------|----------------|
| Capacity | No active session on bridge | `non_terminal_count < max_concurrent_sessions` |
| Prior session | Must be terminal or absent | Non-terminal sessions do not block if under cap |

On success:

1. Allocate new UUID `session_id`
2. Create isolated `SessionRecord` with `WorldStateStore`, `RuntimeHandle`, mirrors
3. Register in session registry
4. If first session or no editing session set → assign as `editing_session_id`
5. Audit + lifecycle telemetry publish

### 3.2 Capacity rejection

When `non_terminal_count ≥ max_concurrent_sessions`:

- Error code: `SESSION_CAPACITY_EXCEEDED`
- No partial session allocation
- Existing sessions unaffected

---

## 4. Session removal

| Trigger | Path | Registry effect |
|---------|------|-----------------|
| `discard_session` | `teardown_discarded_session` | Evict after teardown |
| `capture_session` | `teardown_capture_post_session` | Evict after teardown |
| Auto-cleanup (timeout) | `teardown_stopped_auto_cleanup` / `teardown_failed_auto_cleanup` | Evict after teardown |
| Registry eviction | Post-teardown hook | Remove `session_id` from dict |

If evicted session was `editing_session_id`, bridge assigns editing lock to another non-terminal session (if any) or clears lock.

---

## 5. Session ownership

| Owner | Responsibility |
|-------|----------------|
| **Bridge registry** | Authoritative session existence, lifecycle, runtime handles |
| **Bridge `editing_session_id`** | Authoritative entity mutation target |
| **UI `selectedSessionId`** | Active workspace display target |
| **UI `editingSessionId`** | Mirror of bridge lock; set via `set_editing_session` |

Default: `selectedSessionId == editingSessionId` on tab selection.

---

## 6. New commands (PLAT-RT-M2)

### 6.1 `list_sessions`

Read-only registry snapshot. No session required.

**Response payload:**

| Field | Type | Notes |
|-------|------|-------|
| `sessions[]` | array | Up to 3 entries |
| `sessions[].session_id` | string | UUID |
| `sessions[].state` | string | Lifecycle state |
| `sessions[].created_monotonic` | float | Bridge monotonic clock |
| `sessions[].entity_count` | int | From world registry |
| `sessions[].is_editing` | bool | `session_id == editing_session_id` |
| `editing_session_id` | string \| null | Current editing lock |
| `capacity` | int | `max_concurrent_sessions` |
| `non_terminal_count` | int | Current occupancy |

### 6.2 `set_editing_session`

| Precondition | Effect |
|--------------|--------|
| Target `session_id` exists and is non-terminal | Set `editing_session_id`; audit event |
| Target terminal or unknown | `SESSION_NOT_FOUND` or `INVALID_STATE` |

Payload: `{ "session_id": "uuid" }`

---

## 7. Command routing (revised)

All session-scoped commands:

1. Resolve `session_id` from request body
2. Lookup in registry — unknown → `SESSION_NOT_FOUND`
3. Validate lifecycle preconditions per command
4. Entity mutations additionally require `session_id == editing_session_id`

`tick_timeouts()` runs **per non-terminal session** independently.

---

## 8. ROS / adapter isolation

Each `SessionRecord` gets:

- Isolated `RuntimeHandle` (stub or Gazebo adapter worker)
- `ros_domain_id_for_session(session_id)` when adapter enabled
- Independent audit log path via `audit_log.path_for(session_id)`

One session adapter crash → that session may → `failed` / `runtime_crashed`; siblings continue unless resource exhaustion.

---

## 9. Explicit non-goals

- Distributed registry across bridge processes
- Session migration between bridges
- Persistent session storage / resume after bridge restart
- Multi-user session ownership or RBAC

---

## Related

- [rt_multi_session_governance_v1.md](rt_multi_session_governance_v1.md)
- [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md)
- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §11 (M1 additive)
