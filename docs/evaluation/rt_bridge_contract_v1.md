# RT Bridge Contract (`rt_bridge_contract_v1`)

**Phase:** PLAN-RT-S1 — interactive runtime sandbox (docs only)  
**Schema version:** `rt_bridge_contract_v1`  
**Authority:** [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md)

Governance-safe command contract between **RT Browser UI** and **RT Backend Bridge**. No server implementation in RT-S1.

---

## 1. Transport (future RT-S2)

Prototype phases assume **local-only** RPC (HTTP loopback or unix domain socket). Not WebSocket in SA viewer; not rosbridge from browser.

### 1.1 RT-S2 transport profile (readiness clarification)

| Allowed (RT-S2+) | Forbidden |
|------------------|-----------|
| Loopback HTTP or WebSocket **to RT Backend Bridge only** (127.0.0.1 / unix socket) | rosbridge or legacy [web/](../../web/) stack for RT |
| Bridge mediates all runtime access | Browser → ROS direct publish/subscribe |
| RT-dedicated client (future RT UI) | WebSocket or live hooks in `platform/sa-r0-viewer/` |
| | Public network exposure |

RT-S2 may use WebSocket as a **local transport to the bridge** if declared in the PLAT-RT-S2 wave audit. That is **not** rosbridge and **not** SA viewer streaming.

Every response includes:

```json
{
  "governance_banner": "RT SANDBOX — experimental simulation; not operational state",
  "session_id": "uuid",
  "command_id": "uuid",
  "ok": true,
  "error_code": null
}
```

---

## 2. Common request fields

| Field | Required | Description |
|-------|----------|-------------|
| `command_type` | yes | Allow-listed command name |
| `session_id` | when session active | Ephemeral session UUID |
| `command_id` | yes | Client-generated idempotency key |
| `issued_by` | yes | `rt_ui_prototype` or `maintainer_cli` |
| `authority_scope` | yes | Always `rt_sandbox_prototype` |

---

## 3. Allow-listed commands (prototype)

### 3.1 Session control

| Command | Preconditions | Effect |
|---------|---------------|--------|
| `start_session` | No active session on bridge | `created` → `running` |
| `pause_session` | `running` | → `paused` |
| `resume` | `paused` | → `running` |
| `reset_session` | `running` or `paused` | Reset world; stay in session |
| `stop_session` | `running` or `paused` | → `stopped` |

### 3.2 Entity operations

| Command | Constraints |
|---------|-------------|
| `spawn_entity` | Entity type in prototype catalog; count ≤ `max_entity_count` |
| `move_entity` | Entity must exist in session |
| `delete_entity` | Entity must exist in session |

#### Entity catalog v1 (PLAT-RT-S3)

| `entity_type` | Max per session | Notes |
|---------------|-----------------|-------|
| `radar` | 8 | Static/sensor placeholder — no threat classification |
| `interceptor` | 8 | Movable sandbox actor — not engage/intercept commands |
| `drone` | 8 | Movable sandbox actor |
| `waypoint_marker` | 8 | Static marker |
| **Total** | ≤ `max_entity_count` (32) | No fleet or order-of-battle semantics |

**World bounds (prototype):** `x,y ∈ [-500, 500]`, `z ∈ [0, 200]` (prototype units). Out-of-bounds pose → `INVALID_POSE`.

Types not in this table → `COMMAND_FORBIDDEN` until a new wave extends the catalog.

**Payload:** `spawn_entity` / `move_entity` require `payload` with `entity_type`, `pose` (`x`, `y`, `z`, optional `yaw_deg`). `delete_entity` requires `payload.entity_id`. Optional `payload.entity_id` on spawn (bridge assigns UUID if omitted).

### 3.3 Runtime commands

| Command | Notes |
|---------|-------|
| `send_runtime_command` | Sub-command allow-list only; default **deny-all** |

Prototype sub-commands (illustrative):

- `set_clock_pause` (when not using `pause_session`)
- `reload_world_config` (maintainer-only flag in future)

### 3.4 Telemetry

| Command | Notes |
|---------|-------|
| `subscribe_telemetry` | Read-only; rate ≤ `telemetry_update_rate_cap_hz` |
| `unsubscribe_telemetry` | Drop subscription |

Telemetry path is **read-only** — no writes via subscription channel.

#### Telemetry allow-list v1 (PLAT-RT-S4)

| `channel` | Payload (read-only) | Cap |
|-----------|---------------------|-----|
| `session_health` | `state`, `stub_alive`, `governance_banner` | ≤ 10 Hz aggregate |
| `lifecycle_state` | `state`, `previous_state`, `command_type` | ≤ 10 Hz aggregate |
| `world_summary` | `entity_count`, `revision`, `by_type`, `bounds` | ≤ 10 Hz aggregate |
| `entity_pose_mirror` | `entities[]` with session-scoped poses | ≤ 10 Hz aggregate |
| `clock_mirror` | `paused` (derived from session state) | ≤ 10 Hz aggregate |

**Pull transport (local-only):** `GET /v1/telemetry/pull?session_id=&subscription_id=&max_events=` — read-only drain of subscribed channel events. Loopback bridge only; not exposed in SA viewer.

**Subscription limits:** max **5** channels per subscription; max **1** active subscription per session (resubscribe replaces).

**Forbidden subscriptions:** `/tracks/state`, weapon/engage topics, federation/orchestration status, tactical/readiness channels.

Topics outside this table → `COMMAND_FORBIDDEN` until a new wave extends the allow-list.

### 3.5 Session outcome

| Command | Preconditions |
|---------|---------------|
| `capture_session` | `stopped` only; emits capture candidate |
| `discard_session` | Any non-terminal except `discarded` |

#### Capture response v1 (PLAT-RT-S5)

On success, bridge returns:

| Field | Notes |
|-------|-------|
| `capture_candidate_id` | UUID for staging dir |
| `staging_refs` | `snapshot_ref`, `capture_report_ref`, `audit_ref`, `telemetry_summary_ref` |
| `state` | `captured` |
| `governance_banner` | Capture-specific non-authority banner |

Staging root: `runs/rt_sandbox/captures/<capture_candidate_id>/`. No automatic SA import.

### 3.6 Templates and workflows (PLAT-RT-S6)

See [rt_workflow_contract_v1.md](rt_workflow_contract_v1.md).

| Command | Preconditions |
|---------|---------------|
| `list_runtime_templates` | None or active session |
| `apply_runtime_template` | `running` or `paused` |
| `start_workflow` | `running` or `paused`; workflow idle |
| `advance_workflow` | workflow `in_progress` |
| `reset_workflow` | active session |
| `reload_workflow` | active session; payload `workflow_id` |
| `get_workflow_state` | active session (read-only) |

Templates are RT-local builtins only — not `fixtures/scenarios/` lineage. Capture report may include additive `workflow_summary` and `templates_applied` — no automatic `scenario_pack_ref`.

---

## 4. Blocked commands (always reject)

| Category | Examples | Error code |
|----------|----------|------------|
| Operational / weapon | `engage`, `intercept`, `strike`, `fire`, `neutralize` | `COMMAND_FORBIDDEN` |
| HITL / C2 | `mission_approve`, `operator_authorize`, `command_authority_grant` | `COMMAND_FORBIDDEN` |
| SA / federation | `corpus_promote`, `federation_register`, `publish_to_collection` | `COMMAND_FORBIDDEN` |
| Orchestration | `launch_queue`, `run_experiment_queue` | `COMMAND_FORBIDDEN` |
| SA import / corpus | `import_scenario`, `import_replay`, `auto_capture`, `save_template_to_corpus`, `promote_workflow` | `COMMAND_FORBIDDEN` |
| Parser / topics | `publish_topic`, `alter_parser_contract` | `COMMAND_FORBIDDEN` |
| Browser shortcut | Any direct ROS publish from UI | `COMMAND_FORBIDDEN` |

---

## 5. Rate limiting

Bridge **must** enforce limits from [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md):

| Limit | Value |
|-------|-------|
| Burst | 5 commands/s |
| Sustained | 1 command/s |
| Telemetry aggregate | 10 Hz |

Exceeded → `RESOURCE_LIMIT_EXCEEDED`; session may → `failed`.

---

## 6. Error codes (stub)

| Code | Meaning |
|------|---------|
| `OK` | Success |
| `COMMAND_FORBIDDEN` | Not on allow-list |
| `INVALID_STATE` | Lifecycle precondition failed |
| `INVALID_POSE` | Pose outside world bounds or malformed |
| `ENTITY_NOT_FOUND` | Unknown `entity_id` in session |
| `RESOURCE_LIMIT_EXCEEDED` | Cap exceeded |
| `WORKFLOW_STEP_FAILED` | Workflow step could not complete (PLAT-RT-S6) |
| `SESSION_NOT_FOUND` | Unknown `session_id` |
| `RUNTIME_UNAVAILABLE` | Gazebo/ROS child down |
| `BRIDGE_DISCONNECTED` | Transport loss (client-side) |

---

## 7. Audit log stub (`rt_session_audit_log_v1`)

```json
{
  "schema": "rt_session_audit_log_v1",
  "entries": [
    {
      "command_id": "uuid",
      "session_id": "uuid",
      "command_type": "spawn_entity",
      "issued_by": "rt_ui_prototype",
      "result": "OK",
      "timestamp_utc": "ISO-8601"
    }
  ]
}
```

Explanatory only — not a parser contract.

---

## 8. Prototype bridge security assumptions

Documentation constraints for RT-S2+ implementation. **Not deployed controls in RT-S1.**

| Assumption | Requirement |
|------------|-------------|
| Network exposure | Bridge listens **local-only** (127.0.0.1 or unix socket) during RT-S2–S4 |
| Public internet | No design assumption; no TLS/auth specification in RT-S1 |
| Browser credentials | Browser **never** stores ROS 2 / DDS credentials, tokens, or certificates |
| Browser → ROS | Browser **never** directly publishes or subscribes to ROS topics |
| Command policy | Bridge **deny-by-default**; only §3 allow-list permitted |
| SA viewer | `platform/sa-r0-viewer/` has **no** bridge endpoint and cannot invoke RT commands |
| Legacy `web/` rosbridge | **Not** extended for RT sandbox |
| Production security | OAuth, RBAC, mTLS, secrets store, WAF — **out of scope** until explicit security wave |

Violations of these assumptions require a new governance review — not silent extension in RT-S2.

---

## 9. Example request (conceptual)

```json
{
  "schema": "rt_bridge_request_v1",
  "command_type": "spawn_entity",
  "command_id": "cmd-001",
  "session_id": "sess-001",
  "issued_by": "rt_ui_prototype",
  "authority_scope": "rt_sandbox_prototype",
  "payload": {
    "entity_type": "drone",
    "pose": { "x": 0, "y": 0, "z": 10, "yaw_deg": 0 }
  }
}
```

---

## 10. Related

- [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md)
- [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)
- [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)
