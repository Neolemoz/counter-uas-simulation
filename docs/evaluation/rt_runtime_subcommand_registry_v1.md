# RT Runtime Subcommand Registry (`rt_runtime_subcommand_registry_v1`)

**Phase:** PLAT-RT-R3c — runtime subcommand governance lint  
**Authority:** [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md); [rt_r1_architecture_stabilization_review_r1.md](rt_r1_architecture_stabilization_review_r1.md) finding R1-GOV-04

Canonical registry for `send_runtime_command` subcommands. Deny-by-default: only names in `RUNTIME_SUBCOMMANDS` are accepted.

---

## 1. Registry ownership

| Artifact | Owner | Role |
|----------|-------|------|
| `RUNTIME_SUBCOMMANDS` | `platform/rt-sandbox-bridge/rt_sandbox/governance.py` | **Allow-list authority** — validation gate |
| `RUNTIME_SUBCOMMANDS_RESERVED` | `governance.py` | Documented forbidden names — must never appear in allow-list |
| `RUNTIME_SUBCOMMAND_AUDIT_EXCEPTIONS` | `governance.py` | Subcommand → primary audit `command_type` when they differ |
| Handler branches | `session_runtime_commands.py` | Implementation — `handle_runtime_command()` |
| Governance lint | `runtime_subcommand_governance.py` | Bidirectional sync enforcement (PLAT-RT-R3c) |
| CI | `scripts/rt/lint_rt_runtime_subcommands.py --check` | `tier0` gate |

Adding or removing a subcommand requires: update `RUNTIME_SUBCOMMANDS`, implement/remove handler branch, update this document, pass governance lint, new wave audit if scope expands.

---

## 2. Active subcommands

Requires `enable_gazebo_adapter=true`, active adapter session, and adapter alive. Parent command: `send_runtime_command` with payload `{"sub_command": "<name>"}`.

| `sub_command` | Handler module | Effect | Primary audit `command_type` |
|---------------|----------------|--------|------------------------------|
| `adapter_attach` | `session_runtime_commands.py` | Start/re-attach adapter worker (idempotent) | `adapter_attach` |
| `adapter_detach` | `session_runtime_commands.py` | Terminate worker; clear adapter mirrors | `adapter_detach` |
| `adapter_health` | `session_runtime_commands.py` | Return `runtime_health` in response | `adapter_health` |
| `adapter_poll_feedback` | `session_runtime_commands.py` → `adapter_poll.py` | Poll feedback IPC; update pose mirror | `sync_update`, `sync_stale`, `sync_mismatch`, or `adapter_feedback_lost` via poll apply |
| `adapter_poll_telemetry` | `session_runtime_commands.py` → `adapter_poll.py` | Poll telemetry bundle; update `TelemetryMirror` | `telemetry_update`, `telemetry_stale`, etc. via poll apply |
| `adapter_resync` | `session_runtime_commands.py` → `template_resync.py` | Re-push all registry poses to adapter | `sync_update` (with `resync: true`) — see §4 |
| `mock_inject_drift` | `session_runtime_commands.py` → `runtime_adapter.py` | Test-only: offset sim feedback pose in mock mode | `mock_inject_drift` |

Auto-attach: `start_session` may audit `adapter_attach` when `enable_gazebo_adapter=true` without invoking `send_runtime_command`. See [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §3.3.

Manual poll subcommands share tick ownership with entity/template paths — see [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md).

---

## 3. Reserved / forbidden subcommands

Names in `RUNTIME_SUBCOMMANDS_RESERVED` are documented but **not** in the allow-list. Requests return `COMMAND_FORBIDDEN`.

| `sub_command` | Status | Alternative |
|---------------|--------|-------------|
| `reload_world_config` | Reserved | Future wave required to activate |
| `set_clock_pause` | Reserved | Use `pause_session` |

**Deprecation policy:** Removing an active subcommand requires a scoped wave: remove from `RUNTIME_SUBCOMMANDS`, remove handler branch, add to reserved set if name must stay forbidden, update docs, pass lint. Promoting a reserved name to active requires explicit wave audit.

---

## 4. Audit exceptions

When subcommand name differs from primary audit `command_type`:

| Subcommand | Primary audit `command_type` | Notes |
|------------|------------------------------|-------|
| `adapter_resync` | `sync_update` | Additive audit with `resync: true` in detail — see [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md) |

Poll subcommands (`adapter_poll_feedback`, `adapter_poll_telemetry`) classify as `adapter` for `event_kind` when used as `command_type`; poll apply paths emit `sync_*` or `telemetry_*` events per [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md).

---

## 5. Governance lint (PLAT-RT-R3c)

Automated checks (`lint_runtime_subcommands()`):

| Check | Failure |
|-------|---------|
| `RUNTIME_SUBCOMMANDS` ⊆ implemented handlers | `declared but unimplemented: {name}` |
| Handlers ⊆ `RUNTIME_SUBCOMMANDS` | `implemented but not in RUNTIME_SUBCOMMANDS: {name}` |
| Reserved ∩ allow-list = ∅ | `reserved subcommand incorrectly allowed: {name}` |
| Each allow-list member → `event_kind: adapter` | `subcommand not classified as adapter…` |
| Audit exception keys ⊆ allow-list | `audit exception for unknown subcommand…` |

**Maintainer CLI:**

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 scripts/rt/lint_rt_runtime_subcommands.py --json
```

Wired into `scripts/ci_eval.sh tier0`.

**Note:** `scripts/rt/rt_adapter_inspect.py` `_ADAPTER_CMDS` is an audit-log filter vocabulary (includes lifecycle events like `adapter_teardown`), not the runtime subcommand allow-list.

---

## 6. Related

- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)
- [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)
- [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md)
- [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md)
- [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md)
- [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md)
