# RT Template Resync Policy (`rt_template_resync_policy_v1`)

**Phase:** PLAT-RT-R2d — template adapter resync  
**Authority:** [rt_r2d_template_adapter_resync_plan.md](../platform/rt_r2d_template_adapter_resync_plan.md)

Defines when bridge template/workflow mutations trigger adapter resync and how workflow continuity is audited.

---

## 1. Authority

| Layer | Role |
|-------|------|
| Bridge registry + template apply | **Command-authoritative** template intent |
| `resync_all` / `reset_world` IPC | Transient push or clear of adapter/sim state |
| Pose/telemetry mirrors | **Explanatory** post-resync diagnostics |
| Template command outcome | Registry mutation succeeds even if resync stale (non-blocking) |

See [rt_authority_model_v1.md](rt_authority_model_v1.md).

---

## 2. Triggers

| Trigger | Adapter action | Skip when |
|---------|----------------|-----------|
| `apply_runtime_template` | `resync_all` → poll feedback+telemetry | Adapter inactive; `entities_spawned == 0` |
| `advance_workflow` → `apply_template` | Same | Same |
| `advance_workflow` → `reset_world` | `sync_reset_world` + `clear_pose_sync` → poll telemetry | Adapter inactive |
| `adapter_resync` subcommand | Same as template resync (manual) | Adapter inactive |
| `reset_workflow` / `reload_workflow` | **No resync** | Workflow pointer only |

Implementation: [template_resync.py](../../platform/rt-sandbox-bridge/rt_sandbox/template_resync.py) `run_template_adapter_resync()`.

---

## 3. Audit events

| `command_type` | When |
|----------------|------|
| `template_resync_requested` | Policy decides to attempt resync |
| `template_resync_completed` | Resync + poll OK |
| `template_resync_skipped` | Adapter inactive, no entities, or world not initialized |
| `template_resync_stale` | Post-resync `SYNC_STALE`, `SYNC_MISMATCH`, or IPC failure (non-blocking for template commands) |

All map to `event_kind: sync`. Manual `adapter_resync` retains additive `sync_update` with `resync: true`.

---

## 4. Stale semantics

| Term | Meaning |
|------|---------|
| **Stale adapter** | Post-resync poll reports drift/IPC loss → `template_resync_stale` |
| **Stale template** | Not a failure mode — registry intent applied; resync aligns adapter |
| **Mismatch recovery** | Automatic resync after template apply; maintainer `adapter_resync` for explicit recovery |

Template apply does **not** fail on `template_resync_stale` (contrast: per-entity spawn/move may fail on `SYNC_STALE`).

---

## 5. Related paths

- Entity spawn/move: per-entity `run_post_entity_sync` (PLAT-RT-G3)
- Session reset: `sync_reset_world` + mirror clear (full session)
- Poll unification: [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md)

---

## Related

- [rt_workflow_contract_v1.md](rt_workflow_contract_v1.md)
- [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)
