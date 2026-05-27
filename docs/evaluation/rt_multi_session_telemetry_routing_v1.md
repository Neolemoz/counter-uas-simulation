# RT Multi-Session Telemetry Routing (`rt_multi_session_telemetry_routing_v1`)

**Phase:** PLAN-RT-M1 — multi-session architecture (docs only)  
**Authority:** [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §3.4; [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)

Per-session telemetry pull routing for local multi-session RT sandbox. **No implementation in PLAN-RT-M1.**

---

## 1. Core rules

| Rule | Value |
|------|-------|
| Subscriptions | Max **1** active subscription per session (unchanged) |
| Pull transport | `GET /v1/telemetry/pull?session_id=&subscription_id=&max_events=` (unchanged) |
| Drain semantics | Destructive per subscription — no cross-session merge |
| Mirror authority | Explanatory only — mirrors ≠ authority |

Each session owns independent `TelemetrySubscriptionStore` entries keyed by `session_id`.

---

## 2. UI session roles

| Role | Pull rate cap | Channel set |
|------|---------------|-------------|
| **Active session** | ≤ 10 Hz aggregate | Full allow-list: `session_health`, `lifecycle_state`, `world_summary`, `entity_pose_mirror`, `clock_mirror` |
| **Background session** | ≤ 1 Hz aggregate | Diagnostic subset: `session_health`, `lifecycle_state`, `world_summary` (entity count + revision only) |

Background sessions **must not** subscribe to `entity_pose_mirror` or `clock_mirror` at full rate — reduces adapter poll load and prevents mirror confusion in UI.

---

## 3. UI session switching

| Action | Behavior |
|--------|----------|
| Tab select (promote to active) | Continue existing subscription; increase pull rate to active cap; optionally resubscribe for full channel set |
| Tab deselect (demote to background) | Reduce pull rate; optionally resubscribe to diagnostic channel subset |
| Tab disconnect | `unsubscribe_telemetry` + `stop_session` or `discard_session` for that session only |
| Bridge disconnect (global) | Best-effort cleanup for all connected sessions |

Switching tabs **does not** unsubscribe background sessions unless user explicitly disconnects that session.

---

## 4. Mirror isolation

| Concern | Isolation rule |
|---------|----------------|
| `SessionRecord.pose_sync` | Per-session; never shared |
| `SessionRecord.telemetry_mirror` | Per-session; never shared |
| UI snapshot state | Keyed by `sessionId` — separate maps per session |
| Stale badges | Per-session; background stale does not affect active workspace banners |
| Channel merge in UI | `mergeChannelSnapshots` scoped to one session's pull results only |

---

## 5. Stale handling

| Condition | Active session UI | Background diagnostics UI |
|-----------|-------------------|----------------------------|
| Telemetry poll stale | Full stale highlight in cognition hub + panels | Compact stale chip in accordion row |
| Adapter feedback lost | Sync cognition strip warning | `session_health.adapter_alive = false` in row |
| Pull fault (`lastError`) | Workflow strip fault badge | Row-level fault indicator |

Stale in background session A does **not** suppress editing in active session B.

---

## 6. Rate limiting (revised for M2)

| Limiter scope | Current (frozen) | M1 (future M2) |
|---------------|------------------|----------------|
| Command rate | Global on manager | Per-session |
| Telemetry emit | Global `_rate_ok` on store | Per-session emit gate |
| Pull rate | Client-side interval | Client enforces active vs background caps |

Exceeded per-session telemetry cap → `RESOURCE_LIMIT_EXCEEDED` for that session only.

---

## 7. Adapter poll scheduling (M2 guidance)

When Gazebo adapter enabled with multiple sessions:

- Active session: full post-mutation poll + lifecycle poll
- Background sessions: reduced poll on diagnostic pull tick only
- Poll failures scoped to target session — no bridge-wide adapter detach

See [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md).

---

## 8. Explicit non-goals

- WebSocket push / streaming infra
- Cross-session telemetry aggregation or comparison views
- SA viewer live telemetry hooks
- Shared telemetry ring buffer across sessions

---

## Related

- [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md)
- [rt_telemetry_ui_v1.md](rt_telemetry_ui_v1.md)
- [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)
