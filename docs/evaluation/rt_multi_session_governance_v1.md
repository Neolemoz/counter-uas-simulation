# RT Multi-Session Governance Supplement (`rt_multi_session_governance_v1`)

**Phase:** PLAN-RT-M1 — multi-session architecture (docs only)  
**Authority:** [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) (additive supplement — does not replace frozen §3)

Governance extensions for local single-bridge multi-session RT sandbox. **No implementation in PLAN-RT-M1.**

---

## 1. Relationship to rt_runtime_governance_v1

This document **supplements** [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md). Frozen per-session caps remain unchanged. Only multi-session-specific constants and rules are defined here.

Changing any constant requires a new scoped wave and freeze audit row.

---

## 2. Revised governance constants (M1)

| Constant | Single-session (frozen) | Multi-session (M1 default) | Notes |
|----------|-------------------------|----------------------------|-------|
| `max_concurrent_sessions` | 1 | **3** | Per bridge instance |
| `max_entity_count` | 32 | 32 | Per session — unchanged |
| `max_session_duration` | 3600 s | 3600 s | Per session — unchanged |
| `telemetry_update_rate_cap_hz` | 10 Hz | 10 Hz | Per active session |
| `background_telemetry_pull_cap_hz` | N/A | **1 Hz** | Per background session |
| `command_rate_limit_burst` | 5/s | 5/s | Per session (M2) |
| `command_rate_limit_sustained` | 1/s | 1/s | Per session (M2) |
| `max_staged_captures` | 32 | 32 | Global — unchanged |
| `max_total_entities_across_sessions` | N/A | **64** | Optional aggregate ceiling |

---

## 3. Required banners (additive)

Existing five banners unchanged. Add when ≥ 2 sessions connected:

`MULTI-SESSION — local prototype; not operational coordination`

Secondary (inline in session rail):

`Each session is transient — not replay authority`

---

## 4. No cross-session leakage

| Surface | Rule |
|---------|------|
| Audit logs | Per-session file via `audit_log.path_for(session_id)` |
| Telemetry drain | Subscription scoped to `session_id` |
| World registry | Per `SessionRecord.world` |
| Capture snapshots | Per `capture_candidate_id` from target session only |
| Pose/telemetry mirrors | Per session — never shared references |
| UI state | Per-session snapshot/local mirror maps |

---

## 5. Cleanup guarantees

| Scenario | Behavior |
|----------|----------|
| Session A `failed` | Teardown A only; B and C continue |
| Session A `runtime_crashed` | A → auto-cleanup; siblings unaffected |
| Session A timeout | Per-session `tick_timeouts`; no global bridge reset |
| Bridge process exit | All sessions lost — no persistence (unchanged transient model) |
| Editing session evicted | Reassign lock to next non-terminal session or clear |

Teardown paths unchanged — see [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) §3.

---

## 6. Resource limits

| Limit type | Enforcement |
|------------|-------------|
| Session capacity | `SESSION_CAPACITY_EXCEEDED` when registry full |
| Per-session entity cap | `RESOURCE_LIMIT_EXCEEDED` on target session |
| Aggregate entity cap | `RESOURCE_LIMIT_EXCEEDED` when total entities ≥ 64 |
| Per-session command rate | `RESOURCE_LIMIT_EXCEEDED` on target session |
| Global staged captures | `RESOURCE_LIMIT_EXCEEDED` on next capture |

Caps are **not** auto-tuned by load or user role.

---

## 7. Authority boundaries (unchanged)

| Surface | May start RT session? | May capture? | May mutate corpus/federation? |
|---------|----------------------|--------------|------------------------------|
| RT Browser UI | Request via bridge only | Request only (cognition in T4) | **No** |
| RT Backend Bridge | Yes (local) | Emit capture candidate | **No** |
| `platform/sa-r0-viewer/` | **No** | **No** | **No** |
| Federation CLIs | **No** | **No** | **No** RT session input |

---

## 8. Forbidden architectural directions

Extends [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) §3.2:

- Horizontal scaling, session pooling, load balancing
- Multi-user collaboration rooms
- Cloud orchestration or remote bridge clusters
- Multi-bridge session federation
- Operational SLA / uptime semantics
- Session persistence across bridge restart

**Authorized by M1 (local only):** up to 3 concurrent sessions on **one** bridge process on loopback.

---

## 9. Concurrency lock model

| Lock | Owner | Scope |
|------|-------|-------|
| Registry mutex | Bridge facade | All registry mutations |
| `editing_session_id` | Bridge | Entity mutation gate |
| Per-session rate limiter | Bridge | Command throttle per session |
| UI tab selection | Browser | Display only — not authority |

---

## Related

- [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md)
- [rt_multi_session_editing_ownership_v1.md](rt_multi_session_editing_ownership_v1.md)
- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §11 (M1 additive)
