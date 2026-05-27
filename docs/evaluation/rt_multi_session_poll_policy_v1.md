# RT Multi-Session Poll Policy (`rt_multi_session_poll_policy_v1`)

**Phase:** PLAN-RT-M3 — local multi-session polish (docs only)  
**Authority:** [rt_m3_local_multi_session_polish_plan.md](../platform/rt_m3_local_multi_session_polish_plan.md); extends [rt_multi_session_telemetry_routing_v1.md](rt_multi_session_telemetry_routing_v1.md)

Normative poll and stale-display rules for PLAT-RT-M3. **No implementation in PLAN-RT-M3.**

---

## 1. Session roles

| Role | Definition |
|------|------------|
| **Active** | `selectedSessionId` — full workstation binding |
| **Background** | Connected non-selected session — diagnostics only |

Only one active session at a time. Up to two background sessions when three connected.

---

## 2. Pull cadence

| Role | Max Hz | Channel set |
|------|--------|-------------|
| Active | `min(user_pull_hz, 10)` | Full allow-list per [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md) UI constants |
| Background | **1** | `session_health`, `lifecycle_state`, `world_summary` only |

Background sessions **must not** subscribe to full-rate `entity_pose_mirror` or `clock_mirror`.

---

## 3. Scheduler rules (PLAT target)

| Rule | Rationale |
|------|-----------|
| Per-slot `lastPullUtc` throttle | Avoid synchronized pull bursts |
| Active interval derived from user `pullHz` | Preserves T1 refresh control |
| Background interval ≥ 1000 ms | Governance `background_telemetry_pull_cap_hz` |
| Optional: skip background pull when diagnostics accordion collapsed | Reduce load when user not viewing |
| `pulling` UI flag scoped per slot or per active-only | Avoid active workspace flicker on background pull |

Current M2 implementation uses a single global interval with per-slot throttle — PLAT may refine to per-slot timers.

---

## 4. Stale and fault display

| Condition | Active session | Background session |
|-----------|----------------|-------------------|
| Pull fault (`lastError`) | Workflow strip + panels | Row fault indicator |
| Telemetry stale | Full cognition hub | Compact chip in `BackgroundDiagnostics` |
| Adapter not alive | Sync strip + health badges | `session_health` summary in row |
| Background stale in session A | — | **Does not** block editing in active session B |

Use `sessionRole: "active" | "background"` in cognition helpers where applicable.

---

## 5. Manual refresh

| Control | Scope |
|---------|-------|
| User “Pull now” / refresh | Active session only (M2 default) — optional PLAT “refresh all connected” |
| Maintainer `rt_session_inspect` | Registry + audit — not a substitute for UI pull |

---

## 6. Forbidden

- WebSocket push or server-initiated browser streams  
- Cross-session merged pull buffers  
- Background sessions at active pull rate  
- New telemetry channels without contract wave  

---

## Related

- [rt_multi_session_governance_v1.md](rt_multi_session_governance_v1.md)
- [rt_m3_local_multi_session_ux_review_r1.md](rt_m3_local_multi_session_ux_review_r1.md)
- [rt_roadmap_plat_rt_m3_v1.md](rt_roadmap_plat_rt_m3_v1.md)
