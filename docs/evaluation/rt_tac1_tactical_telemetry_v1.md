# RT Tactical Telemetry Plan (`rt_tac1_tactical_telemetry_v1`)

**Phase:** PLAN-RT-TAC1 — future telemetry surfaces (docs only)  
**Authority:** [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md); [rt_authority_model_v1.md](rt_authority_model_v1.md); [rt_tac1_tactical_modes_v1.md](rt_tac1_tactical_modes_v1.md)

Defines **planned** tactical telemetry fields and channels for PLAT-RT-TAC2+. **No bridge subscription, handler, or UI changes in PLAN-RT-TAC1.**

---

## 1. Design principles

1. Every payload includes `governance_banner` (string, required).
2. Every payload includes `authority_label` from [rt_authority_model_v1.md](rt_authority_model_v1.md) §5.
3. Every payload includes `source: rt_tactical_controller` or `source: rt_bridge_session`.
4. Tactical telemetry is **RT-session scoped** — keyed by `session_id`.
5. No new ROS topic subscriptions for tactical data in bridge allow-list until explicit wave.
6. Pull model aligns with existing `subscribe_telemetry` / HTTP pull — no WebSocket.

---

## 2. Channel: `tactical_state`

**Delivery (future):** New telemetry channel `tactical_state` on active session pull (≤ 10 Hz active, ≤ 1 Hz background).

**Subscription:** Requires PLAT-RT-TAC2+ bridge allow-list extension.

### 2.1 Payload schema (sketch)

```json
{
  "schema": "rt_tactical_state_v1",
  "session_id": "uuid-ephemeral",
  "tactical_mode": "manual",
  "selected_candidate_id": "entity-uuid-or-null",
  "assigned_candidate_id": "entity-uuid-or-null",
  "tti_s": null,
  "tactical_health": {
    "feasible": false,
    "summary": "no_intercept_solution_in_window",
    "stale": false
  },
  "assignment_lock_active": false,
  "pending_recommendation_id": null,
  "authority_label": "command_authoritative",
  "source": "rt_bridge_session",
  "governance_banner": "TACTICAL TELEMETRY — sandbox simulation; not operational state",
  "sampled_at_utc": "ISO-8601"
}
```

### 2.2 Field semantics

| Field | Meaning | Authority label (typical) |
|-------|---------|---------------------------|
| `tactical_mode` | `manual` \| `assisted` \| `autonomous` | `command_authoritative` on session record |
| `selected_candidate_id` | Highlight for review | Manual: user; Assisted: user or explanatory; Autonomous: `tactical_controller_authoritative` |
| `assigned_candidate_id` | Committed sandbox assignment | Manual: user; Assisted: `user_approval_authoritative`; Autonomous: `tactical_controller_authoritative` |
| `tti_s` | Time-to-intercept at interceptor speed cap | `explanatory_telemetry` |
| `tactical_health` | Feasibility summary + stale flag | `explanatory_telemetry` |
| `assignment_lock_active` | Post-commit lock window | `explanatory_telemetry` |
| `pending_recommendation_id` | Assisted mode only | `tactical_recommendation_explanatory` |

---

## 3. Channel: `tactical_recommendation` (Assisted — TAC3)

**Delivery:** Event on pull after `request_recommendation` or controller tick.

```json
{
  "schema": "rt_tactical_recommendation_v1",
  "recommendation_id": "uuid",
  "session_id": "uuid-ephemeral",
  "candidate_id": "entity-uuid",
  "tti_s": 12.4,
  "feasibility": { "feasible": true, "reason": "feasible" },
  "expires_at_utc": "ISO-8601",
  "authority_label": "tactical_recommendation_explanatory",
  "governance_banner": "RECOMMENDATION — requires user approval; not assignment authority"
}
```

Bridge **must not** apply assignment until `approve_recommendation` with matching `recommendation_id`.

---

## 4. Mapping to engine observability

| RT field | Engine reference (optional read-only input) |
|----------|---------------------------------------------|
| `tti_s` | `t_intercept_at_cap` from feasibility triple |
| `tactical_health.summary` | `FeasibilityDecision.reason` |
| `selected_candidate_id` | May mirror `/interceptor/selected_id` when adapter exposes — **not** authoritative if diverged |
| `assigned_candidate_id` | May mirror `assigned_target` — RT mode rules win |

Divergence between RT mirror and engine topic **must** surface as explanatory mismatch in capture cognition (TAC5) — never silent override.

---

## 5. UI cognition (future)

| Surface | Behavior |
|---------|----------|
| Workstation mirror column | `tactical_state` snapshot + banner |
| Cesium marker accent | Selected candidate outline (V1 fidelity patterns) |
| Assisted panel | Recommendation card + approve/reject |
| Autonomous panel | Loop status + pause → Manual |

Copy per [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md) §5.

---

## 6. Forbidden telemetry

| Channel / topic | Reason |
|-----------------|--------|
| `/tracks/state` | Parser contract — bridge forbidden |
| Weapon / engage topics | Operational boundary |
| Federation status | SA platform |
| Raw `[TACTICAL_*]` log stream as subscription | Use structured RT mirrors instead |

---

## 7. Out of scope

- `platform/rt-sandbox-bridge/telemetry_subscriptions.py` changes
- `platform/rt-sandbox-ui/` components
- SA viewer overlays

---

## Related

- [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md)
- [rt_multi_session_telemetry_routing_v1.md](rt_multi_session_telemetry_routing_v1.md)
- [rt_telemetry_ui_v1.md](rt_telemetry_ui_v1.md)
