# RT-M2 — Multi-Session Isolation Audit

**Phase:** PLAT-RT-M2  
**Prerequisite:** PLAN-RT-M1 frozen

---

## Isolation surface checklist

| Surface | Verification | Result |
|---------|--------------|--------|
| World registry | Per `SessionRecord.world`; `test_cross_session_world_isolation` | **Pass** |
| Adapter mirrors | `clear_adapter_mirrors` per session teardown | **Pass** |
| Telemetry drain | `subscription.session_id` match on pull | **Pass** |
| Audit paths | `audit_log.path_for(session_id)` per record | **Pass** |
| Capture staging | Target session world only; `test_capture_does_not_read_sibling_world` | **Pass** |
| Editing lock | `EDITING_SESSION_MISMATCH`; `test_set_editing_session_and_mismatch` | **Pass** |
| Sibling failure | No auto-discard of other sessions; `test_failed_session_sibling_survives` | **Pass** |
| UI state maps | Per-session `editBySession` in `App.tsx`; workspace slot maps | **Pass** |
| Registry capacity | Fourth start → `SESSION_CAPACITY_EXCEEDED` | **Pass** |
| Aggregate entity cap | Sum across non-terminal sessions on spawn | **Pass** |
| Per-session telemetry rate | `_rate_ok(session_id)` in `telemetry_subscriptions.py` | **Pass** |
| Terminal eviction | `discarded`/`captured` removed from registry | **Pass** |

---

## Thread safety

`ThreadingHTTPServer` serves concurrent requests; registry mutations and `editing_session_id` updates occur under `SessionRegistry` lock (see [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) §11).

---

## Verdict

**All isolation surfaces pass** for PLAT-RT-M2 local single-bridge multi-session support.
