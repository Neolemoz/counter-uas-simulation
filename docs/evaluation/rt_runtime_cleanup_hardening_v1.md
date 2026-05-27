# RT Runtime Cleanup Hardening (`rt_runtime_cleanup_hardening_v1`)

**Phase:** PLAN-RT-F2 / PLAT-RT-F2  
**Authority:** [session_teardown.py](../../platform/rt-sandbox-bridge/rt_sandbox/session_teardown.py)

## Teardown sequence (required order per path)

1. `entity_cleanup` (when world present)
2. `telemetry_cleanup` (when subscriptions exist)
3. Adapter mirror clear (`clear_adapter_mirrors`)
4. **`tactical_cleanup`** — reset `TacticalCaptureBuffer`, set `session.tactical = None`
5. Runtime terminate / orphan audits as today

## Audit: `tactical_cleanup`

| Field | Meaning |
|-------|---------|
| `trigger` | e.g. `discard_session`, `capture_session`, `auto_cleanup` |
| `had_tactical` | boolean |
| `buffer_was_empty` | boolean when tactical existed |
| `timeline_counts` | optional snapshot of list lengths before reset |

Emitted only when `session.tactical` was a `TacticalController`.

## Terminal registry evict

Unchanged: `_evict_terminal_sessions` after poll tick removes DISCARDED/CAPTURED records.

## Related

- [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md)
