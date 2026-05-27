# RT-M3 P0 — Governance Review R1 (PLAT-RT-M3 P0)

**Phase:** PLAT-RT-M3 P0 — session inspect + poll UX  
**Plan:** [rt_plat_m3_p0_session_inspect_poll_plan.md](../platform/rt_plat_m3_p0_session_inspect_poll_plan.md)  
**Poll policy:** [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md)  
**Freeze audit:** [rt_plat_m3_p0_freeze_audit.md](rt_plat_m3_p0_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| New bridge commands? | **No** — `list_sessions` only via inspect CLI |
| `RUNTIME_SUBCOMMANDS` changes? | **No** |
| SA viewer changes? | **No** |
| Parser/topic changes? | **No** |
| Distributed multi-bridge? | **No** |
| P1 features in P0? | **No** |

**Recommendation:** Freeze **PLAT-RT-M3 P0**.

---

## 1. Governance constants

| Constant | Value | P0 impact |
|----------|-------|-----------|
| `max_concurrent_sessions` | 3 | Unchanged |
| `background_telemetry_pull_cap_hz` | 1 | Per-slot throttle preserved |
| Editing lock | One `editing_session_id` | Unchanged |

| Finding ID | Verdict |
|------------|---------|
| M3P0-GOV-CONST-01 | Pass |

---

## 2. RT↔SA separation

| Check | Result |
|-------|--------|
| No SA viewer integration | **Pass** |
| Inspect CLI read-only | **Pass** |
| No auto-import / corpus writes | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| M3P0-GOV-SA-01 | Pass |

---

## 3. Poll policy compliance

| Rule | P0 implementation | Verdict |
|------|-------------------|---------|
| Per-slot `lastPullUtc` throttle | `useRtSessionWorkspace` loop | Pass |
| `pulling` scoped per slot; active refresh uses active slot only | `SessionSlot.pulling` | Pass |
| Background diagnostic channels only | Unchanged M2 | Pass |
| Background stale isolated | `BackgroundDiagnostics` row chips | Pass |

| Finding ID | Verdict |
|------------|---------|
| M3P0-GOV-POLL-01 | Pass |
| M3P0-GOV-POLL-02 | Pass — global active flicker fixed |

---

## 4. Maintainer surfaces

| Surface | Rule |
|---------|------|
| `rt_session_inspect.py` | Loopback + audit read; fails closed if bridge down |
| UI poll tuning | No bridge mutation |

| Finding ID | Verdict |
|------------|---------|
| M3P0-GOV-CLI-01 | Pass |

---

## Stop line

Do not start PLAT-RT-M3 P1 without separate freeze audit.
