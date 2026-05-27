# RT Multi-Session Roadmap (`rt_roadmap_m1_m2_v1`)

**Phase:** PLAN-RT-M1 — multi-session architecture roadmap (docs only)  
**Authority:** [rt_m1_multi_session_architecture_plan.md](../platform/rt_m1_multi_session_architecture_plan.md); [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md)

Roadmap for local single-bridge multi-session RT sandbox. **No wave listed here may start without** its own plan, governance review, and freeze audit.

---

## Cross-wave dependencies

```mermaid
flowchart LR
  M1[PLAN-RT-M1_docs]
  M2[PLAT-RT-M2_registry_UI]
  M3[PLAT-RT-M3_optional_polish]
  M1 --> M2
  M2 --> M3
```

---

## PLAN-RT-M1 — Multi-Session Architecture Plan

| Allowed | Forbidden |
|---------|-----------|
| Session registry architecture docs | Bridge/UI implementation |
| Telemetry routing contract | Distributed runtime |
| Editing ownership rules | SA viewer changes |
| Capture/handoff isolation docs | Automatic replay ingestion |
| Governance supplement (`max_concurrent_sessions=3`) | Federation integration |
| UX planning (tabs, workspace, diagnostics) | Cloud / multi-user infra |
| Governance + architecture reviews | Parser/topic changes |

**Stop line:** End after freeze audit. Do not start PLAT-RT-M2.

---

## PLAT-RT-M2 — Multi-Session Bridge + UI (blocked)

**Prerequisite:** PLAN-RT-M1 frozen

| Allowed | Forbidden |
|---------|-----------|
| `SessionRegistry` replacing `_session` slot | Multi-bridge orchestration |
| Wire `max_concurrent_sessions=3` | SA viewer integration |
| `list_sessions`, `set_editing_session` commands | Automatic SA import |
| Per-session rate limiters + `tick_timeouts` | Federation writes |
| Bridge-enforced editing lock | Cloud infra |
| UI session tabs + multi-session hook | Tactical/HITL semantics |
| Per-session snapshot/local mirror maps | Live-default adapter change |
| Background diagnostics panel | Parser/topic changes |
| Integration tests (3 sessions, capacity, editing lock, capture isolation) | |

**Stop line:** No PLAT-RT-M3 without M2 freeze audit.

---

## PLAT-RT-M3 — Optional polish (provisional)

**Prerequisite:** PLAT-RT-M2 frozen; **PLAN-RT-M3 frozen** — see [rt_m3_freeze_audit.md](rt_m3_freeze_audit.md), [rt_roadmap_plat_rt_m3_v1.md](rt_roadmap_plat_rt_m3_v1.md)

| Theme | Notes |
|-------|-------|
| Background pull optimization | Adaptive poll scheduling for background sessions |
| Maintainer multi-session inspect CLI | `rt_session_inspect.py list` / health summary |
| Session rail UX polish | Tab reorder, session naming |

Out of M1/M2 scope unless explicitly authorized. **PLAT-RT-M3 not authorized** until implementation freeze audit.

---

## Forbidden (unchanged from post-G5)

| Theme | Rationale |
|-------|-----------|
| Distributed multi-bridge | Conflicts with local prototype |
| Automatic SA replay ingestion | G5 + R1 stop line |
| Federation / corpus writes from RT | `rt_sa_export_boundary_v1` |
| SA viewer live session hooks | PLAN-RT-S1 boundary |
| Autonomous runtime behavior | Not in RT frontier |
| Production security / cloud infra | Prototype local-only |

**Authorized by PLAN-RT-M1:** up to **3 concurrent sessions** on **one** loopback bridge process.

---

## Stop line

PLAN-RT-M1 defines architecture only. PLAT-RT-M2 is **not authorized** until M1 freeze audit completes. PLAT-RT-M3 requires M2 freeze.

---

## Related

- [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md)
- [rt_multi_session_governance_v1.md](rt_multi_session_governance_v1.md)
- [rt_m1_freeze_audit.md](rt_m1_freeze_audit.md)
