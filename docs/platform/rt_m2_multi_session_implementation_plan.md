# RT-M2 — Multi-Session Runtime UI + Bridge (PLAT-RT-M2)

**Phase:** PLAT-RT-M2 — multi-session bridge + workstation UI  
**Prerequisite:** PLAN-RT-M1 frozen  
**Authority:** [AGENTS.md](../../AGENTS.md); M1 contracts in `docs/evaluation/rt_multi_session_*_v1.md`

**Companion artifacts:**

- [rt_m2_governance_review_r1.md](../evaluation/rt_m2_governance_review_r1.md)
- [rt_m2_isolation_audit.md](../evaluation/rt_m2_isolation_audit.md)
- [rt_m2_freeze_audit.md](../evaluation/rt_m2_freeze_audit.md)

---

## 1. Purpose

Implement **local single-bridge multi-session** support (cap=3) per frozen PLAN-RT-M1:

- Bridge `SessionRegistry` replacing sole `_session` slot
- `list_sessions` / `set_editing_session` commands
- Bridge-enforced editing lock and aggregate entity cap
- Per-session telemetry emit rate limits
- RT UI session tabs, background diagnostics, per-session edit state maps

**Stop line:** no PLAT-RT-M3, RT-SA2, RT-V1, distributed multi-bridge, or SA auto-import.

---

## 2. Deliverables

| Layer | Artifact |
|-------|----------|
| Bridge | `session_registry.py`, `session_registry_handlers.py`, manager refactor |
| Governance | `max_concurrent_sessions=3`, `REGISTRY_COMMANDS`, editing gate |
| UI | `useRtSessionWorkspace.ts`, `SessionTabBar`, `BackgroundDiagnostics`, `App.tsx` maps |
| Tests | Bridge pytest (capacity, isolation, editing lock, capture, sibling survival) |
| UI tests | `SessionTabBar.test.tsx`, `useRtSessionWorkspace.test.ts`, governance banner |
| Docs | M2 reviews, isolation audit, freeze audit, registry update |

---

## 3. Architecture summary

Single bridge process holds up to three non-terminal `SessionRecord` entries. The UI selects one **active** workspace (full telemetry pull, editing surfaces) and one **editing** session (bridge lock). Background slots pull diagnostic channels at 1 Hz only.

---

## 4. Allowed / forbidden

**Allowed:** `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, bridge/UI tests, M2 docs, additive governance constants, registry commands.

**Forbidden:** SA viewer, parser/topic changes, distributed runtime, multi-bridge, SA auto-import, federation writes, new telemetry channels, WebSocket push, cloud infra, tactical/HITL UX.

---

## 5. Regression commands

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
```
