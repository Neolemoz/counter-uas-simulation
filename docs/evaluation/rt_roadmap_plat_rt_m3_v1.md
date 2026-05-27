# RT PLAT-RT-M3 Roadmap (`rt_roadmap_plat_rt_m3_v1`)

**Phase:** PLAN-RT-M3 — PLAT implementation roadmap (docs only; **not authorized** until PLAT freeze)  
**Prerequisite:** PLAN-RT-M3 frozen  
**Plan:** [rt_m3_local_multi_session_polish_plan.md](../platform/rt_m3_local_multi_session_polish_plan.md)  
**UX review:** [rt_m3_local_multi_session_ux_review_r1.md](rt_m3_local_multi_session_ux_review_r1.md)  
**Poll policy:** [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md)

No implementation wave may start without governance review, freeze audit, and regression per scope below.

---

## 1. Purpose

Implement **local multi-session polish** on frozen PLAT-RT-M2: maintainer session inspection, polling/stale UX refinements, and optional tab/workspace ergonomics — **single bridge, cap=3**.

---

## 2. PLAT-RT-M3 P0 (required)

| Deliverable | Layer | Notes |
|-------------|-------|-------|
| `scripts/rt/rt_session_inspect.py` | Maintainer CLI | `list`, `show`, `summary`, `audit`; loopback `list_sessions` |
| Poll policy compliance | UI | Per [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md) |
| Per-slot pull UX | UI | Active-only `pulling` flag; background row `lastPullUtc` age |
| Tests | CLI + UI | Inspect smoke; extend `useRtSessionWorkspace.test.ts` |
| Docs | Evaluation | `rt_plat_m3_freeze_audit.md`, optional `rt_plat_m3_governance_review_r1.md` |

### P0 allowed

- `platform/rt-sandbox-ui/` hook and diagnostics tweaks  
- `scripts/rt/rt_session_inspect.py`  
- Tests under `src/counter_uas/test/` and `platform/rt-sandbox-ui/`  
- No new bridge HTTP commands  

### P0 forbidden

- New telemetry channels  
- `RUNTIME_SUBCOMMANDS` changes  
- SA viewer changes  
- Distributed / multi-bridge  

---

## 3. PLAT-RT-M3 P1 (optional, same wave audit)

| Deliverable | Notes |
|-------------|-------|
| Pause background poll when diagnostics accordion collapsed | Power/UX |
| Tab switch confirm for dirty local entity mirror | M1 gap closure |
| Session display name | `localStorage` per `sessionId`; RT-only |
| “Refresh all connected” | Optional; diagnostic channels only for background |

---

## 4. PLAT-RT-M3 P2 (defer or bundle)

| Deliverable | Notes |
|-------------|-------|
| Tab reorder persistence | localStorage order key |
| Richer background row fields | From existing channels only |

---

## 5. Regression

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
python3 scripts/rt/rt_session_inspect.py summary   # after implementation
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## 6. Explicit non-goals

| Theme | Rationale |
|-------|-----------|
| Distributed multi-bridge | AGENTS.md forbidden |
| PLAN-RT-F7 advisory | Separate wave |
| M4 / cloud orchestration | Out of RT frontier |
| Background Cesium globe | Workstation contract — active globe only |
| Cross-session entity drag | Authority / isolation risk |

---

## 7. Stop line

**PLAN-RT-M3** ends at docs freeze. **PLAT-RT-M3** is blocked until:

1. Implementation plan row in freeze registry  
2. Governance review + freeze audit  
3. Regression green for wave scope  

Do not merge PLAT-RT-M3 with F7, distributed runtime, or bridge protocol changes.

---

## Related

- [rt_roadmap_m1_m2_v1.md](rt_roadmap_m1_m2_v1.md)
- [rt_m2_freeze_audit.md](rt_m2_freeze_audit.md)
- [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md)
