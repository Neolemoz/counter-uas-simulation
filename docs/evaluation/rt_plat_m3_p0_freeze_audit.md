# RT-M3 P0 — Freeze Audit (PLAT-RT-M3 P0)

**Phase:** PLAT-RT-M3 P0 — session inspect + poll UX  
**Status:** frozen

**Plan:** [rt_plat_m3_p0_session_inspect_poll_plan.md](../platform/rt_plat_m3_p0_session_inspect_poll_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Maintainer CLI `list` / `summary` / `show` / `audit` | `scripts/rt/rt_session_inspect.py` |
| 2 | Per-slot `pulling`; active-only export | `useRtSessionWorkspace.ts` |
| 3 | Pull age + stale helpers | `pullAge.ts` |
| 4 | Background diagnostics: last pull age, pulling chip, stale badge | `BackgroundDiagnostics.tsx` |
| 5 | Pytest inspect CLI | `test_rt_session_inspect.py` |
| 6 | Vitest pull age + diagnostics + workspace contract | `pullAge.test.ts`, `BackgroundDiagnostics.test.tsx`, `useRtSessionWorkspace.test.ts` |
| 7 | Governance review | [rt_plat_m3_p0_governance_review_r1.md](rt_plat_m3_p0_governance_review_r1.md) |
| 8 | Registry + AGENTS | Yes |

No changes under `platform/sa-r0-viewer/`. No bridge HTTP protocol changes.

---

## Inspect workflow summary

1. Start RT bridge (`run_rt_bridge.py`).
2. `python3 scripts/rt/rt_session_inspect.py summary` — capacity, `editing_session_id`, non-terminal count.
3. `python3 scripts/rt/rt_session_inspect.py list` — registry rows.
4. `python3 scripts/rt/rt_session_inspect.py show <session_id>` — lifecycle, registry editing role, audit tail, `poll_state: ui_managed`.
5. `python3 scripts/rt/rt_session_inspect.py audit <session_id> --tail N` — read-only audit tail.
6. Correlate with workstation **Background session diagnostics** (`last pull`, `pulling`, `stale` per row).

---

## Boundary guarantees

- Read-only inspect — no session mutation  
- `max_concurrent_sessions=3`, single editing lock preserved  
- No new telemetry channels  
- No SA viewer scope  
- No distributed multi-bridge  

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| `test_rt_session_inspect.py` | pass |
| `pullAge.test.ts` + `BackgroundDiagnostics.test.tsx` + workspace contract | pass |
| `npm run build` | pass |

---

## Recommended next (advisory)

**PLAT-RT-M3 P1** per [rt_roadmap_plat_rt_m3_v1.md](rt_roadmap_plat_rt_m3_v1.md):

1. Pause background poll when diagnostics accordion collapsed  
2. Tab switch confirm for dirty local entity mirror  
3. Session display name (`localStorage`)  
4. Optional refresh-all (diagnostic channels only)

---

## Stop line

PLAT-RT-M3 P0 frozen. Do not start P1 without implementation plan + governance review + freeze audit.
