# RT-F6 P0 — Freeze Audit (PLAT-RT-F6 P0)

**Phase:** PLAT-RT-F6 P0 — advisory readiness mirror  
**Status:** frozen

**Plan:** [rt_plat_f6_p0_readiness_mirror_implementation_plan.md](../platform/rt_plat_f6_p0_readiness_mirror_implementation_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Python derive | `advisory_derive.py` |
| 2 | TypeScript derive | `deriveAdvisoryState.ts` + helpers |
| 3 | Maintainer CLI | `rt_handoff_advisory_status.py` |
| 4 | Golden fixtures + expected | `fixtures/rt_handoff/f6_advisory_examples/expected/` |
| 5 | Vitest golden tests | `deriveAdvisoryState.test.ts` |
| 6 | Pytest golden + integration | `test_rt_handoff_advisory.py` |
| 7 | Handoff panel advisory column + strip | `CaptureHandoffWorkflowPanel.tsx` |
| 8 | Workbench advisory strip | `ExperimentWorkbenchPanel.tsx` |
| 9 | Governance review | [rt_plat_f6_p0_governance_review_r1.md](rt_plat_f6_p0_governance_review_r1.md) |
| 10 | Registry + AGENTS | Yes |

No changes under `platform/sa-r0-viewer/`. No bridge HTTP protocol changes.

---

## Readiness mirror summary

**Advisory ladder:** `capture_ready` → `review_complete` → `approval_ready` → `handoff_ready` → `import_ready` (terminal at `handoff_import_committed`).

**Derive inputs:** `handoff_status_summary` + staging signals (CLI); `CaptureHandoffRow` mapper (UI).

**Blocked:** reject/defer only — precondition errors cap ladder height without `blocked` advisory state.

---

## Boundary guarantees

- Read-only derive — no staging or corpus writes  
- `capture_session ≠ SA import` preserved  
- No bridge subcommand registry changes  
- No SA viewer scope  
- Export event `handoff_ready` ≠ advisory `handoff_ready` — tested in golden fixtures  

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| `test_rt_handoff_advisory.py` | 14 passed |
| `deriveAdvisoryState.test.ts` | 13 passed |
| `npm run build` | pass |

---

## Recommended next (advisory)

**PLAT-RT-F6 P1** — `SaWorkflowAdvisoryPanel`, checklist chips, `BANNER_SA_WORKFLOW_ADVISORY`, import advisory strip per [rt_sa_workflow_advisory_ui_v1.md](rt_sa_workflow_advisory_ui_v1.md).

---

## Stop line

PLAT-RT-F6 P0 frozen. Do not start P1 without implementation plan + governance review + freeze audit.
