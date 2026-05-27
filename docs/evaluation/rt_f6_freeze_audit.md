# RT-F6 — Freeze Audit (PLAN-RT-F6)

**Phase:** PLAN-RT-F6 — SA workflow automation advisory  
**Status:** frozen (docs only)

**Plan:** [rt_f6_sa_workflow_automation_advisory_plan.md](../platform/rt_f6_sa_workflow_automation_advisory_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Workflow automation contract | [rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md) |
| 2 | Advisory UI contract | [rt_sa_workflow_advisory_ui_v1.md](rt_sa_workflow_advisory_ui_v1.md) |
| 3 | Reference fixtures | [fixtures/rt_handoff/f6_advisory_examples/](../../fixtures/rt_handoff/f6_advisory_examples/) |
| 4 | Architecture review | [rt_f6_architecture_review_r1.md](rt_f6_architecture_review_r1.md) |
| 5 | Governance review | [rt_f6_governance_review_r1.md](rt_f6_governance_review_r1.md) |
| 6 | Handoff contamination review | [rt_f6_handoff_contamination_review_r1.md](rt_f6_handoff_contamination_review_r1.md) |
| 7 | PLAT roadmap | [rt_roadmap_plat_rt_f6_v1.md](rt_roadmap_plat_rt_f6_v1.md) |
| 8 | Next frontiers update | [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) |
| 9 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` for this wave.

---

## Workflow automation architecture summary

**Advisory ladder:** `capture_ready` → `review_complete` → `approval_ready` → `handoff_ready` → `import_ready` (terminal SA authority at `handoff_import_committed` only).

**Composition:** Derives from frozen SA1 export audit + SA2 mirror + F5 experiment eligibility (warn-only). Does not replace `rt_handoff_review.py`, `rt_capture_approve.py`, or `rt_sa_import commit`.

**Automation tiers (PLAT advisory):**

- **P0:** Readiness mirror — derive + read-only CLI  
- **P1:** Advisory checklist UI — workbench, staging column, import strip  
- **P2:** Optional maintainer helpers — dry-run default; no auto-commit  

**Naming guard:** Export event `handoff_ready` (pre-approval) ≠ advisory `handoff_ready` (post-approve packaging). UI uses "packaging ready (advisory)" for the latter.

---

## Boundary guarantees

- Advisory states are **not** SA replay authority  
- `capture_session ≠ SA import` preserved  
- Corpus authority begins only at explicit maintainer `commit --corpus-dest`  
- No parser/topic/bridge changes in PLAN wave  
- No SA viewer or auto-import scope  
- No federation writes from RT sessions  
- PLAN-RT-F6 ≠ PLAT-RT-SA1/SA2/SA3 replacements  
- P2 batch helpers require contamination re-check before PLAT authorization  

---

## Recommended PLAT-RT-F6 scope (advisory)

See [rt_roadmap_plat_rt_f6_v1.md](rt_roadmap_plat_rt_f6_v1.md):

- **P0 (recommended next):** `deriveAdvisoryState`, `rt_handoff_advisory_status.py`, golden fixtures  
- **P1:** `SaWorkflowAdvisoryPanel`, workbench badges, `BANNER_SA_WORKFLOW_ADVISORY`  
- **P2:** Batch advisory report, dry-run pipeline wrapper, corpus diff preview — default-off  

---

## Regression evidence (docs-only wave)

Existing suites unchanged — cited as baseline for PLAT waves:

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands` | unchanged |
| Bridge pytest (SA1/SA2 handoff paths) | unchanged |
| `tier0-rt-ui` | unchanged |

---

## Stop line

PLAN-RT-F6 frozen (docs only). Do not start **PLAT-RT-F6 P0** without implementation plan + `rt_plat_f6_p0_*` governance review + freeze audit. Stop before implementation in this wave.
