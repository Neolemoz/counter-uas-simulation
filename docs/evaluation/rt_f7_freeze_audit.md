# RT-F7 — Freeze Audit (PLAN-RT-F7)

**Phase:** PLAN-RT-F7 — post-F6 advisory expansion  
**Status:** frozen (docs only)

**Plan:** [rt_f7_post_f6_advisory_expansion_plan.md](../platform/rt_f7_post_f6_advisory_expansion_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Master plan | [rt_f7_post_f6_advisory_expansion_plan.md](../platform/rt_f7_post_f6_advisory_expansion_plan.md) |
| 2 | Maintainer workflow contract | [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md) |
| 3 | Contamination gates contract | [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) |
| 4 | Aggregation contract | [rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md) |
| 5 | Reference fixtures | [fixtures/rt_handoff/f7_advisory_examples/](../../fixtures/rt_handoff/f7_advisory_examples/) |
| 6 | Architecture review | [rt_f7_architecture_review_r1.md](rt_f7_architecture_review_r1.md) |
| 7 | Governance review | [rt_f7_governance_review_r1.md](rt_f7_governance_review_r1.md) |
| 8 | Contamination review | [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md) |
| 9 | PLAT roadmap | [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md) |
| 10 | Next frontiers v3 | [rt_roadmap_next_frontiers_v3.md](rt_roadmap_next_frontiers_v3.md) |
| 11 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` for this wave.

---

## F7 architecture summary

Post-F6 advisory expansion adds normative **maintainer triage and bulk workflow** (queue prioritization bands, blocker grouping taxonomy, triage lanes, bulk scan→report→preview flow), **stronger contamination gates** (F7-CONT matrix, lineage detect-only warnings, escalation depth limits, forbidden automation superset), and **structured aggregation** (`rt_advisory_batch_summary_v1` with queue priority, blocker group rollups, readiness cohorts, experiment warn-only rollups) atop frozen F6 per-capture derive and P2 batch scan — preserving SA1 commit gate and F6 five-rung ladder semantics.

---

## Boundary guarantees

- Advisory queue, cohorts, and rollups are **not** SA replay authority  
- `capture_session ≠ SA import` preserved  
- Corpus authority begins only at explicit maintainer `commit --corpus-dest`  
- No parser/topic/bridge changes in PLAN wave  
- No SA viewer or auto-import scope  
- No federation writes from RT sessions  
- PLAN-RT-F7 ≠ PLAT-RT-F7 implementation  
- PLAN-RT-F7 ≠ distributed multi-bridge (v1 label retired)  
- PLAT-RT-F7 P0–P2 require per-phase governance + contamination review  

---

## Recommended PLAT-RT-F7 scope (advisory)

See [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md):

- **P0 (recommended next):** Queue priority, blocker groups, batch summary schema, CLI `--sort` / `--group-by`, lineage warn field, golden fixtures  
- **P1:** Read-only triage queue UI, cohort chips, grouped blocker strip  
- **P2:** Bulk hardening, stand-up JSON export, stricter dry-run guards  

**Alternate:** Optional platform checkpoint review if P0 planning reveals F6 P2 overlap — see [rt_roadmap_next_frontiers_v3.md](rt_roadmap_next_frontiers_v3.md).

Default: **PLAT-RT-F7 P0** before optional checkpoint.

---

## Regression evidence (docs-only wave)

Existing suites unchanged — cited as baseline for PLAT waves:

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | unchanged |
| `test_rt_handoff_batch_advisory.py` | unchanged |
| `tier0-rt-ui` | unchanged |

---

## Stop line

**PLAN-RT-F7** frozen (docs only). Do not start **PLAT-RT-F7 P0** without implementation plan + `rt_plat_f7_p0_*` governance review + contamination review + freeze audit. Stop before implementation in this wave.
