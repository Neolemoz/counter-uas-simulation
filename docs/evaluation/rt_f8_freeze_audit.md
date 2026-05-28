# RT-F8 — Freeze Audit (PLAN-RT-F8)

**Phase:** PLAN-RT-F8 — post-F7 advisory maintainer expansion  
**Status:** frozen (docs only)

**Plan:** [rt_f8_post_f7_advisory_maintainer_expansion_plan.md](../platform/rt_f8_post_f7_advisory_maintainer_expansion_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Master plan | [rt_f8_post_f7_advisory_maintainer_expansion_plan.md](../platform/rt_f8_post_f7_advisory_maintainer_expansion_plan.md) |
| 2 | Maintainer workflow v2 | [rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md) |
| 3 | Contamination gates v2 | [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md) |
| 4 | Aggregation v2 | [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md) |
| 5 | Reference fixtures | [fixtures/rt_handoff/f8_advisory_examples/](../../fixtures/rt_handoff/f8_advisory_examples/) |
| 6 | Architecture review | [rt_f8_architecture_review_r1.md](rt_f8_architecture_review_r1.md) |
| 7 | Governance review | [rt_f8_governance_review_r1.md](rt_f8_governance_review_r1.md) |
| 8 | Contamination review | [rt_f8_handoff_contamination_review_r1.md](rt_f8_handoff_contamination_review_r1.md) |
| 9 | PLAT roadmap | [rt_roadmap_plat_rt_f8_v1.md](rt_roadmap_plat_rt_f8_v1.md) |
| 10 | Next frontiers v8 | [rt_roadmap_next_frontiers_v8.md](rt_roadmap_next_frontiers_v8.md) |
| 11 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` for this wave.

---

## F8 architecture summary

Post-F7 advisory maintainer expansion adds **maintainer ergonomics v2** (named filter presets, focus sets, time-boxed stand-up passes, review-lane templates, stand-up template packs), **contamination gates v2** (F8-CONT-01..14 matrix, LIN-06..10, X2 packet/cohort adjacency, escalation depth through template render), and **aggregation v2** (`rt_advisory_batch_summary_v2` with `multi_capture_cohorts`, `handoff_rollup`, `readiness_cohorts_v2`, and `experiment_handoff_rollup`) atop frozen PLAT-RT-F7 queue/triage/export — preserving SA1 commit gate, F6 five-rung ladder semantics, and F7 P0–P7 band ranks.

---

## Boundary guarantees

- Filter presets, cohort v2, and rollups are **not** SA replay authority  
- `capture_session ≠ SA import` preserved  
- Corpus authority begins only at explicit maintainer `commit --corpus-dest`  
- No parser/topic/bridge changes in PLAN wave  
- No SA viewer or auto-import scope  
- No federation writes from RT sessions  
- X2 experiment cohort ≠ readiness cohort v2  
- PLAN-RT-F8 ≠ PLAT-RT-F8 implementation  
- PLAN-RT-F8 ≠ PLAT-RT-F7 replacement  
- PLAT-RT-F8 P0–P2 require per-phase governance + contamination review  

---

## Recommended next (advisory)

See [rt_roadmap_plat_rt_f8_v1.md](rt_roadmap_plat_rt_f8_v1.md) and [rt_roadmap_next_frontiers_v8.md](rt_roadmap_next_frontiers_v8.md):

| Option | When |
|--------|------|
| **PLAT-RT-F8 P0** (recommended) | Maintainer advisory throughput is binding — summary v2 + presets |
| **PLAN-RT-V4** (alternate) | Demo/visualization cognition is binding |
| **PLAN-RT-X3** (alternate) | X2 compare ergonomics gaps evidenced post-X2 |

Default: **PLAT-RT-F8 P0** after PLAN freeze — **not authorized** by this audit alone.

---

## Regression evidence (docs-only wave)

Existing suites unchanged — cited as baseline for PLAT waves:

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | unchanged |
| `test_advisory_queue.py` + `test_rt_handoff_batch_advisory.py` | unchanged |
| `tier0-rt-ui` | unchanged |

---

## Stop line

**PLAN-RT-F8** frozen (docs only). Do not start **PLAT-RT-F8 P0** without implementation plan + `rt_plat_f8_p0_*` governance review + contamination review + freeze audit. Stop before implementation in this wave.
