# RT-F8 P0 — Freeze Audit (PLAT-RT-F8 P0)

**Phase:** PLAT-RT-F8 P0 — advisory summary v2, filter presets, focus sets, template render  
**Status:** frozen

**Plan:** [rt_plat_f8_p0_advisory_v2_implementation_plan.md](../platform/rt_plat_f8_p0_advisory_v2_implementation_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | `ADVISORY_BATCH_SUMMARY_V2_SCHEMA`, presets, focus, cohort v2 | Yes |
| 2 | `build_advisory_batch_summary_v2_document` | Yes |
| 3 | `render_template_pack` (standup_json_v2, standup_md_daily, standup_md_minimal) | Yes |
| 4 | CLI `--schema f8`, `--preset`, `--focus-captures`, `--template-pack`, `--cohort-index-ref` | Yes |
| 5 | TS `advisoryAggregationV2.ts` + types | Yes |
| 6 | Read-only UI strips + `CaptureHandoffWorkflowPanel` wiring | Yes |
| 7 | Pytest + vitest | Yes |
| 8 | P0 reviews | Yes |
| 9 | Registry + AGENTS | Yes |

No changes under `platform/sa-r0-viewer/`. No bridge HTTP protocol changes. No subcommand registry changes.

---

## F8 P0 summary

**Summary v2:** `rt_advisory_batch_summary_v2` with `readiness_cohorts_v2`, `multi_capture_cohorts`, `handoff_rollup`, optional `experiment_handoff_rollup`.  
**Presets:** Eight filter-only preset ids (`blocked_today` … `all_staged`).  
**Focus:** Capture ID intersection with `in_focus_set` flag.  
**Templates:** Render-only packs for maintainer export preview; `dry_run: true` on batch-derived docs.  
**UI:** Preset selector, focus chips, cohort v2 strip, experiment-handoff strip, stand-up passes, template preview — no actions.

---

## Boundary guarantees

- Advisory v2 rollups ≠ authority  
- Presets/templates ≠ CLI invocation  
- No auto-import; `--dry-run` default on batch CLI  
- No `--commit-all`  
- No SA viewer scope  
- No `readiness_score`  

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| `test_advisory_queue.py` | 14 passed |
| `test_rt_handoff_batch_advisory.py` | 19 passed |
| Vitest (rt-sandbox-ui) | 330 passed |
| `npm run build` | pass |

---

## Recommended next (advisory)

| Option | When |
|--------|------|
| **PLAT-RT-F8 P1** | Default — triage preset integration, cohort v2 chips on triage rows |
| **Platform checkpoint review** | Optional if P1 scope overlaps F7 P1 triage panel |

Default: **PLAT-RT-F8 P1** per [rt_roadmap_plat_rt_f8_v1.md](rt_roadmap_plat_rt_f8_v1.md).

---

## Stop line

PLAT-RT-F8 P0 frozen. Do not start **P1** without implementation plan + governance + contamination + freeze. Stop before P2 in this wave.
