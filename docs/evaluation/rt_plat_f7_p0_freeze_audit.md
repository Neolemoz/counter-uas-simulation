# RT-F7 P0 — Freeze Audit (PLAT-RT-F7 P0)

**Phase:** PLAT-RT-F7 P0 — advisory queue + aggregation foundations  
**Status:** frozen

**Plan:** [rt_plat_f7_p0_advisory_queue_implementation_plan.md](../platform/rt_plat_f7_p0_advisory_queue_implementation_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | `advisory_queue.py` | Yes |
| 2 | `build_advisory_batch_summary_document` | Yes |
| 3 | `lineage_warnings` on derive | Yes |
| 4 | CLI `--sort` / `--group-by` / `--manifest-ref` / `--schema` | Yes |
| 5 | TS `advisoryQueue.ts` + aggregate extensions | Yes |
| 6 | UI chips + batch mirror strip | Yes |
| 7 | `CaptureHandoffWorkflowPanel` wiring | Yes |
| 8 | Pytest + vitest | Yes |
| 9 | P0 reviews | Yes |
| 10 | Registry + AGENTS | Yes |

No changes under `platform/sa-r0-viewer/`. No bridge HTTP protocol changes.

---

## F7 P0 summary

**Queue:** Priority bands P0_block–P7_terminal with stable sort for batch reports and UI table.  
**Aggregation:** `rt_advisory_batch_summary_v1` extends F6 summary with `blocker_groups`, `readiness_cohorts`, optional `experiment_rollup`.  
**Lineage:** Detect-only `lineage_warnings` on per-capture derive.  
**UI:** Read-only queue band, cohort, and blocker chips; session rollup strip — no actions.

---

## Boundary guarantees

- Advisory queue/cohorts ≠ authority  
- `capture_session ≠ SA import` preserved  
- No auto-import; `--dry-run` default on batch CLI  
- No `--commit-all`  
- No SA viewer scope  
- No bridge subcommand registry changes  

---

## Regression evidence

| Suite | Result |
|-------|--------|
| `lint_rt_runtime_subcommands --check` | pass |
| `test_advisory_queue.py` | 8 passed |
| `test_rt_handoff_batch_advisory.py` | 12 passed |
| Vitest (rt-sandbox-ui) | 241 passed |
| `npm run build` | pass |

---

## Recommended next (advisory)

| Option | When |
|--------|------|
| **PLAT-RT-F7 P1** | Default — dedicated triage queue panel, grouped blocker strip polish |
| **Platform checkpoint review** | Optional if P1 scope overlaps App.tsx concentration |

Default: **PLAT-RT-F7 P1** per [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md).

---

## Stop line

PLAT-RT-F7 P0 frozen. Do not start **P1** without implementation plan + governance + freeze. Stop before P2 in this wave.
