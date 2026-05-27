# RT-F3 — Freeze Audit (PLAT-RT-F3)

**Phase:** PLAT-RT-F3 — tactical annex & continuity review UI  
**Status:** frozen

Plan: [rt_plat_f3_experiment_annex_review_implementation_plan.md](../platform/rt_plat_f3_experiment_annex_review_implementation_plan.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | Annex schema + localStorage cache | Yes |
| 2 | Tactical annex review panel + timeline tables | Yes |
| 3 | Continuity review hub (analytics + lineage + annex) | Yes |
| 4 | Compare annex strip | Yes |
| 5 | `rt_experiment_annex_pack.py` | Yes |
| 6 | Workbench + cognition wiring | Yes |
| 7 | Vitest + pytest | Yes |

---

## Review workflow

1. Batch run → manifest with `capture_staging_ref` + annex summary.
2. `rt_experiment_annex_pack.py --manifest … --out bundle.json` → import in workbench.
3. **Continuity review** → per-run analytics, capture lineage chips, full timelines.
4. **Compare** → annex finals/count strip for pinned runs with loaded annex.
5. SA import remains manual (SA1) — review does not trigger import.

---

## Regression evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py \
  src/counter_uas/test/test_rt_experiment_annex_pack.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## Next frontier (advisory)

Per [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md): **platform hardening (F2)** — lint/docs debt, maintainer ergonomics. Not M3, F5 SA automation, or F4 realism without new audit.

---

## Stop line

Do not start post-F3 expansion without explicit new wave audit.
