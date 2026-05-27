# RT-F1 — Freeze Audit (PLAT-RT-F1)

**Phase:** PLAT-RT-F1 — experiment analytics + sweep catalog  
**Status:** frozen

Plan: [rt_plat_f1_experiment_analytics_implementation_plan.md](../platform/rt_plat_f1_experiment_analytics_implementation_plan.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | `deriveExperimentAnalytics` + Zod report | Yes |
| 2 | `sweepCompile` + catalog fixture | Yes |
| 3 | Analytics + trend + sweep UI | Yes |
| 4 | Workbench + cognition hub wiring | Yes |
| 5 | `rt_experiment_analytics.py` | Yes |
| 6 | Vitest + pytest | Yes |

---

## Architecture

Manifest (+ optional batch spec) → `rt_experiment_analytics_report_v1` in browser or maintainer CLI. Sweep catalog groups compile to batch YAML for `rt_experiment_batch.py` (X1). No live bridge re-pull for stored runs.

---

## Boundary guarantees

- Derived analytics are not operational authority
- No bridge/SA/parser changes
- Browser `capture_session` still forbidden
- No readiness/winner rollups

---

## Regression evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py \
  src/counter_uas/test/test_rt_experiment_analytics.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

---

## Next frontier (advisory)

Per [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md): optional **F3** annex timeline UI in RT workbench, or **platform hardening** waves — not M3, SA auto-import, or distributed multi-bridge without new audit.

---

## Stop line

Do not start post-F1 expansion without explicit new wave audit.
