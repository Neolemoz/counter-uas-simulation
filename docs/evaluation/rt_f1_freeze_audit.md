# RT-F1 — Freeze Audit (PLAN-RT-F1)

**Phase:** PLAN-RT-F1 — experiment analytics & template sweep catalog  
**Status:** frozen (docs only)

**Plan:** [rt_f1_experiment_analytics_plan.md](../platform/rt_f1_experiment_analytics_plan.md), [rt_f1_template_sweep_catalog_plan.md](../platform/rt_f1_template_sweep_catalog_plan.md)

---

## Scope delivered

| # | Deliverable | Done |
|---|-------------|------|
| 1 | Analytics contract | [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md) |
| 2 | Sweep catalog contract | [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md) |
| 3 | UI planning contract | [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md) |
| 4 | Reference fixture | [fixtures/rt_experiments/sweep_catalog_v1.yaml](../../fixtures/rt_experiments/sweep_catalog_v1.yaml) |
| 5 | Architecture review | [rt_f1_architecture_review_r1.md](rt_f1_architecture_review_r1.md) |
| 6 | Analytics review | [rt_f1_analytics_review_r1.md](rt_f1_analytics_review_r1.md) |
| 7 | Governance review | [rt_f1_governance_review_r1.md](rt_f1_governance_review_r1.md) |
| 8 | PLAT roadmap | [rt_roadmap_plat_rt_f1_v1.md](rt_roadmap_plat_rt_f1_v1.md) |
| 9 | Registry + AGENTS | Yes |

No changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, or `src/counter_uas/` for this wave.

---

## Analytics architecture summary

**Inputs:** `rt_experiment_manifest_v1`, optional `rt_experiment_batch_v1`, optional capture staging file reads.

**Output:** `rt_experiment_analytics_report_v1` with per-run metrics (entity/sync/lifecycle/tactical/capture), pairwise compare badges (X1 ids), and count-only rollups.

**Principles:** Deterministic derive, explanatory-only, no bridge API changes, no SA authority.

---

## Sweep catalog summary

**Schema:** `rt_experiment_sweep_catalog_v1` with named groups (`ridge_defense_variants`, `sensor_range_variants`, `tactical_mode_comparison`) compiling to `rt_experiment_batch_v1` via `explicit_list`.

**Templates:** Builtin `rt_runtime_template_v1` only — not SA scenario corpus.

**Workflow:** Catalog → batch YAML → `rt_experiment_batch.py` → manifest → analytics (PLAT-RT-F1).

---

## Boundary guarantees

- Analytics and rollups are **not** operational authority  
- No parser/topic/bridge changes in PLAN wave  
- Browser `capture_session` still forbidden (X1)  
- No SA viewer or auto-import scope  

---

## Recommended PLAT-RT-F1 scope (advisory)

See [rt_roadmap_plat_rt_f1_v1.md](rt_roadmap_plat_rt_f1_v1.md):

- **P0:** `analyticsDerive.ts`, `ExperimentAnalyticsPanel`, `SweepCatalogBrowser`, workbench wiring  
- **P1:** `rt_experiment_analytics.py`, trend strip  
- **P2:** sweep compile helper + catalog tests  

**Not authorized** by this freeze.

---

## Regression evidence

```text
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
→ lint_rt_runtime_subcommands OK (7 subcommands)

python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py -q
→ 147 passed

cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
→ 104 passed (31 files); build OK

scripts/ci_eval.sh tier0-rt-ui
→ OK
```

Recorded at PLAN-RT-F1 freeze.

---

## Stop line

**PLAN-RT-F1 frozen.** Do not start **PLAT-RT-F1** implementation without PLAT plan + governance review + freeze audit.

**Verdict: frozen** for PLAN-RT-F1 (documentation only).
