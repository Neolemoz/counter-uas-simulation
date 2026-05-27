# RT-X1 — Freeze Audit

**Phase:** PLAT-RT-X1 — experimentation workbench  
**Status:** frozen

Plan: [rt_x1_experimentation_workbench_plan.md](../platform/rt_x1_experimentation_workbench_plan.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | `rt_experiment_manifest_v1` + store/compare | Yes |
| 2 | Experiment workbench + compare UI | Yes |
| 3 | Tactical A/B + telemetry/terrain compare | Yes |
| 4 | `rt_experiment_batch.py` maintainer CLI | Yes |
| 5 | Batch panel (CLI guidance, no browser capture) | Yes |
| 6 | pytest + vitest | Yes |

---

## Experimentation architecture

Pull telemetry snapshots are pinned into `rt_experiment_manifest_v1` (localStorage + JSON export). The UI compares two live sessions or two pinned runs with explanatory diff badges. Maintainer batch CLI sequences `start_session` → dwell → `stop_session` → `capture_session` and writes manifest + annex summaries under `runs/rt_sandbox/experiments/`.

---

## Boundary guarantees

- Explanatory compare only; no SA/parser/tactical authority changes
- Capture ≠ import unchanged
- No bridge protocol changes

---

## Regression evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm test && npm run build
```

---

## Stop line

Do not start post-X1 expansion (full annex timeline UI in RT, template sweep catalog, PLAT-RT-M3 distributed) without explicit new wave audit.
