# RT-F3 — Architecture Review R1 (PLAN-RT-F3)

**Phase:** PLAN-RT-F3  
**Contracts:** [rt_experiment_annex_review_ui_v1.md](rt_experiment_annex_review_ui_v1.md), [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md)

## Boundary: summary vs full annex

| Storage | Content | Written by |
|---------|---------|------------|
| Manifest `tactical_annex_summary` | Final mode, ids, timeline **counts** | `rt_experiment_batch.py` |
| Annex cache (browser) | Full `rt_tactical_capture_annex_v1` | User import / annex pack CLI |
| Staging `tactical_annex.json` | Source of truth at capture | TAC5 bridge |

F1 rollups continue to use manifest summary counts only — F3 does not change `rollupFromPerRun`.

## SA parity

SA-R0 `TacticalReplayContinuityPanel` is the **consumption** reference. RT F3 mirrors table UX locally without importing `sa-r0-viewer` packages.

## Verdict

**Pass** — architecture is additive, low risk, RT-local.
