# RT-F3 — Analytics Review R1 (PLAT-RT-F3)

**Phase:** PLAT-RT-F3 — analytics + annex join

## Join semantics

| Layer | Role in continuity hub |
|-------|------------------------|
| `deriveExperimentAnalytics` per_run | Run metrics (mode, entities, capture, normalization ref) |
| Manifest `tactical_annex_summary` | Count hints on manifest |
| Annex cache | Full timelines — does not alter F1 rollup |

F1 `rollupFromPerRun` unchanged — annex cache is orthogonal.

## Verdict

**Pass** — analytics remain derived-only; annex adds explanatory depth only.
