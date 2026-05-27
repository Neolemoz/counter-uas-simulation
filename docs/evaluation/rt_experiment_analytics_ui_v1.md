# RT Experiment Analytics UI Contract (`rt_experiment_analytics_ui_v1`)

**Phase:** PLAN-RT-F1 — UI planning (docs); PLAT-RT-F1 implements panels  
**Prerequisite:** PLAT-RT-X1 workbench, PLAN-RT-F1 contracts  
**Authority:** [rt_f1_experiment_analytics_plan.md](../platform/rt_f1_experiment_analytics_plan.md)

Defines **read-only cognition surfaces** for experiment analytics and sweep browsing. Extends the X1 experiment workbench tier — does not replace compare or batch panels.

---

## 1. Governance chrome

| Constant | Value |
|----------|-------|
| `BANNER_ANALYTICS` | `RT ANALYTICS — derived summaries only; not operational authority` |

Additive only — **do not modify** frozen T1/V2/X1 banner strings.

Display `BANNER_ANALYTICS` on: analytics panel, sweep browser, trend strip when visible.

---

## 2. Surfaces

### 2.1 Experiment badges (per-run)

Compact chips derived from `rt_experiment_analytics_report_v1.per_run[]`:

| Chip | Source field |
|------|----------------|
| Mode | `tactical_mode` or `unknown` |
| Entities | `entity_count` |
| Capture | `has_capture` yes/no |
| Annex | sum of `annex_timeline_counts` values when present |

No color semantics implying good/bad outcomes.

### 2.2 Compare summaries

- Reuse X1 [ExperimentComparePanel](../../platform/rt-sandbox-ui/src/experiment/ExperimentComparePanel.tsx) for A/B.  
- Analytics extension: table of `compare_pairs[]` badges for selected run pair or full matrix (upper triangle only).  
- **No winner column**, no score column.

### 2.3 Trend views

`ExperimentTrendStrip` (PLAT-RT-F1):

- X-axis: `recorded_at_utc` order (or `run_id` order if timestamps equal).  
- Y-axis labels: `entity_count`, optional `tti_s` — numeric text only, no operational thresholds.  
- Purpose: visual scan of manifest run sequence — not time-series authority.

### 2.4 Sweep rollups

When manifest linked to sweep group:

| Display | Source |
|---------|--------|
| Group label | `sweep_group_id` |
| Run count | `rollup.run_count` |
| Capture count | `rollup.capture_count` |
| Mode distribution | `rollup.mode_counts` as text histogram |

Label: **counts only — not success rate**.

---

## 3. Planned components (PLAT-RT-F1)

| Component | Path (planned) | Role |
|-----------|----------------|------|
| `ExperimentAnalyticsPanel` | `src/experiment/ExperimentAnalyticsPanel.tsx` | Import report JSON or derive client-side; badges + rollup |
| `SweepCatalogBrowser` | `src/experiment/SweepCatalogBrowser.tsx` | Load catalog fixture/JSON; show groups; emit batch YAML snippet + CLI |
| `ExperimentTrendStrip` | `src/experiment/ExperimentTrendStrip.tsx` | Trend strip from `per_run` |

### Integration

- Collapsible tier in experiment workbench footer (below or beside X1 compare/batch).  
- Wired from [App.tsx](../../platform/rt-sandbox-ui/src/App.tsx) or [ExperimentWorkbenchPanel](../../platform/rt-sandbox-ui/src/experiment/ExperimentWorkbenchPanel.tsx).  
- `RuntimeCognitionHub`: optional one-line when analytics visible — "Experiment analytics — derived summaries only".

---

## 4. Explicit non-features

- SA compare mode import  
- Replay scrubber sync  
- Auto handoff commit  
- `federation_register` from UI  
- Browser `capture_session` or subprocess batch  
- Live bridge re-pull for analytics (use pinned manifest)  
- Readiness / effectiveness / winner UI  

---

## 5. Import paths

| Artifact | Import method |
|----------|----------------|
| Analytics report JSON | File picker / paste JSON (maintainer) |
| Sweep catalog | Default load `fixtures/rt_experiments/sweep_catalog_v1.yaml` path text + import button |
| Manifest | Reuse X1 import |

Optional dev static serve of `/fixtures/rt_experiments/` — not required; import button is primary.

---

## 6. Related

- [rt_experiment_analytics_v1.md](rt_experiment_analytics_v1.md)
- [rt_experiment_sweep_catalog_v1.md](rt_experiment_sweep_catalog_v1.md)
- [rt_experiment_workbench_v1.md](rt_experiment_workbench_v1.md)
