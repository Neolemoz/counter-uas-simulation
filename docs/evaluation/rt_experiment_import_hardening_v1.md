# RT Experiment Import Hardening (`rt_experiment_import_hardening_v1`)

**Phase:** PLAN-RT-F2 / PLAT-RT-F2

## Safe parse pattern

All UI imports return `{ ok: true, data }` or `{ ok: false, error: string }` — never throw to alert handlers.

| Schema | Guard |
|--------|-------|
| `rt_experiment_manifest_v1` | `safeParseManifest` |
| `rt_experiment_analytics_report_v1` | `safeParseAnalyticsReport` |
| `rt_tactical_capture_annex_v1` | `safeParseAnnex` |
| `rt_experiment_annex_bundle_v1` | `safeParseAnnexBundle` |

## Annex cache lifecycle

| Rule | Detail |
|------|--------|
| Corrupt cache entry | Skip entry; retain valid keys |
| Manifest import / run remove | `pruneAnnexCacheForManifest` drops orphan run_ids |
| Bundle import | Partial success: import valid entries, report skipped |

## Related

- [rt_experiment_annex_review_ui_v1.md](rt_experiment_annex_review_ui_v1.md)
