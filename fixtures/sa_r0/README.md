# SA-R0 platform fixtures

Committed source of truth for the frozen SA-R0 replay platform (PLAT-SA-R0 through PLAT-SA-F1a). Served interactively via mirror at `platform/sa-r0-viewer/public/demo/`.

**Explanatory replay artifacts only** — not operational authority.

## Directory map

| Path | Contents |
|------|----------|
| `demo_<pack_id>/` | `replay_sa_bundle_v1` index + demo log/meta sidecars |
| `sweeps/<sweep_id>/` | MC sweep manifests, members, spatial analytics, reports |
| `presentations/` | Storyboard decks + `index.json` |
| `synthesis/` | Cross-sweep synthesis, linkage, cognition rollups, corpus index |
| `corpus_releases/` | Deterministic release snapshots (`sa_r0_corpus_r1_r1/`) |
| `corpus_audits/` | Drift report, release diff, regen run report (F1b) |
| `research_bundles/sa_r0_corpus_r1/` | Portable offline corpus (+ `.zip`) |

Default viewer bundle: `demo_ridge_defense/` (mirrored to `public/demo/index.json`).

## Regeneration

Do not edit `public/demo/` directly. Use the maintainer checklist:

[docs/evaluation/sa_platform_maintainer_checklist.md](../../docs/evaluation/sa_platform_maintainer_checklist.md)

Quick gate:

```bash
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

## Reviewer entry

[docs/evaluation/sa_r0_reviewer_quickstart.md](../../docs/evaluation/sa_r0_reviewer_quickstart.md)
