# SA Platform Maintainer Checklist

Post-STAB maintenance runbook for `fixtures/sa_r0/` and `platform/sa-r0-viewer/public/demo/`. **Additive integrity tooling only** — no new platform features.

Authority: [sa_stabilization_plan.md](sa_stabilization_plan.md), [sa_stabilization_freeze_audit.md](sa_stabilization_freeze_audit.md).

## Pre-merge / pre-share gate

Run from repository root:

```bash
python3 -m pytest src/counter_uas/test/ -q --tb=short
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

All checks must pass (warnings on duplicate narrative bullets are acceptable per STAB policy).

## Ordered regeneration

When audit stderr indicates drift, run in this order:

```bash
python3 scripts/evaluation/sync_sa_catalog.py
python3 scripts/evaluation/gen_d3_sweep_enrichment.py
python3 scripts/evaluation/gen_e1_presentation_fixtures.py
python3 scripts/evaluation/gen_e2_research_fixtures.py
```

Then re-run the pre-merge gate above.

## Audit check → fix mapping

| Check ID | Typical fix |
|----------|-------------|
| `catalog_sync` | `sync_sa_catalog.py` |
| `fixture_pub_parity` | Regen script for affected subtree (see stderr hint) |
| `synthesis_stale` | `gen_e2_research_fixtures.py` or `build_cross_sweep_synthesis.py` |
| `linkage_stale` | `gen_e2_research_fixtures.py` or `build_replay_linkage.py` |
| `linkage_graph` | Fix linkage JSON sources, then `build_replay_linkage.py` |
| `storyboard_urls` | Fix presentation JSON scene `target_url`s, then `gen_e1_presentation_fixtures.py` |
| `pattern_taxonomy` | Fix sweep member tags or `PATTERN_PRIORITY`, then `gen_d3_sweep_enrichment.py` |
| `narrative_duplicates` | Fix empty bullets in sweep enrichment sources |
| `sweep_reports` | `export_replay_analytics_report.py --sweep <id> --format md,json` then E1/E2 regen |
| `research_bundle` | `gen_e2_research_fixtures.py` |
| `bundle_catalog` | `sync_sa_catalog.py` + verify `fixtures/scenarios/` packs |
| `governance_batch` | Fix forbidden wording in fixture copy, then regen |

## Per-sweep report refresh (manual)

```bash
python3 scripts/evaluation/export_replay_analytics_report.py --sweep ridge_overlap_sweep --format md,json
```

Repeat for each sweep ID, then run E1/E2 orchestrators if publication or synthesis exports changed.

## Three-tier sync model

| Tier | Path | Sync |
|------|------|------|
| Source | `fixtures/sa_r0/` | Written by regen scripts |
| Viewer mirror | `platform/sa-r0-viewer/public/demo/` | Dual-written by sync/gen scripts |
| Research bundle | `fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/` | `export_research_bundle.py` via `gen_e2_research_fixtures.py` |

Do not manually `cp` fixture files to `public/demo/` — use the scripts above.

## Parity allowlists

Enforced by `sa_integrity_lib.py`:

- **Synthesis:** JSON/MD files including `storyline_linkage_overlay_v1.json`
- **Sweep reports:** analytics, narrative, cluster, compare/review JSON, publication HTML/MD
- **PNG assets:** not byte-parity gated in CI (maintainer regen via `gen_presentation_assets.py`)

## Forbidden in maintenance passes

- Matplotlib regeneration on every CI run
- Parser/topic/schema changes
- Viewer feature or GovernanceChrome redesign
- Operational/readiness semantics in fixture copy

## Related

- [fixtures/sa_r0/README.md](../../fixtures/sa_r0/README.md) — directory map
- [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md) § Platform fixture maintenance
- [scripts/evaluation/README.md](../../scripts/evaluation/README.md) — evaluation script index
