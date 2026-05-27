# SA Platform Maintainer Checklist

Post-STAB maintenance runbook for `fixtures/sa_r0/` and `platform/sa-r0-viewer/public/demo/`. **Additive integrity tooling only** — no new platform features.

Authority: [sa_stabilization_plan.md](sa_stabilization_plan.md), [sa_stabilization_freeze_audit.md](sa_stabilization_freeze_audit.md).

UX architecture (plan-only; no viewer changes until PLAT-SA-H2): [h1_sandbox_ux_architecture_plan.md](../platform/h1_sandbox_ux_architecture_plan.md) (PLAN-SA-H1).

Offline experiment orchestration (PLAT-SA-H3): [experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md); run `python3 scripts/evaluation/run_experiment_queue.py --dry-run --manifest fixtures/orchestration/manifests/ridge_defense_synthetic.json` then `sync_orchestration_mirrors.py`.

Replay workstation integration (PLAT-SA-H4): [experiment_workflow_continuity_v1.md](experiment_workflow_continuity_v1.md) — reviewer navigation path (Scenario → Replay → Compare → Corpus → Report); viewer read-only, no CLI changes.

Presentation UX polish (PLAT-SA-H5): [h5_publication_presentation_ux_polish_plan.md](../platform/h5_publication_presentation_ux_polish_plan.md) — presentation mode → Print layout / Save map frame / Copy chapter; Report storyboard cards.

Authoring workstation (PLAT-SA-A1): [scenario_authoring_workflow_v1.md](scenario_authoring_workflow_v1.md) — `promote_scenario_pack.py`, `sync_authoring_mirrors.py`; viewer AUTHORING panels read-only. After pack edits: `--record-validation` then `--status promoted`; sync mirrors before share.

Authoring operations (PLAT-SA-A2): [scenario_authoring_operations_v1.md](scenario_authoring_operations_v1.md) — `audit_scenario_authoring_integrity.py --strict`, `lint_scenario_authoring_manifest.py --all-catalog-packs`; `sync_authoring_mirrors.py` refreshes `integrity_report.json`. Catalog sync: `sync_sa_catalog.py` (warn) or `--strict-promotion`. Promotion ergonomics: `--summary`, `--repro-check`, `--lineage-report` via audit CLI.

Orchestration operations (PLAT-SA-I1): [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md) — `audit_orchestration_integrity.py --strict`, `lint_orchestration_ops_manifest.py --all-manifests`; `promote_experiment_manifest.py` for ops lifecycle; `sync_orchestration_mirrors.py` refreshes `integrity_report.json` and full queue/audit/ops mirrors. Continuity: [experiment_orchestration_continuity_v1.md](experiment_orchestration_continuity_v1.md).

Async orchestration (PLAT-SA-I2, frozen): [sa_i2_async_orchestration_plan.md](../platform/sa_i2_async_orchestration_plan.md) — [experiment_orchestration_async_operations_v1.md](experiment_orchestration_async_operations_v1.md). Commands: `record_async_execution.py`, `audit_orchestration_async_integrity.py --strict`, `lint_orchestration_async_manifest.py --all-manifests --check`. Flags: `--allow-async-worker` (never default on `run_experiment_queue.py`). Stop line: no distributed workers, browser authority, or federation.

Async recovery (PLAT-SA-I3, frozen): [sa_i3_async_recovery_plan.md](../platform/sa_i3_async_recovery_plan.md) — [experiment_orchestration_async_recovery_v1.md](experiment_orchestration_async_recovery_v1.md). Commands: `audit_orchestration_recovery.py --strict`, `audit_orchestration_recovery.py --refresh-fixtures`, `sync_orchestration_mirrors.py`. Stop line: no federation, multi-corpus ops, live retry orchestration, or browser-triggered recovery.

Multi-corpus federation (PLAT-SA-F2A, frozen): [sa_f2a_multi_corpus_federation_plan.md](sa_f2a_multi_corpus_federation_plan.md). Commands: `build_replay_federation_index.py --check`, `audit_replay_federation_integrity.py --check --strict`, `audit_federation_recovery_continuity.py --check --strict`, `gen_f2a_federation_fixtures.py`. Stop line: no cloud federation, live sync, collaborative editing, or browser orchestration.

RT interactive sandbox — **consolidation plateau** (PLAN-RT-C1 frozen; PLAT-RT-F1–F6 P2 frozen): [rt_c1_platform_consolidation_freeze_audit.md](rt_c1_platform_consolidation_freeze_audit.md), [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md). Prior maturity plateau: [rt_r2_platform_maturity_freeze_audit.md](rt_r2_platform_maturity_freeze_audit.md), [rt_f1_freeze_audit.md](rt_f1_freeze_audit.md), [rt_roadmap_plat_rt_f1_v1.md](rt_roadmap_plat_rt_f1_v1.md). Dev-only: `scripts/rt/run_rt_bridge.py` → `capture_session` via `rt_experiment_batch.py`; sweep catalog [fixtures/rt_experiments/sweep_catalog_v1.yaml](../../fixtures/rt_experiments/sweep_catalog_v1.yaml). Staging: `runs/rt_sandbox/captures/`, `runs/rt_sandbox/experiments/`. PLAT-RT-F1 frozen: `rt_experiment_analytics.py` derives reports from manifests; sweep catalog at `fixtures/rt_experiments/sweep_catalog_v1.yaml`. PLAT-RT-F3 frozen: `rt_experiment_annex_pack.py` packs `tactical_annex.json` into workbench import bundles. PLAT-RT-F2 frozen: `clear_tactical_state` on session teardown; optional read-only `python3 scripts/rt/rt_staging_integrity_audit.py` for staging dirs missing `candidate.json`. Stop line: no post-F2 expansion without wave audit; no auto SA ingestion; browser must not call `capture_session`.

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
python3 scripts/evaluation/gen_f1_corpus_fixtures.py
python3 scripts/evaluation/run_replay_corpus_regen.py
```

Preview regen steps: `python3 scripts/evaluation/run_replay_corpus_regen.py --dry-run`

Corpus reproducibility gate: `python3 scripts/evaluation/verify_replay_corpus_reproducibility.py`

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
| `research_bundle` | `gen_e2_research_fixtures.py` or `gen_f1_corpus_fixtures.py` |
| `corpus_index_stale` | `build_replay_corpus_index.py` |
| `corpus_lineage` | `audit_replay_lineage.py` after index regen |
| `corpus_release_stale` | `build_replay_corpus_release.py` |
| `corpus_drift_stale` | `build_replay_corpus_drift_report.py` |
| `corpus_provenance` | `audit_replay_corpus_provenance.py` |
| `corpus_release_diff` | `diff_replay_corpus_releases.py` then refresh release if needed |
| `corpus_viewer_audit_mirror` | `build_replay_corpus_drift_report.py` + `build_replay_corpus_release.py` (dual-write viewer mirrors) |
| `corpus_evolution_stale` | `build_replay_corpus_evolution.py` |
| `corpus_publication_stale` | `build_replay_corpus_publication.py` |
| `corpus_release_export` | `export_replay_corpus_release.py` |
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

- [sa_platform_governance_review_r1.md](sa_platform_governance_review_r1.md) — G1 boundaries, freeze posture, roadmap (post F1)
- [sa_platform_maturity_assessment_r1.md](sa_platform_maturity_assessment_r1.md) — G1 maturity and sustainability
- [sa_platform_frontier_review_r1.md](sa_platform_frontier_review_r1.md) — G1 frontier candidate matrix
- [sa_platform_release_checkpoint.md](sa_platform_release_checkpoint.md) — merge-ready release gate summary
- [fixtures/sa_r0/README.md](../../fixtures/sa_r0/README.md) — directory map
- [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md) § Platform fixture maintenance
- [scripts/evaluation/README.md](../../scripts/evaluation/README.md) — evaluation script index
