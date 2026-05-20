# SA-R0 Replay Viewer

Read-only replay-analysis viewer for `replay_sa_bundle_v1` artifacts. Not a live operational or tactical display.

## Quick start

```bash
cd platform/sa-r0-viewer
npm install
npm run dev
```

Open http://localhost:5173 (loads `public/demo/index.json` by default).

**Reviewer guide:** [docs/evaluation/sa_r0_reviewer_quickstart.md](../../docs/evaluation/sa_r0_reviewer_quickstart.md) — URL cheat sheet, compare/sweep/presentation workflows, and 15-minute onboarding path.

**Scenario catalog (C1b):** Use the in-app scenario picker (loads `public/demo/catalog.json`) or `?demo=<pack_id>` (e.g. `?demo=saturation_ingress`). Tag filters are client-side only.

Custom bundle: `?bundle=/path/to/index.json` (must be served; use dev server `fs.allow` or copy under `public/`).

Regenerate demo bundles and sync catalog from repo root:

```bash
python3 scripts/evaluation/sync_sa_catalog.py
```

See [docs/evaluation/sa_platform_maintainer_checklist.md](../../docs/evaluation/sa_platform_maintainer_checklist.md) for full regen order and integrity gates.

## Build demo bundle from repo root

```bash
python3 scripts/evaluation/replay_sa_bundle.py pack \
  --narrative-json runs/evaluation/RUN.replay_narrative.json \
  --observability-json runs/evaluation/RUN.replay_observability.json \
  --viz-manifest runs/evaluation/RUN.replay_viz/replay_static_visualization.json \
  --scenario-pack fixtures/scenarios/ridge_defense \
  --out-dir runs/evaluation/RUN.sa_bundle/
```

## Governance

- Mode: `replay_static` only
- Fictional georef — not deployed geography
- Dashed tracks = log-evidenced samples, not continuous path truth

See [docs/evaluation/sa_r0_implementation_plan.md](../../docs/evaluation/sa_r0_implementation_plan.md).
