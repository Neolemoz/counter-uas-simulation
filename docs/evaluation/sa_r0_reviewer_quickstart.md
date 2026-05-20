# SA-R0 Reviewer Quickstart

Fast path for mentors and reviewers using the frozen SA-R0 replay platform (PLAT-SA-R0 through PLAT-SA-E2). **Explanatory replay review only** — not operational authority, not readiness, not causal proof.

For evidence-layer rules, read [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) first. For static HTML onboarding, see [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md) § Fifteen-minute reviewer onboarding path.

## Start the viewer

```bash
cd platform/sa-r0-viewer
npm install
npm run dev
```

Open http://localhost:5173 (default bundle: `ridge_defense`).

## Fifteen-minute SA-R0 path

| Minutes | Activity | Where |
|---------|----------|-------|
| 0–3 | Evidence layers + forbidden wording | [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) |
| 3–6 | Default demo spatial orientation | `http://localhost:5173/` |
| 6–9 | Catalog pack + compare | `?demo=valley_ingress`, then `?pair=valley_vs_ridge_replay` |
| 9–12 | Sweep workstation + filmstrip | `?sweep=ridge_overlap_sweep&member=0`, then `?filmstrip=0,1,2` |
| 12–15 | Storyboard walkthrough | `?presentation=walkthrough_valley_ingress_long&demo=valley_ingress&chapter=0` |

Close every segment with explicit negation: derived summary, not operational readiness, not causal proof.

## URL cheat sheet

Base URL: `http://localhost:5173`

| Goal | Query string |
|------|--------------|
| Default demo (`ridge_defense`) | *(none)* |
| Catalog pack | `?demo=valley_ingress` |
| Compare (curated pair) | `?pair=valley_vs_ridge_replay` |
| Compare (ad hoc) | `?compare=valley_ingress,saturation_ingress` |
| Sweep member | `?sweep=ridge_overlap_sweep&member=0` |
| Cohort filmstrip (2–4 members) | `?sweep=ridge_overlap_sweep&filmstrip=0,1,2` |
| Predefined cohort | `?sweep=ridge_overlap_sweep&cohort=los_fragmented_replay_cohort` |
| Bundle chapter walkthrough | `?demo=valley_ingress&walkthrough=1&chapter=0` |
| Storyboard deck | `?presentation=walkthrough_valley_ingress_long&demo=valley_ingress&chapter=0` |
| Cross-sweep synthesis (viewer) | Open synthesis panel in viewer chrome after loading any sweep |
| Cross-sweep summary (offline) | `/demo/synthesis/cross_sweep_summary.md` |

**Compare modes:** `?pair=` loads a curated entry from `compare_pairs.json`. `?compare=packA,packB` loads two catalog packs ad hoc. Both are explanatory side-by-side views only.

**Walkthrough vs storyboard:** Use `walkthrough=1` for bundle `presentation.chapters` on a single demo pack. Use `presentation=<storyboard_id>` for cross-demo storyboard decks under `/demo/presentations/`. Pick one mode per URL — they are not combined.

## Storyboard decks

Index: `/demo/presentations/index.json` (six decks).

| Storyboard ID | Smoke URL |
|---------------|-----------|
| `showcase_best_narratives` | `?presentation=showcase_best_narratives&sweep=valley_sensor_sweep&chapter=0` |
| `showcase_topology_divergence` | `?presentation=showcase_topology_divergence&sweep=ridge_overlap_sweep&chapter=0` |
| `showcase_ambiguity_heavy` | `?presentation=showcase_ambiguity_heavy&sweep=delayed_detection_sweep&chapter=0` |
| `walkthrough_valley_ingress_long` | `?presentation=walkthrough_valley_ingress_long&demo=valley_ingress&chapter=0` |
| `walkthrough_assignment_instability` | `?presentation=walkthrough_assignment_instability&sweep=saturation_assignment_sweep&chapter=0` |
| `showcase_cross_sweep_synthesis` | `?presentation=showcase_cross_sweep_synthesis&sweep=ridge_overlap_sweep&chapter=0` |

## Three-tier artifact model

| Tier | Path | Use |
|------|------|-----|
| Source | `fixtures/sa_r0/` | Committed source of truth |
| Viewer mirror | `platform/sa-r0-viewer/public/demo/` | Interactive review via `npm run dev` |
| Offline archive | `fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/` (+ `.zip`) | Portable mentor handoff; not the live viewer |

The research bundle zip is for offline reading and print-to-PDF. The viewer is for interactive spatial orientation and sweep workstation review.

## Static exports

Per-sweep publication packets: `fixtures/sa_r0/sweeps/<id>/reports/publication_packet.html` (mirrored under `public/demo/sweeps/`).

Open via viewer static server, e.g. `/demo/sweeps/ridge_overlap_sweep/reports/publication_packet.html`.

Sweep context is required for in-viewer “Export review pack” links — load a `?sweep=` URL first.

## Sensor placement studies

Valley experiment packs (`valley_ingress_*`) share the valley demo log. See [comparison_foundations.md](comparison_foundations.md) § Sensor placement study before comparing geometry variants.

## Related documents

- [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md) — full mentor workflow
- [replay_compare_v1.md](replay_compare_v1.md) — compare mode contract
- [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md) — sweep manifests
- [replay_storyboard_v1.md](replay_storyboard_v1.md) — storyboard schema
- [sa_platform_maintainer_checklist.md](sa_platform_maintainer_checklist.md) — maintainer regen (not required for reviewers)
