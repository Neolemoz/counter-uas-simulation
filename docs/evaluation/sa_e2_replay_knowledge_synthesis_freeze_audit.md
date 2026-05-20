# SA E2 — Replay Knowledge Synthesis & Research Publication Freeze Audit (PLAT-SA-E2)

Wave plan: [sa_e2_replay_knowledge_synthesis_plan.md](sa_e2_replay_knowledge_synthesis_plan.md)

Schema docs: [replay_cross_sweep_synthesis_v1.md](replay_cross_sweep_synthesis_v1.md), [replay_linkage_v1.md](replay_linkage_v1.md), [replay_cognition_rollup_v1.md](replay_cognition_rollup_v1.md), [replay_research_bundle_v1.md](replay_research_bundle_v1.md), [replay_publication_packet_v1.md](replay_publication_packet_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| E2 schema docs (synthesis, linkage, cognition, research bundle, publication) | Done |
| `build_cross_sweep_synthesis.py` cross-sweep rollups | Done |
| `build_replay_linkage.py` rule-based linkage index | Done |
| `build_replay_cognition_rollup.py` reviewer cognition summaries | Done |
| `replay_viz_sweep_figures.py` static sweep figures (EVAL-VIZ-E2 slice) | Done |
| Publication HTML/MD + `replay_publication_html.py` | Done |
| `gen_presentation_assets.py` thumbnail/card generation | Done |
| `export_research_bundle.py` portable corpus bundles | Done |
| Viewer synthesis/linkage panels + cross-sweep storyboard deck | Done |
| `gen_e2_research_fixtures.py` orchestrator + tests + CI | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No HITL / engage / readiness UX | Pass |
| Explanatory-only synthesis and linkage copy | Pass |
| No parser/topic changes | Pass |
| Additive schema only | Pass |
| GovernanceChrome not structurally redesigned | Pass |
| No ML clustering or autonomous recommendations | Pass |
| Full PLAN-VIZ-R2 (rosbag/Plotly) not implemented | Pass (deferred) |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_cross_sweep_synthesis.py \
  src/counter_uas/test/test_replay_linkage.py \
  src/counter_uas/test/test_replay_publication_export.py \
  src/counter_uas/test/test_replay_research_bundle.py \
  src/counter_uas/test/test_replay_presentation.py \
  src/counter_uas/test/test_replay_storytelling.py -q
python3 scripts/evaluation/gen_e2_research_fixtures.py
python3 scripts/evaluation/build_cross_sweep_synthesis.py --check
python3 scripts/evaluation/export_research_bundle.py --check
python3 scripts/evaluation/sync_sa_catalog.py
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
```

## UX observations

- Cross-sweep synthesis panel collapses by default to avoid overwhelming sweep workstation layout.
- Linkage panel surfaces related sweep families with read-only `?sweep=` links — reduces reviewer fragmentation without implying causal doctrine.
- Publication packets embed numbered figures and citation-style replay refs for print-to-PDF mentor handoff.
- Research bundle zip enables reproducible offline archive of synthesis + sweep reports.

## Limitations

- Static figures are pre-generated in fixtures; CI does not regenerate matplotlib on every run.
- Full PLAN-VIZ-R2 (rosbag trajectory overlays, Plotly) remains planning-only.
- Cross-sweep compare remains descriptive rollups — viewer compare mode stays 2-slot A/B.

## Recommended post-E2 roadmap

- Live `monte_carlo.py` cohort import into sweep workstation
- Full PLAN-VIZ-R2 rosbag/Plotly implementation wave (separate from E2)
- Interactive cross-sweep compare beyond 2-slot A/B
- Automated PDF generation (browser print-to-PDF remains manual)

## Verdict

**Verdict: frozen** for PLAT-SA-E2.
