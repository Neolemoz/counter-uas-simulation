# SA E2 — Replay Knowledge Synthesis & Research Publication Workflow (PLAT-SA-E2)

**Status:** implementation wave (additive to PLAT-SA-E1)

## Goal

Evolve the replay presentation platform into a governance-safe **knowledge synthesis and research publication** layer: cross-sweep rollups, rule-based linkage, publication-grade static artifacts, and portable research bundles — all deterministic and replay-local.

## Allowed

- Cross-sweep synthesis scripts under `scripts/evaluation/`
- `replay_cross_sweep_synthesis_v1`, `replay_linkage_v1`, `replay_cognition_rollup_v1`, `replay_research_bundle_v1`, `replay_publication_packet_v1` schemas
- Static figure integration (EVAL-VIZ-E2 slice) from sweep spatial aggregates
- Print-optimized HTML/MD publication packets
- Read-only synthesis/linkage panels in `platform/sa-r0-viewer/`
- Portable research bundle zip export
- Tests, `tier0-sa-r0`, freeze audit

## Forbidden

- WebSocket / rosbridge / live ROS integration
- ML clustering, semantic AI reasoning, causal certainty claims
- Tactical recommendations, deployment readiness, operational planning semantics
- Realtime collaboration services, cloud hosting
- Full PLAN-VIZ-R2 (rosbag overlays, Plotly) — deferred
- Parser/topic contract changes
- GovernanceChrome structural redesign

## Deliverables

| ID | Deliverable |
|----|-------------|
| E2.1 | Schema docs (synthesis, linkage, cognition, research bundle, publication packet) |
| E2.2 | `build_cross_sweep_synthesis.py` + cross-sweep rollups |
| E2.3 | `build_replay_linkage.py` rule-based linkage index |
| E2.4 | `build_replay_cognition_rollup.py` reviewer cognition summaries |
| E2.5 | `replay_viz_sweep_figures.py` static sweep figures |
| E2.6 | Publication HTML/MD extensions + `replay_publication_html.py` |
| E2.7 | `gen_presentation_assets.py` thumbnail/card generation |
| E2.8 | `export_research_bundle.py` portable corpus bundles |
| E2.9 | Viewer synthesis panels + cross-sweep storyboard deck |
| E2.10 | `gen_e2_research_fixtures.py`, CI, freeze audit, registry |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_cross_sweep_synthesis.py \
  src/counter_uas/test/test_replay_linkage.py \
  src/counter_uas/test/test_replay_publication_export.py \
  src/counter_uas/test/test_replay_research_bundle.py \
  src/counter_uas/test/test_replay_presentation.py \
  src/counter_uas/test/test_replay_storytelling.py -q
python3 scripts/evaluation/gen_e2_research_fixtures.py
python3 scripts/evaluation/sync_sa_catalog.py
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
```

## Related docs

- [replay_cross_sweep_synthesis_v1.md](replay_cross_sweep_synthesis_v1.md)
- [replay_linkage_v1.md](replay_linkage_v1.md)
- [replay_cognition_rollup_v1.md](replay_cognition_rollup_v1.md)
- [replay_research_bundle_v1.md](replay_research_bundle_v1.md)
- [replay_publication_packet_v1.md](replay_publication_packet_v1.md)
- [sa_e1_research_presentation_plan.md](sa_e1_research_presentation_plan.md)
