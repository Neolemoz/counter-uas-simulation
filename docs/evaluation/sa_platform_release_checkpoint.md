# SA Platform Release Checkpoint

**Branch:** `codex/replay-narrative-tooling-r1`  
**Checkpoint commits:** through `b51149f` (Post-STAB hygiene)  
**Status:** merge-ready platform checkpoint — documentation and integrity only; no new platform wave.

`AGENTS.md` remains primary governance authority.

## Architecture maturity summary

The platform frontier is frozen through **PLAT-SA-STAB** plus a post-STAB hygiene pass. Maturity layers:

1. **Static replay review** — observability, narrative, comprehension HTML (EVAL-DEMO-R1, EVAL-VIZ-*)
2. **SA-R0 replay platform** — `replay_sa_bundle_v1` + read-only Cesium viewer
3. **Scenario & compare** — topology packs (C1a/C1b), side-by-side compare (D1)
4. **Sweep analytics** — MC sweep manifests, spatial overlays, narrative workstation (D2/D3)
5. **Presentation & synthesis** — storyboards, publication packets, cross-sweep rollups, research bundles (E1/E2)
6. **Integrity gate** — centralized audit, governance batch lint, `tier0-sa-r0` CI (STAB + hygiene)

Runtime research frontier (realism/lifecycle) remains **separate** and bounded — not merged with platform changes on this branch.

## Frozen wave inventory

| ID | Wave | Audit |
|----|------|-------|
| PLAT-SA-R0 | Replay platform + viewer | [sa_r0_freeze_audit.md](sa_r0_freeze_audit.md) |
| PLAT-SA-B1 | Geometry replay | [sa_b1_geometry_freeze_audit.md](sa_b1_geometry_freeze_audit.md) |
| PLAT-SA-B2 | Rich scenario packs | [sa_b2_rich_scenario_freeze_audit.md](sa_b2_rich_scenario_freeze_audit.md) |
| PLAT-SA-C1a | Scenario schema | [sa_c1a_scenario_schema_freeze_audit.md](sa_c1a_scenario_schema_freeze_audit.md) |
| PLAT-SA-C1b | Authoring refinement | [sa_c1b_scenario_authoring_refinement_freeze_audit.md](sa_c1b_scenario_authoring_refinement_freeze_audit.md) |
| PLAT-SA-D1 | Comparative replay | [sa_d1_comparative_replay_freeze_audit.md](sa_d1_comparative_replay_freeze_audit.md) |
| PLAT-SA-D2 | MC spatial analytics | [sa_d2_monte_carlo_spatial_analytics_freeze_audit.md](sa_d2_monte_carlo_spatial_analytics_freeze_audit.md) |
| PLAT-SA-D3 | Narrative intelligence | [sa_d3_replay_narrative_intelligence_freeze_audit.md](sa_d3_replay_narrative_intelligence_freeze_audit.md) |
| PLAT-SA-E1 | Research presentation | [sa_e1_research_presentation_freeze_audit.md](sa_e1_research_presentation_freeze_audit.md) |
| PLAT-SA-E2 | Knowledge synthesis | [sa_e2_replay_knowledge_synthesis_freeze_audit.md](sa_e2_replay_knowledge_synthesis_freeze_audit.md) |
| PLAT-SA-STAB | Platform stabilization | [sa_stabilization_freeze_audit.md](sa_stabilization_freeze_audit.md) |

Full index: [freeze_registry_r1.md](freeze_registry_r1.md).

## Three-tier artifact model

| Tier | Path | Role |
|------|------|------|
| Source | `fixtures/sa_r0/` | Committed source of truth |
| Viewer mirror | `platform/sa-r0-viewer/public/demo/` | Interactive review (`npm run dev`) |
| Offline archive | `fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/` | Portable zip for mentor handoff |

Regeneration: [sa_platform_maintainer_checklist.md](sa_platform_maintainer_checklist.md). Reviewer entry: [sa_r0_reviewer_quickstart.md](sa_r0_reviewer_quickstart.md).

## Governance boundaries preserved

- Explanatory-only replay semantics; mirrors ≠ authority
- No parser/topic/schema changes on this checkpoint
- No HITL, readiness, operational, or WebSocket expansion
- No GovernanceChrome redesign; no ML recommendation/ranking systems
- Additive-only evolution; freeze-before-expansion discipline maintained

## Validation results (release gate)

Run from repository root:

```bash
python3 -m pytest src/counter_uas/test/ -q --tb=short
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```

**Checkpoint run:** all gates green (200 pytest; 36 viewer tests; 12 integrity checks OK; narrative-duplicate warnings only).

## Remaining intentional limitations

- Duplicate narrative bullets across sweeps: **warnings only** (synthetic fixture template copy)
- Pattern tag recompute drift: **warnings only**
- PNG/binary asset byte parity: not enforced on every CI run
- Matplotlib figure regen: maintainer-driven, not CI-on-every-run
- Cross-sweep compare: descriptive rollups only; viewer compare stays 2-slot A/B
- PLAN-VIZ-R2 (rosbag/Plotly): planning-only, not implemented
- PDF export: manual browser print-to-PDF

## Merge readiness

- Working tree clean after checkpoint verification
- Branch pushed to `origin/codex/replay-narrative-tooling-r1`
- Draft PR: update description with this checkpoint summary before merge to `main`
- Post-merge: no new platform frontier until explicit governance review
