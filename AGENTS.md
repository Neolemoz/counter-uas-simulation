# AGENTS.md

## Repository Direction

This repository is evolving into a **governance-aware replay-analysis and autonomy evaluation platform** with a bounded high-fidelity simulation substrate.

Identity in practice:

- **Platform (mature):** parser-safe evaluation, replay observability, static reviewer UX, freeze discipline, demo/review workflows — see [docs/evaluation/freeze_registry_r1.md](docs/evaluation/freeze_registry_r1.md).
- **Simulation substrate (bounded):** ROS 2 / Gazebo counter-UAS stack; realism waves frozen unless explicitly re-opened.

Not toward:

- PX4 integration
- MAVLink integration
- hardware bringup
- HITL/operator workflow systems
- tracker redesign

## Core Philosophy

- additive-only evolution
- replay-safe realism
- parser-safe evolution
- freeze-before-expansion
- governance before implementation
- compatibility-path preservation
- no architecture creep

## Frozen Governance

- mirrors != authority
- explanatory evidence != authoritative state
- replay logs != parser contracts
- realism != hardware readiness
- lifecycle degradation != authority semantics

## Frozen Boundaries

- no tracker redesign
- no fusion redesign
- no MHT/JPDA redesign
- no parser-contract changes
- no topic/schema changes
- no hardware/PX4/MAVLink assumptions
- no HITL/operator semantics

## Runtime Realism Philosophy

- realism refinement stays additive-only
- prefer propagation quality over realism breadth
- preserve existing `/tracks/state` semantics
- preserve compatibility paths
- default-off realism expansion is preferred

## Evaluation Philosophy

- matched-seed evaluation preferred
- parser-visible summaries remain stable
- additive metrics/taxonomies only
- replay annotations are explanatory only
- dormant lifecycle counters are not proof of tracker robustness

## Workflow Discipline

- narrow scoped waves only
- explicit allowed/forbidden scope
- post-wave governance audit required
- regression verification required
- freeze + roadmap update after stable waves

## Current Frontiers

Three frontiers are defined. Do not merge them in a single wave.

### Platform frontier (replay analysis and reviewer UX)

- [Replay Demo & Review Workflow R1](docs/evaluation/replay_demo_review_workflow_r1.md) — mentor/demo replay review (frozen docs)
- [Freeze Registry R1](docs/evaluation/freeze_registry_r1.md) — maintained freeze index
- [Situational Awareness UI Planning R1](docs/evaluation/situational_awareness_ui_planning_r1.md) — docs frozen (PLAN-SA-R1); boundaries only
- [SA-R0 Replay Platform](docs/evaluation/sa_r0_implementation_plan.md) — read-only replay viewer (`platform/sa-r0-viewer/`) + `replay_sa_bundle_v1` packager; see PLAT-SA-R0 in freeze registry
- [SA B1 Geometry Replay](docs/evaluation/sa_b1_geometry_replay_plan.md) — LOS overlays, valley ingress fixtures; see PLAT-SA-B1 in freeze registry
- [SA C1a Scenario Schema](docs/evaluation/scenario_schema_v1.md) — portable `fixtures/scenarios/` topology packs + `validate_scenario.py`; see PLAT-SA-C1a in freeze registry
- [SA B2 Rich Replay Scenario Packs](docs/evaluation/sa_b2_rich_scenario_packs_plan.md) — eight topology packs + synthetic demo bundles; see PLAT-SA-B2 in freeze registry
- [SA C1b Scenario Authoring Refinement](docs/evaluation/sa_c1b_scenario_authoring_refinement_plan.md) — metadata refinement, catalog picker, provenance, comparison foundations; see PLAT-SA-C1b in freeze registry
- [SA D1 Comparative Replay](docs/evaluation/sa_d1_comparative_replay_plan.md) — side-by-side compare mode, topology/outcome diff, sensor experiment packs; see PLAT-SA-D1 in freeze registry
- [SA D2 Monte Carlo Spatial Analytics](docs/evaluation/sa_d2_monte_carlo_spatial_analytics_plan.md) — MC sweep manifests, spatial analytics overlays, sweep catalog, replay variability cognition; see PLAT-SA-D2 in freeze registry
- [SA D3 Replay Narrative Intelligence](docs/evaluation/sa_d3_replay_narrative_intelligence_plan.md) — narrative summaries, cohort workstation, N-slot filmstrip, pattern taxonomy, review exports; see PLAT-SA-D3 in freeze registry
- [SA E1 Research Presentation](docs/evaluation/sa_e1_research_presentation_plan.md) — presentation mode, guided walkthroughs, storytelling layer, storyboard fixtures; see PLAT-SA-E1 in freeze registry
- [SA E2 Replay Knowledge Synthesis](docs/evaluation/sa_e2_replay_knowledge_synthesis_plan.md) — cross-sweep synthesis, linkage index, publication packets, research bundles; see PLAT-SA-E2 in freeze registry
- [SA Stabilization](docs/evaluation/sa_stabilization_plan.md) — platform integrity audits, fixture parity, governance batch lint; see PLAT-SA-STAB in freeze registry
- [SA F1a Corpus Indexing](docs/evaluation/sa_f1a_corpus_indexing_plan.md) — replay corpus index, structural lineage, release snapshots; see PLAT-SA-F1a in freeze registry
- [SA F1b Corpus Audit Operations](docs/evaluation/sa_f1b_corpus_audit_operations_plan.md) — drift reports, release diffs, regen orchestration, provenance audits; see PLAT-SA-F1b in freeze registry
- [SA F1c Corpus Navigation](docs/evaluation/sa_f1c_corpus_navigation_plan.md) — corpus browser, lineage navigation, drift surfacing, reviewer cognition workflows; see PLAT-SA-F1c in freeze registry
- [SA F1d Long-Horizon Publication](docs/evaluation/sa_f1d_long_horizon_publication_plan.md) — evolution tracking, publication packet, release archive export, chronology panel; see PLAT-SA-F1d in freeze registry
- [SA F2A Multi-Corpus Federation](docs/evaluation/sa_f2a_multi_corpus_federation_plan.md) — federation manifest, cross-corpus indexing, integrity audits, publication collections, recovery continuity cognition (PLAN-SA-F2A + PLAT-SA-F2A, frozen)
- [Sandbox UX H1](docs/platform/h1_sandbox_ux_architecture_plan.md) — workstation information hierarchy (PLAN-SA-H1, docs frozen)
- [Sandbox Workspace H2](docs/evaluation/sa_h2_sandbox_workspace_freeze_audit.md) — segment shell and discover rails (PLAT-SA-H2, frozen)
- [Offline Orchestration H3](docs/platform/h3_offline_experiment_orchestration_plan.md) — CLI queues and read-only mirrors (PLAT-SA-H3, frozen)
- [Workstation Integration H4](docs/platform/h4_sandbox_replay_workstation_integration_plan.md) — end-to-end replay workflow navigation (PLAT-SA-H4, frozen)
- [Presentation UX Polish H5](docs/platform/h5_publication_presentation_ux_polish_plan.md) — publication-grade visual polish and export handoff (PLAT-SA-H5, frozen)
- [Authoring Workstation A1](docs/platform/sa_a1_authoring_workstation_foundations_plan.md) — authoring workflow, manifest, promotion CLI, read-only AUTHORING profile (PLAN-SA-A1 + PLAT-SA-A1, frozen)
- [Authoring Operations A2](docs/platform/sa_a2_authoring_operations_plan.md) — corpus integrity audits, lifecycle retirement, promotion ergonomics, 12-pack backfill (PLAN-SA-A2 + PLAT-SA-A2, frozen)
- [Orchestration Operations I1](docs/platform/sa_i1_orchestration_operations_plan.md) — offline orchestration lifecycle, integrity audits, replay continuity, 12-pack manifest coverage (PLAN-SA-I1 + PLAT-SA-I1, frozen)
- [Async Orchestration I2](docs/platform/sa_i2_async_orchestration_plan.md) — async foundations: fingerprints, worker bookkeeping, integrity audits, read-only viewer cognition (PLAN-SA-I2 + PLAT-SA-I2, frozen)
- [Async Recovery I3](docs/platform/sa_i3_async_recovery_plan.md) — recovery/reconciliation docs, batch review artifacts, recovery audits, read-only recovery cognition (PLAN-SA-I3 + PLAT-SA-I3, frozen)

Forbidden on the platform frontier: live dashboards, HITL/operator semantics, readiness scoring, parser/topic changes, runtime redesign, rosbridge/WebSocket in eval tooling, extending legacy `web/` rosbridge pages.

### Runtime research frontier (realism / lifecycle)

Threshold-sensitive lifecycle activation refinement through the existing:

`/fused_detections -> tracking_node -> /tracks/state`

flow.

Wave 2 status:

- propagation refinement succeeded in Wave 2
- lifecycle counters still remain largely dormant
- current blocker is sustained threshold crossing

Runtime research may proceed only in narrow, default-off, parser-safe waves. It does not authorize replay UI implementation or operational semantics.

### RT Sandbox frontier (interactive runtime — plan-only)

- [RT-S1 Interactive Sandbox](docs/platform/rt_s1_interactive_sandbox_architecture_plan.md) — transient Browser → RT Backend Bridge → ROS2/Gazebo workflows (PLAN-RT-S1, docs frozen); see [rt_s1_freeze_audit.md](docs/evaluation/rt_s1_freeze_audit.md), [rt_s1_architecture_readiness_review_r1.md](docs/evaluation/rt_s1_architecture_readiness_review_r1.md)
- Contracts: [rt_bridge_contract_v1.md](docs/evaluation/rt_bridge_contract_v1.md), [rt_session_lifecycle_v1.md](docs/evaluation/rt_session_lifecycle_v1.md), [rt_sa_export_boundary_v1.md](docs/evaluation/rt_sa_export_boundary_v1.md), [rt_runtime_governance_v1.md](docs/evaluation/rt_runtime_governance_v1.md)
- Roadmap: [rt_roadmap_s2_s6_v1.md](docs/evaluation/rt_roadmap_s2_s6_v1.md) — RT-S2 through RT-S6 waves
- [RT-S2 Bridge Prototype](docs/platform/rt_s2_bridge_prototype_plan.md) — loopback HTTP bridge + session manager + runtime stub (PLAT-RT-S2, frozen); see [rt_s2_freeze_audit.md](docs/evaluation/rt_s2_freeze_audit.md)
- [RT-S3 Entity Sandbox](docs/platform/rt_s3_entity_sandbox_plan.md) — transient entity registry, world state, spawn/move/delete, reset_session (PLAT-RT-S3, frozen); see [rt_s3_freeze_audit.md](docs/evaluation/rt_s3_freeze_audit.md)
- [RT-S4 Telemetry & Visualization](docs/platform/rt_s4_telemetry_visualization_plan.md) — subscribe/unsubscribe telemetry, loopback pull, RT-only CLI viz (PLAT-RT-S4, frozen); see [rt_s4_freeze_audit.md](docs/evaluation/rt_s4_freeze_audit.md)
- [RT-S5 Runtime Capture & Export Boundary](docs/platform/rt_s5_runtime_capture_plan.md) — `capture_session`, RT-only staging artifacts, export-boundary enforcement, maintainer capture CLIs (PLAT-RT-S5, frozen); see [rt_s5_freeze_audit.md](docs/evaluation/rt_s5_freeze_audit.md)
- [RT-S6 Sandbox Templates & Workflows](docs/platform/rt_s6_sandbox_workflow_plan.md) — RT-local templates, multi-step session workflows, workflow audit, `rt_workflow_inspect.py` (PLAT-RT-S6, frozen); see [rt_s6_freeze_audit.md](docs/evaluation/rt_s6_freeze_audit.md)

**Vocabulary:** “RT interactive sandbox” (PLAT-RT-S*) is **not** the SA replay workstation “sandbox” (PLAN-SA-H*) and **not** registry RT-1..7 realism waves.

Forbidden on the RT Sandbox frontier: rosbridge and legacy `web/` extension for RT, browser→ROS direct execution, SA viewer live hooks, federation/orchestration authority from RT sessions, tactical/HITL/C2 semantics, cloud/multi-user infra, production security infra. Local loopback transport to RT bridge only. Stop line after PLAT-RT-S6: no Gazebo/ROS runtime integration or SA replay ingestion from the RT bridge without a new wave audit.
