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

Two frontiers are active. Do not merge them in a single wave.

### Platform frontier (replay analysis and reviewer UX)

Post–Comprehension R1 platform work is **documentation and planning first**:

- [Replay Demo & Review Workflow R1](docs/evaluation/replay_demo_review_workflow_r1.md) — mentor/demo replay review (frozen docs)
- [Freeze Registry R1](docs/evaluation/freeze_registry_r1.md) — maintained freeze index
- [Situational Awareness UI Planning R1](docs/evaluation/situational_awareness_ui_planning_r1.md) — plan-only; no live UI implementation authorized here

Forbidden on the platform frontier: live dashboards, HITL/operator semantics, readiness scoring, parser/topic changes, runtime redesign.

### Runtime research frontier (realism / lifecycle)

Threshold-sensitive lifecycle activation refinement through the existing:

`/fused_detections -> tracking_node -> /tracks/state`

flow.

Wave 2 status:

- propagation refinement succeeded in Wave 2
- lifecycle counters still remain largely dormant
- current blocker is sustained threshold crossing

Runtime research may proceed only in narrow, default-off, parser-safe waves. It does not authorize replay UI implementation or operational semantics.
