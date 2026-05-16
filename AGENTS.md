# AGENTS.md

## Repository Direction

This repository is evolving into a **high-fidelity autonomy robustness experimentation platform**.

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

## Current Frontier

Threshold-sensitive lifecycle activation refinement through the existing:

`/fused_detections -> tracking_node -> /tracks/state`

flow.

Wave 2 status:

- propagation refinement succeeded in Wave 2
- lifecycle counters still remain largely dormant
- current blocker is sustained threshold crossing
