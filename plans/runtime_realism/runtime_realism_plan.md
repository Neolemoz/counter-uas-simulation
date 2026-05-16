# Runtime Realism Plan

## Objective

Strengthen the simulation as a runtime autonomy validation platform by adding
realism mechanisms that stress tracking, tactical commitment, and execution
robustness without breaking replay comparability.

## Planning guardrails

- preserve deterministic seeds where possible
- keep additions replay-visible when they affect runtime behavior materially
- prefer additive scenario knobs over silent default changes
- preserve parser-safe evolution

## Workstreams

### 1. Timing realism

- latency injection for detections and track delivery
- asynchronous publication timing across sensor paths
- packet jitter and bounded delivery skew
- stale-state simulation for delayed track consumers

### 2. Detection realism

- dropped detections
- intermittent visibility windows
- false positives and false re-acquisition events
- sensor-specific burst loss patterns

### 3. Track realism

- track fragmentation and re-identification stress
- stale-track linger behavior
- covariance growth or simplified uncertainty-state hooks for evaluation-only use
- divergence between truth motion and consumer-visible track continuity

### 4. Target and environment realism

- target maneuver randomness with seeded reproducibility
- delayed-detection starts
- clutter-like nuisance events at the scenario layer
- environmental uncertainty buckets for repeatable stress cohorts

## Roadmap skeleton

### Near-term

- define realism knobs and scenario metadata fields
- define deterministic seed strategy for timing and dropout paths
- define replay-visible evidence expectations for realism events

### Mid-term

- implement timing and dropout realism in parser-safe additive fashion
- extend scenario packs with realism stress classes
- add evaluation rows and aggregates for realism-induced failure modes

### Validation hooks

- stale-event counts
- fragmentation counts
- delayed-detection onset metrics
- false-positive incidence
- reproducibility checks under fixed seeds

## Wave 1 freeze checkpoint

Completed and frozen:

- delayed/stale detection injection
- burst-dropout realism
- async timing diversity
- ghost / false detections
- fragmentation overlays
- replay-safe realism and ambiguity sweeps
- additive realism and ambiguity taxonomies
- Wave 1 tracks-state-centered ambiguity overlays:
  - near-threshold ghost placement
  - staggered fragmentation windows
  - evaluation-only lifecycle thrash derivation
  - Wave 1 ambiguity profile matrix
  - replay-safe additive annotation expansion

Validation status:

- stable

Current conclusion:

- runtime ambiguity pressure increased
- tracker lifecycle counters remained largely dormant
- current ambiguity stress still acts mainly through noisy-measurement compatibility paths

Next narrow realism frontier:

- additive lifecycle-propagation refinement through the existing `/fused_detections -> tracking_node -> /tracks/state` flow

Frozen boundaries for that next frontier:

- no tracker redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware or HITL assumptions
- do not overinterpret dormant lifecycle metrics as proof of tracker robustness

## Wave 2 freeze checkpoint

Completed and frozen:

- sensor-path propagation toggle
- phase-aligned fragmentation refinement
- near-threshold ghost persistence
- Wave 2 lifecycle-pressure profile matrix
- matched-seed lifecycle activation sweeps
- regression and governance validation

Validation status:

- stable

Current conclusion:

- propagation refinement succeeded partially
- downstream tactical/runtime metrics moved
- lifecycle counters remained dormant
- the current blocker is sustained threshold crossing
- ambiguity now reaches sensing/fusion/tracking behavior more strongly than Wave 1

Key architectural insight:

- propagation quality mattered more than realism breadth
- additional realism families are not yet justified
- current priority remains lifecycle-threshold activation through existing paths

Next narrow realism frontier:

- additive threshold-sensitive lifecycle activation refinement through the existing `/fused_detections -> tracking_node -> /tracks/state` flow

Frozen boundaries for that next frontier:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware or HITL assumptions
- do not overinterpret dormant lifecycle metrics as proof of tracker robustness

## Explicit non-goals

- hardware timing fidelity claims
- waveform-level radar realism
- production network/C2 timing models
- operator workflow semantics
