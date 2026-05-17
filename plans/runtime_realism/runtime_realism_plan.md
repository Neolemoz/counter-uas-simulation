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

## Wave 3 freeze checkpoint

Completed and frozen:

- bringup-topology sweep support
- cadence-aligned silence bursts
- silence/resume oscillation refinement
- Wave 3 threshold-sensitive profile matrix
- matched-seed lifecycle activation sweeps
- regression and governance validation

Validation status:

- stable

Current conclusion:

- bringup topology was exercised successfully
- fragmentation pressure increased
- lifecycle counters still remained dormant
- tactical / selection visibility may itself be under-driven in the current bringup evaluation topology
- sustained threshold crossing remains unresolved

Key architectural insight:

- propagation refinement alone was insufficient
- lifecycle activation may now be limited by evaluation/topology visibility itself
- additional realism breadth is not yet justified
- current priority remains topology-aware lifecycle observability refinement through existing paths

Next narrow realism frontier:

- topology-aware lifecycle observability refinement through the existing
  `bringup.launch.py -> /fused_detections -> tracking_node -> /tracks/state` path

Frozen boundaries for that next frontier:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware or HITL assumptions
- do not overinterpret dormant lifecycle metrics or `nan` tactical summary fields as proof of tracker robustness

## Wave 4 freeze checkpoint

Completed and frozen:

- default-off passive bringup lifecycle observer
- passive selection-visibility proxy
- additive observer-aware evaluation summaries
- regression and governance validation

Validation status:

- stable

Current conclusion:

- bringup lifecycle and selection visibility are now observable
- evidence remains passive and non-authoritative
- lifecycle activation itself remains unresolved when counters stay dormant

Key architectural insight:

- bringup visibility is now better instrumented
- this did not change tracker or fusion behavior
- additional realism breadth is not yet justified

## Wave 5 freeze checkpoint

Completed and frozen:

- confirmed-track reachability profile in the real bringup topology
- bounded recovery-envelope sweep around confirmed-track geometry
- micro phase-spacing sweep around the validated `cycle=7 / gap=4` cadence
- matched-seed recovery / churn / deletion comparison
- regression and governance validation

Validation status:

- stable

Current conclusion:

- recovery exists in the current tracker architecture once confirmed-track reachability is present
- recovery is cadence-sensitive
- lifecycle stability is phase-sensitive
- aggressive silence does not monotonically improve recovery
- `fragmentation_stagger_cycle_ticks:=7` / `fragmentation_stagger_gap_ticks:=4` with `fragmentation_stagger_phase_ticks:=1` is an instability pocket
- `fragmentation_stagger_cycle_ticks:=7` / `fragmentation_stagger_gap_ticks:=4` with `fragmentation_stagger_phase_ticks:=3` is the strongest bounded fragmented operating point in the tested region

Key architectural insight:

- the main remaining frontier is no longer "can recovery happen" — it can
- the real constraint is the narrow operating envelope between useful recovery activation and excessive churn / deletion
- additional realism breadth is still not justified

Safe bounded experimental region:

- confirmed-track bringup geometry:
  - `target_start_x_m:=-1500.0`
  - `target_start_y_m:=0.0`
  - `target_start_z_m:=300.0`
- default observer-enabled reachability baseline
- fragmented timing experiments should stay near the validated `7/4` cadence family and avoid `phase=1`
- use `phase=3` as the preferred bounded fragmented reference point inside the tested region

Frozen unstable pocket:

- `cycle=7 gap=4 phase=1` showed the worst matched-seed stability in the tested region:
  - success dropped to `50%`
  - coast, churn, continuity changes, and deletions all increased materially

Next narrow realism frontier:

- bounded cadence / phase refinement inside the validated reachability geometry
- preserve matched-seed evidence and replay-safe observability
- do not widen realism scope or modify tracker / fusion logic

Frozen boundaries for that next frontier:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware or HITL assumptions
- do not reinterpret additive observability as authority
- current priority remains topology-aware lifecycle activation refinement through existing paths

Next narrow realism frontier:

- topology-aware lifecycle activation refinement through the existing
  `bringup.launch.py -> /fused_detections -> tracking_node -> /tracks/state` path

Frozen boundaries for that next frontier:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware or HITL assumptions
- do not overinterpret dormant lifecycle metrics as proof of tracker robustness

## Wave 5 threshold-crossing audit

Wave 5 is currently focused on determining whether lifecycle dormancy is structural
or threshold-tuning-related.

Allowed direction:

- default-off threshold-pressure refinements only
- cadence/silence tuning through the real bringup path
- additive observability only

Frozen boundaries remain unchanged:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware or HITL assumptions
- do not interpret dormant lifecycle counters as proof of tracker robustness

## Explicit non-goals

- hardware timing fidelity claims
- waveform-level radar realism
- production network/C2 timing models
- operator workflow semantics
