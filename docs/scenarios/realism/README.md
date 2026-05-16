# Runtime realism scenario overlays

These files define **additive**, **seedable**, replay-safe realism overlay packs for the current
simulation runtime. They are not launch defaults and they do not redefine parser-visible
contracts.

Each overlay is intended to be used through explicit launch arguments or evaluation notes/cohorts.

- deterministic seed discipline is required
- defaults remain legacy-compatible when overlays are not enabled
- `Point` paths remain compatibility surfaces
- `/tracks/state` remains the preferred realism-oriented state path

Current starter overlays:

- `delayed_detection.yaml`
- `burst_dropout.yaml`
- `async_sensor_rates.yaml`
- `ghost_detections.yaml`
- `fragmented_detections.yaml`
- `crossing_targets.yaml`

Wave 1 lifecycle-stress sweep profiles live in:

- `scripts/evaluation/fixtures/ambiguity_sweep_profiles.csv`

Those profiles use only the implemented Wave 1 ambiguity mechanisms:

- near-threshold ghost placement
- staggered fragmentation windows

Wave 2 lifecycle-propagation sweep profiles live in:

- `scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave2.csv`

Those profiles use only the implemented Wave 2 refinement mechanisms:

- sensor-path propagation toggle
- phase-aligned / staggered fragmentation refinement
- near-threshold ghost persistence

## Wave 1 frozen status

Wave 1 tracks-state-centered ambiguity realism is now frozen with a **stable** validation
verdict.

Implemented in Wave 1:

- near-threshold ghost placement
- staggered fragmentation windows
- evaluation-only lifecycle thrash derivation
- Wave 1 ambiguity profile matrix
- replay-safe additive annotation expansion

Frozen non-goals:

- no tracker or fusion redesign
- no `/tracks/state` semantic change
- no topic/schema change
- no parser-contract change
- no hardware, PX4, MAVLink, or HITL/operator assumptions

Empirical conclusion:

- Wave 1 increases runtime ambiguity pressure
- Wave 1 does not yet activate tracker lifecycle counters meaningfully
- current stress still mainly affects noisy-measurement compatibility paths
- the remaining frontier is stronger propagation into `/fused_detections -> tracking_node -> /tracks/state`

Caution:

- do not overinterpret dormant lifecycle counters as proof of tracker robustness
- replay annotations remain explanatory only

## Wave 2 frozen status

Wave 2 lifecycle-propagation refinement is frozen as **stable**.

Implemented in Wave 2:

- sensor-path propagation toggle
- phase-aligned fragmentation refinement
- near-threshold ghost persistence
- Wave 2 lifecycle-pressure profile matrix
- matched-seed lifecycle activation sweeps
- regression and governance validation

Empirical conclusion:

- propagation refinement succeeded partially
- downstream tactical/runtime metrics moved
- lifecycle counters still remained dormant
- the current blocker is sustained threshold crossing
- ambiguity now reaches sensing/fusion/tracking behavior more strongly than Wave 1

Frozen boundaries:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware/PX4/MAVLink/HITL assumptions

Key insight:

- propagation quality mattered more than realism breadth
- additional realism families are not yet justified
- current priority remains lifecycle-threshold activation through existing paths

Next roadmap frontier:

- additive threshold-sensitive lifecycle activation refinement through the existing `/fused_detections -> tracking_node -> /tracks/state` flow
- longer effective silence windows
- phase-sensitive timing refinement
- sustained threshold pressure
- default-off, replay-safe, parser-safe, additive-only
