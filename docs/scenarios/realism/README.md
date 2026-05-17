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

Wave 3 threshold-sensitive lifecycle-activation sweep profiles live in:

- `scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave3.csv`

Those profiles use only the implemented Wave 3 threshold-edge mechanisms:

- bringup-topology sweep support
- cadence-aligned silence bursts
- silence/resume oscillation refinement

Wave 5 threshold-envelope fixtures live in:

- `scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave5.csv`
- `scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave5_recovery_envelope.csv`
- `scripts/evaluation/fixtures/ambiguity_sweep_profiles_wave5_phase_spacing.csv`

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

## Wave 3 frozen status

Wave 3 threshold-sensitive lifecycle activation is frozen as **stable**.

Implemented in Wave 3:

- bringup-topology sweep support
- cadence-aligned silence bursts
- silence/resume oscillation refinement
- Wave 3 threshold-sensitive profile matrix
- matched-seed lifecycle activation sweeps
- regression and governance validation

Empirical conclusion:

- bringup topology was exercised successfully
- fragmentation pressure increased
- lifecycle counters still remained dormant
- tactical / selection visibility may itself be under-driven in the current bringup evaluation topology
- sustained threshold crossing remains unresolved

Frozen boundaries:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware/PX4/MAVLink/HITL assumptions

Key insight:

- propagation refinement alone was insufficient
- lifecycle activation may now be limited by evaluation/topology visibility itself
- additional realism breadth is not yet justified
- current priority remains topology-aware lifecycle observability refinement through existing paths

Next roadmap frontier:

- topology-aware lifecycle observability refinement through the existing
  `bringup.launch.py -> /fused_detections -> tracking_node -> /tracks/state` path
- keep it default-off, replay-safe, parser-safe, and additive-only

Caution:

- do not interpret dormant lifecycle counters or `nan` tactical summary fields as proof of tracker robustness

## Wave 4 passive observability tap

A default-off passive bringup observer can be enabled to emit evidence-only logs from `/tracks/state`
and optional selection evidence. It is additive-only, replay-safe, and does not change tracker,
fusion, or parser contracts.

The same observer can also emit a passive selection-visibility proxy from track-persistence and
track-churn evidence only; it does not assign targets or claim authority.

## Wave 4 frozen status

Wave 4 topology-aware lifecycle observability refinement is frozen as **stable**.

Implemented in Wave 4:

- default-off passive bringup lifecycle observer
- passive selection-visibility proxy from `/tracks/state` continuity evidence
- additive observer-aware evaluation summaries
- regression and governance validation

Empirical conclusion:

- bringup lifecycle and selection visibility are now observable
- evidence remains passive and non-authoritative
- lifecycle activation itself remains unresolved when counters stay dormant

Frozen boundaries:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware/PX4/MAVLink/HITL assumptions

Next roadmap frontier:

- topology-aware lifecycle activation refinement through the existing
  `bringup.launch.py -> /fused_detections -> tracking_node -> /tracks/state` path
- keep it default-off, replay-safe, parser-safe, and additive-only

## Wave 5 frozen status

Wave 5 threshold-crossing lifecycle activation refinement is frozen as **stable**.

Implemented in Wave 5:

- confirmed-track reachability audit through the real bringup topology
- bounded recovery-envelope sweep around validated reachability geometry
- micro phase-spacing sweep around the validated `cycle=7 / gap=4` cadence
- matched-seed recovery / churn / deletion comparisons
- regression and governance validation

Empirical conclusion:

- recovery exists
- recovery is cadence-sensitive
- lifecycle stability is phase-sensitive
- aggressive silence does not monotonically improve recovery
- `cycle=7 gap=4 phase=1` is an instability pocket in the tested region
- `cycle=7 gap=4 phase=3` is the strongest bounded fragmented operating point in the tested region

Safe bounded region:

- confirmed-track geometry:
  - `target_start_x_m:=-1500.0`
  - `target_start_y_m:=0.0`
  - `target_start_z_m:=300.0`
- observer-enabled reachability baseline remains the safest reference point
- if fragmented timing is required, stay near the validated `7/4` cadence family
- prefer `phase=3` over `phase=1` inside that family

Frozen unstable pocket:

- `fragmentation_stagger_cycle_ticks:=7`
- `fragmentation_stagger_gap_ticks:=4`
- `fragmentation_stagger_phase_ticks:=1`

In matched-seed runs this pocket produced the weakest stability in the tested region, including:

- reduced success
- elevated coast and churn
- elevated continuity changes
- elevated track deletions

Boundaries remain unchanged:

- no tracker redesign
- no fusion redesign
- no `/tracks/state` semantic change
- no parser-contract change
- no topic/schema change
- no hardware/PX4/MAVLink/HITL assumptions

Warning:

- do not assume stronger silence improves recovery
- do not interpret additive observability or lifecycle evidence as authority
