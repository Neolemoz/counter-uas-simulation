# Layer B: Simulation Realism

## 1. Layer Purpose

Layer B defines simulation realism boundaries for the Counter-UAS testbed. It keeps perception, tracking, timing, plant/Gazebo behavior, scenario assumptions, and offline rollout assumptions aligned with the current kinematic architecture.

Layer B does not make the simulation aerodynamically, operationally, or hardware realistic. Its job is to make assumptions explicit, keep live/offline behavior from silently drifting, and define validation expectations before realism features are added.

## 2. Current Realism State

- The stack is a kinematic ROS/Gazebo simulation, not a flight-dynamics simulator.
- Radar and camera publish simplified `geometry_msgs/Point` detections with range/FOV/noise/dropout-style knobs.
- Fusion publishes a single Point stream by default; stamped-input paths exist but are not the default full-stack contract.
- Tracking publishes `/tracks` as Point and `/tracks/state` as `nav_msgs/Odometry`; `tracks_state` is the preferred full-stack validation source.
- `interception_logic_node.py` supports `ground_truth`, `fused`, `tracks`, and `tracks_state` source selection.
- `kinematic_plant.py` owns deterministic point-mass plant stepping with command timeout, delay, smoothing, turn/accel limiting, speed clamp, and Euler integration.
- `interceptor_controller_node.py` applies the plant model and uses Gazebo `set_pose`; Gazebo is a pose sink, not physics truth.
- Offline heatmaps and validation scripts reuse some shared math/plant assumptions, but launch/default parity still needs explicit checks.

## 3. Realism Ownership Boundaries

- Perception realism: `radar_sim_node.py`, `camera_sim_node.py`, `noisy_measurement_node.py`, and related config.
- Tracking realism: `tracking_node.py`, `/tracks`, `/tracks/state`, and tracking tests.
- Source-selection realism: `interception_logic_node.py` and launch defaults.
- Plant realism: `kinematic_plant.py` and `interceptor_controller_node.py`.
- Timing/delay realism: sensor delay knobs, measurement delay compensation, plant `cmd_delay_s`, controller timeout, and run metadata.
- Rollout realism: heatmap and MC rollout paths in `interception_logic_node.py`, `render_intercept_heatmap_prob_offline.py`, and `validate_heatmap_vs_gazebo.py`.
- Launch/config realism: `bringup.launch.py`, `gazebo_target.launch.py`, `gazebo_target_multi.launch.py`, and config YAML.
- Offline evaluation realism: `run_capture.py`, `monte_carlo.py`, `scripts/evaluation/*`, and parser-visible artifacts.

## 4. Planned Realism Workstreams

- Measurement realism:
  - Expected files/modules: radar, camera, noisy measurement, fusion, config YAML.
  - Risks: Point paths discard stamp, velocity, covariance, and identity; sensor delay may be lost downstream.
  - Validation: fusion pair-up tests, config audit, stamped/unstamped path checks.
  - Non-goals: RF/EO physics, photorealistic sensors, classifier realism.
- `tracks_state`-first validation:
  - Expected files/modules: `tracking_node.py`, `interception_logic_node.py`, `bringup.launch.py`.
  - Risks: Point/Odometry confusion and finite-difference velocity fallback.
  - Validation: tracking candidate tests, guidance snapshots, launch default audit.
  - Non-goals: multi-object tracker redesign.
- Delay/smoothing parity:
  - Expected files/modules: sensor delay knobs, guidance delay compensation, `kinematic_plant.py`, `interceptor_controller_node.py`.
  - Risks: double compensation, live/offline tau or delay mismatch.
  - Validation: plant tests, guidance snapshots, launch arg audit.
  - Non-goals: PX4/autopilot integration.
- Rollout/plant parity:
  - Expected files/modules: `kinematic_plant.py`, heatmap scripts, validation scripts, launch files.
  - Risks: offline limits diverge from live launch values; Gazebo pose sink mistaken for physics truth.
  - Validation: `test_kinematic_plant.py`, `test_heatmap_agreement.py`, matched Gazebo spot checks.
  - Non-goals: aerodynamic plant.
- Launch default verification:
  - Expected files/modules: full-stack, standalone single-target, and multi-target launch files.
  - Risks: hidden default drift and source-selector mismatch.
  - Validation: grep/default audit and lightweight launch/import smoke when available.
  - Non-goals: changing defaults from this layer.
- Scenario realism:
  - Expected files/modules: target controller, world sim, scenario docs/configs, launch presets.
  - Risks: scenario claims exceed kinematic assumptions or offline mirrors stale values.
  - Validation: scenario metadata, cohort tags, matched launch args.
  - Non-goals: operational ROE or real mission modeling.
- Offline/live parity checks:
  - Expected files/modules: run capture, Monte Carlo, heatmap, statistical validation, parser scripts.
  - Risks: parser drift, mixed cohorts, dry-run mistaken for validation.
  - Validation: parser tests, MC tests, stats tests, heatmap agreement tests.
  - Non-goals: Monte Carlo redesign.

## 5. Explicit Non-Goals

- No CFD, aerodynamics, wind, propulsion, or 6-DoF vehicle model.
- No military RF modeling or photorealistic sensing.
- No PX4/SITL or hardware integration.
- No ML autonomy or classifier implementation.
- No UI redesign.
- No launch-default changes.
- No claims of field readiness or real-world effectiveness.

## 6. Validation Philosophy

Good-enough realism means the simulation clearly states its assumptions and uses the same assumptions in live runs, offline rollouts, heatmaps, and reports. Deterministic kernels and seeded stochastic runs should remain replayable. Approximation is acceptable when named, bounded, and not presented as hardware or operational truth.

Launch/default drift should be treated as a contract change. Any change to topic types, source defaults, plant limits, delay knobs, parser-visible fields, or heatmap assumptions must update the relevant docs and targeted tests.

## 7. Layer B Contracts

- `tracks_state` is the preferred full-stack validation input when velocity/covariance matter.
- Point topics are position-only and must not be treated as stamped track state.
- Launch defaults in `bringup.launch.py`, `gazebo_target.launch.py`, and `gazebo_target_multi.launch.py` are realism contracts.
- `KinematicPlantParams`, `PlantCommand`, and `PlantStepResult` define the current plant model boundary.
- Heatmap and MC artifacts must record enough launch/seed/cohort metadata to explain their realism envelope.
- Parser-visible tags and CSV/meta schemas must stay compatible unless an explicit migration is planned.

## 8. AI-Agent Guidance

- Read `ARCHITECTURE_MAP.md` before editing realism-related files.
- Identify the launch entrypoint before reasoning about defaults.
- Prefer `tracks_state` for full-stack realism validation unless deliberately testing legacy Point paths.
- Do not casually edit `interception_logic_node.py`; isolate pure helpers or tests where possible.
- Do not change launch defaults without updating docs, tests, and evaluation assumptions.
- Before commits that touch realism behavior, run the relevant sensor/fusion/tracking, guidance, plant, parser, and heatmap tests.

## 9. Realism Hotspots

- Point detections discard velocity, stamp, covariance, and identity.
- Multi-target flow still uses per-target Point topics rather than a richer track-state list.
- Sensor delay can be lost when downstream contracts remain unstamped.
- Bringup, standalone single-target, and multi-target launches intentionally differ.
- Offline heatmap credibility depends on live/default parity for turn-rate, acceleration, tau, delay, target motion, and source selection.
- `interception_logic_node.py` spans source selection, guidance calls, policy, metrics, and markers.
- Parser tags and MC CSV/meta fields are evaluation contracts.

## 10. Unresolved Realism Ambiguities

- No single parser/schema version field is consistently persisted across all evaluation artifacts.
- Formal heatmap acceptance thresholds remain partly heuristic.
- Matched-seed realism campaign thresholds are not yet standardized.
- Stamped fusion exists, but the default stack still carries Point-based simplifications.
- Layer B has not yet defined a single realism tier vocabulary shared by docs, launch, and reports.

## 11. Recommended First Implementation Candidate

Start with a documentation/test-only launch-default and realism-contract audit for the `tracks_state` full-stack path. Verify the source selector, topic names, plant knobs, heatmap rollout assumptions, and parser-visible metadata without changing runtime behavior.
