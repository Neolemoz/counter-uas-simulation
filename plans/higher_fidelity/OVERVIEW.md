# Higher Fidelity Dynamics Layer

## 1. Layer Purpose

The Higher Fidelity Dynamics layer defines future plant-fidelity evolution beyond the current kinematic model: aerodynamic assumptions, actuator lag, turn-rate realism, acceleration realism, autopilot abstraction, plant authority separation, Gazebo/physics boundaries, and live/offline plant parity.

This layer does not replace the current kinematic plant. It prepares boundaries for future fidelity increases without changing guidance semantics, launch defaults, or parser/report compatibility.

## 2. Current Dynamics State

- `kinematic_plant.py` is the current deterministic point-mass plant model.
- `KinematicPlantParams` includes fixed `dt_s`, speed clamp, optional acceleration/turn-rate limits, optional `autopilot_tau_s`, optional legacy EMA, command delay, timeout, and fallback velocity.
- `step_kinematic_plant()` selects a command, applies delay/smoothing/limits, clamps speed, and integrates position with Euler stepping.
- `interceptor_controller_node.py` consumes `cmd_velocity`, calls the shared plant, publishes position, and sends Gazebo `set_pose`.
- Gazebo is currently a pose, visual, and optional contact/corroboration sink, not the source of interceptor flight dynamics.
- `guidance_kernel.py` emits velocity intent; it does not own plant execution.
- `interception_logic_node.py` also applies guidance-side acceleration and turn-rate limiting, so motion constraints exist in more than one layer.

## 3. Higher-Fidelity Ownership Boundaries

- Guidance intent: `guidance_kernel.py` and `guidance_lib.py`.
- Shared plant execution: `kinematic_plant.py`.
- Live plant adapter: `interceptor_controller_node.py`.
- Guidance-side limiting and heatmap rollout assumptions: `interception_logic_node.py`.
- Launch/config dynamics defaults: `gazebo_target.launch.py`, `gazebo_target_multi.launch.py`, and `bringup.launch.py`.
- Offline rollout parity: heatmap and validation scripts.
- Evaluation/replay metadata: run capture, MC outputs, sidecars, parser reports.
- Future SITL/PX4 boundary: documentation-only until explicit interface contracts are designed.

## 4. Planned Higher-Fidelity Workstreams

- Kinematic plant contract freeze:
  - Expected files/modules: `kinematic_plant.py`, plant tests, docs.
  - Risks: behavior drift hidden behind realism language.
  - Validation: `test_kinematic_plant.py`, `test_autopilot_model.py`.
  - Non-goals: changing plant semantics.
- Live/offline plant parity:
  - Expected files/modules: controller, heatmap rollout, validation scripts, launch files.
  - Risks: controller defaults differ from MC/offline rollout defaults.
  - Validation: launch/default audit, heatmap agreement tests.
  - Non-goals: new physics model.
- Actuator/autopilot abstraction:
  - Expected files/modules: `KinematicPlantParams`, controller launch args, tests.
  - Risks: legacy EMA and tau smoothing mixed without clear semantics.
  - Validation: delay/tau/timeout fixtures.
  - Non-goals: real autopilot inner loop.
- Turn-rate/acceleration realism:
  - Expected files/modules: guidance-side limiter, plant limiter, ramp tests.
  - Risks: double limiting or mismatch between guidance intent and plant execution.
  - Validation: `test_accel_limit_ramp.py`, plant tests, guidance snapshots if commands change.
  - Non-goals: aerodynamic load model.
- Gazebo integration boundary:
  - Expected files/modules: controller, Gazebo pose tools, contact corroborator docs.
  - Risks: `set_pose` mistaken for rigid-body dynamics.
  - Validation: adapter smoke only when runtime changes.
  - Non-goals: replacing Gazebo integration now.
- Replay/evaluation metadata:
  - Expected files/modules: run sidecars, MC outputs, evaluation scripts.
  - Risks: dynamics knobs omitted from replay artifacts.
  - Validation: parser/metadata fixtures.
  - Non-goals: statistical redesign.
- SITL/PX4 preparation:
  - Expected files/modules: docs only.
  - Risks: implying present capability.
  - Validation: boundary review.
  - Non-goals: PX4/SITL runtime integration.

## 5. Explicit Non-Goals

- No aerodynamic validation.
- No wind, drag, propulsion, 6-DoF, or CFD model.
- No PX4/SITL integration.
- No hardware adapter.
- No replacement of `kinematic_plant.py`.
- No guidance redesign.
- No launch-default changes.
- No real flight-readiness claims.

## 6. Dynamics Philosophy

Guidance semantics and plant execution must remain separate. Guidance produces velocity intent; the plant decides how that intent is realized under delay, smoothing, limits, speed clamp, and integration.

Higher fidelity should be introduced as explicit, testable tiers. Visualization realism is not physics realism. Gazebo visuals and pose updates do not validate aerodynamic behavior. Deterministic replay remains a core requirement for pure plant and offline rollout behavior.

## 7. Higher-Fidelity Contracts

- `KinematicPlantParams`, `KinematicPlantState`, `KinematicPlantMemory`, `PlantCommand`, and `PlantStepResult` define the current plant API.
- `cmd_velocity` is velocity intent, not an actuator command in hardware terms.
- Launch dynamics knobs are contracts when used in evaluation.
- Offline rollout must document which live plant features it mirrors or omits.
- `plant.use_measured_dt` affects determinism and requires replay justification before use in validation.
- Parser/report schemas must remain compatible when dynamics metadata is added.

## 8. AI-Agent Guidance

- Start plant/dynamics work in `kinematic_plant.py` and tests, then check `interceptor_controller_node.py`.
- Do not change guidance snapshots unless command semantics intentionally change.
- Do not casually edit `interception_logic_node.py` limiters without parity tests.
- Keep launch defaults stable unless explicitly tasked to change them.
- Do not use aerospace language that implies validation beyond the kinematic model.
- Run plant, autopilot, acceleration-ramp, guidance snapshot, heatmap, and parser tests as appropriate.

## 9. Dynamics Hotspots

- Motion constraints exist in both guidance/policy and plant/controller layers.
- Legacy EMA and first-order tau are different smoothing models.
- Plant-side accel/turn limits can be disabled while guidance-side limits remain active.
- `set_pose` can hide the absence of rigid-body physics.
- Offline heatmaps can drift from live plant settings.
- Measured dt can reduce deterministic replay.
- Dynamics claims can easily exceed evidence.

## 10. Unresolved Dynamics Ambiguities

- Whether legacy EMA should remain a long-term behavior or be replaced by tau-only semantics.
- Whether guidance-side and plant-side limiting should remain separate.
- Which artifact is truth for validation: soft geometry, Gazebo contact, or shared rollout.
- How future SITL/PX4 interfaces should map velocity intent to controller commands.
- Which dynamics knobs must be mandatory in replay metadata.

## 11. Recommended First Higher-Fidelity Implementation Candidate

Start with a plant-default parity audit: document live controller defaults, guidance-side limits, heatmap rollout defaults, MC metadata, and test coverage before adding any new dynamics model.
