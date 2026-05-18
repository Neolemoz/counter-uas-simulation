# Real Hardware Path Layer

## 1. Layer Purpose

The Real Hardware Path layer defines future simulation-to-hardware transition boundaries: autopilot interface assumptions, onboard/offboard split, telemetry contracts, hardware abstraction, replay from real logs, deterministic replay limits, safety constraints, SITL preparation, and sim-to-real drift risks.

This layer documents future boundaries only. It does not introduce hardware integration, PX4/SITL runtime integration, MAVLink/MAVROS adapters, or guidance/tactical redesign.

## 2. Current Hardware-Readiness State

- The repository is a ROS/Gazebo kinematic simulation, not flight-ready software.
- No PX4, SITL, MAVLink, MAVROS, flight-controller mode, arming, failsafe, or actuator driver exists.
- Current command flow is `interception_logic_node.py` publishing `geometry_msgs/Vector3` `cmd_velocity` plus selected/assigned signals.
- `interceptor_controller_node.py` integrates commands with `kinematic_plant.py` and writes poses through Gazebo `set_pose`.
- `guidance_kernel.py` is a portable deterministic guidance contract, but it is not an autopilot.
- `kinematic_plant.py` is a deterministic point-mass execution model, not hardware truth.
- Replay/evaluation is log and sidecar based, with partial determinism for pure kernels and seeded simulation campaigns.

## 3. Real-Hardware Ownership Boundaries

- Guidance intent: `guidance_kernel.py` and `guidance_lib.py`.
- Simulation plant: `kinematic_plant.py`.
- Current simulation actuator adapter: `interceptor_controller_node.py`.
- Tactical authority and command publication: `interception_logic_node.py`.
- Launch/config defaults: `bringup.launch.py`, `gazebo_target.launch.py`, `gazebo_target_multi.launch.py`.
- Telemetry-like simulation topics: `/tracks/state`, `/tracks`, `/fused_detections`, `/<id>/cmd_velocity`, `/<id>/position`, selected/assigned topics, and `ImpactEvent`.
- Replay/evaluation artifacts: logs, `.meta.json`, MC CSV/JSON, heatmap artifacts, parser outputs.
- Future hardware adapter: not implemented; must be separated from guidance semantics and safety authority.

## 4. Planned Real-Hardware Workstreams

- Simulation-to-real boundary map:
  - Expected files/modules: architecture map, guidance/plant/controller docs, launch contracts.
  - Risks: simulation topics treated as flight-controller contracts.
  - Validation: documentation review.
  - Non-goals: hardware driver.
- Telemetry contract definition:
  - Expected files/modules: future schemas, `/tracks/state` lessons, replay docs.
  - Risks: Point topics lack stamp, covariance, velocity, and IDs.
  - Validation: schema tests before adapter work.
  - Non-goals: real sensor telemetry ingestion.
- Autopilot adapter boundary:
  - Expected files/modules: future adapter docs, guidance command contract, plant contract.
  - Risks: velocity intent mapped to hardware commands without authority/safety checks.
  - Validation: offline adapter fixtures only.
  - Non-goals: PX4/SITL runtime wiring.
- Safety authority gate:
  - Expected files/modules: safety/policy docs, HITL/C2 docs, future command contract.
  - Risks: command-capable adapter before arming/hold/abort/failsafe contracts.
  - Validation: safety state-machine fixtures before runtime use.
  - Non-goals: real weapons authorization.
- Replay from telemetry:
  - Expected files/modules: run sidecars, parser scripts, future telemetry log schema.
  - Risks: real telemetry not replayable without clocks, command echo, health, and environment context.
  - Validation: schema and parser fixtures.
  - Non-goals: full flight-test data system.
- Deterministic replay limits:
  - Expected files/modules: docs, metadata schema, evaluation reports.
  - Risks: hardware variability presented as deterministic.
  - Validation: report language and metadata checks.
  - Non-goals: exact replay of uncontrolled real-world conditions.
- SITL/PX4 preparation:
  - Expected files/modules: docs only.
  - Risks: implying present capability.
  - Validation: boundary review.
  - Non-goals: package, launch, or driver integration.
- Sim-to-real drift audit:
  - Expected files/modules: plant, guidance, telemetry, launch, evaluation docs.
  - Risks: kinematic assumptions exceed hardware capability.
  - Validation: contract checklist.
  - Non-goals: hardware validation claim.

## 5. Explicit Non-Goals

- No hardware integration.
- No PX4/SITL runtime integration.
- No MAVLink/MAVROS adapter.
- No flight readiness or operational capability claim.
- No real-world weapons authorization.
- No guidance/tactical redesign.
- No launch-default changes.
- No replacement of the current kinematic plant.

## 6. Hardware-Transition Philosophy

Simulation truth and hardware truth must be kept separate. In simulation, truth comes from Gazebo pose updates, kinematic plant state, soft hit geometry, logs, and sidecars. In hardware, truth would require stamped telemetry, command acknowledgements, flight-controller state, health/failsafe status, hardware logs, and environment context.

Guidance outputs are intent. A future hardware adapter must own command translation, timing, acceptance, arming mode, safety interlocks, and telemetry feedback without changing guidance semantics.

## 7. Real-Hardware Contracts

- `GuidanceCommand` is portable intent, not a flight-controller command.
- `cmd_velocity` is a simulation command topic, not a hardware-safe command contract.
- `selected_id` and `assigned_target` are simulation authority signals today.
- Future telemetry must be stamped and include state, command echo, health/failsafe state, source, and timing basis.
- Hardware replay artifacts must capture command stream, telemetry stream, configuration, firmware/software identity, clocks, environment notes, and parser-visible outcomes.
- New hardware fields must be additive and schema-tested before report use.

## 8. AI-Agent Guidance

- Do not add hardware drivers, PX4/SITL launch files, or MAVLink code under this planning task.
- Keep guidance, plant, safety, and C2 authority boundaries explicit.
- Do not reinterpret current ROS topics as hardware-ready interfaces.
- Preserve parser/report compatibility when adding future telemetry evidence.
- Avoid language implying flight readiness or operational capability.
- Start future implementation with schemas and offline adapter tests, not actuator commands.

## 9. Hardware-Transition Hotspots

- `interception_logic_node.py` couples selection, policy, command publication, hit logic, metrics, and markers.
- `cmd_velocity`, `selected_id`, and `assigned_target` are command-capable simulation topics.
- Point-based topics are weak telemetry contracts.
- Gazebo `set_pose` is not flight dynamics.
- Real telemetry cannot be deterministic without synchronized clocks and complete logs.
- Safety and HITL/C2 authority boundaries are not implemented.
- Documentation language can overstate capability.

## 10. Unresolved Hardware Ambiguities

- Onboard/offboard split is undefined.
- Future flight-controller command type is undefined.
- Hardware telemetry schema is undefined.
- Arming, hold, abort, failsafe, and geofence/range contracts are undefined.
- SITL/PX4 integration boundary is only conceptual.
- Real telemetry replay requirements are not standardized.
- Sim-to-real acceptance thresholds are not defined.

## 11. Recommended First Real-Hardware Implementation Candidate

Start with a real-hardware path design document and schema inventory: map current ROS simulation topics to required future telemetry/command evidence, define safety authority prerequisites, and keep all work read-only until contracts are approved.
