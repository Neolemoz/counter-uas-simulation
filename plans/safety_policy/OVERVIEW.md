# Safety / Policy Layer

## 1. Layer Purpose

The Safety / Policy layer defines simulation guardrails around engagement behavior: fail-safe behavior, confidence gates, engagement constraints, abort semantics, policy observability, operator boundaries, launch/config assumptions, and replayable policy evidence.

This layer separates guidance correctness from policy approval, feasibility from authorization, and hit detection from engagement legality. It does not define real-world weapons authorization.

## 2. Current Safety/Policy State

- `interception_logic_node.py` owns most policy gates and hit legality checks.
- `interceptor_controller_node.py` starts idle, responds to selected/assigned target changes, falls back on stale command timeout, and resets on sim-clock rewind.
- Classification gating exists, but `classification_placeholder_confidence` is a placeholder and not a classifier.
- Engagement constraints include dome layers, `guidance_start_layer`, `engagement_layer`, hit layer behavior, hit threshold, min target z, min interceptor z, min interceptor travel, sensing gate, tracking delay, feasibility gate, rollout gate, and MC gate.
- Current abort/standby behavior is implicit: zero command, selected-id clear, assigned-target clear, stop topic, post-impact hide, return-to-origin, or sim reset depending on path.
- Policy evidence is primarily stdout/log tags, `ImpactEvent`, run sidecars, and parser outputs.

## 3. Safety/Policy Ownership Boundaries

- Engagement constraints: `interception_logic_node.py`.
- Guidance correctness: `guidance_lib.py`, `guidance_kernel.py`, and guidance tests.
- Plant execution and fail-safe idle behavior: `kinematic_plant.py` and `interceptor_controller_node.py`.
- Launch/config safety assumptions: `bringup.launch.py`, `gazebo_target.launch.py`, `gazebo_target_multi.launch.py`, and config YAML.
- Policy evidence and replay: `run_capture.py`, `analyze_run.py`, `monte_carlo.py`, failure taxonomy scripts, and statistical reports.
- Operator boundary: currently launch/config/log visibility only; no authoritative operator input exists.
- UI boundary: markers and displays are observational, not safety truth.

## 4. Planned Safety/Policy Workstreams

- Policy decision inventory:
  - Expected files/modules: `interception_logic_node.py`, launch files, config YAML, layer docs.
  - Risks: hidden policy gates embedded in guidance/tactical code.
  - Validation: grep parameters/log tags and documentation review.
  - Non-goals: behavior changes or operator controls.
- Fail-safe and standby semantics:
  - Expected files/modules: `interception_logic_node.py`, `interceptor_controller_node.py`, plant tests.
  - Risks: zero command, selection clear, and return-to-origin semantics confused.
  - Validation: focused controller tests and synthetic logs before refactor.
  - Non-goals: real abort or flight termination.
- Confidence-gate stabilization:
  - Expected files/modules: classification placeholder parameters, launch/config, failure taxonomy.
  - Risks: placeholder confidence interpreted as real classifier.
  - Validation: classification-deny log fixtures and parser checks.
  - Non-goals: ML classifier or real target ID.
- Feasibility vs authorization separation:
  - Expected files/modules: guidance helpers, plant helpers, heatmap validation, `interception_logic_node.py`.
  - Risks: geometry feasibility treated as authorization.
  - Validation: guidance snapshots, plant tests, heatmap agreement, taxonomy tests.
  - Non-goals: weapons authorization.
- Engagement legality evidence:
  - Expected files/modules: `ImpactEvent.msg`, hit logs, parser scripts.
  - Risks: `[HIT]` emitted without enough gate/reason evidence.
  - Validation: parser fixtures for allowed and blocked cases.
  - Non-goals: changing hit physics.
- Abort/operator boundary preparation:
  - Expected files/modules: selected/assigned topics, logs, future HITL/C2 docs.
  - Risks: operator display mistaken for authority.
  - Validation: read-only authority-boundary review.
  - Non-goals: command-capable C2.
- Safety replay/reporting:
  - Expected files/modules: run capture, analyzer, failure taxonomy, statistics.
  - Risks: policy evidence omitted from MC artifacts.
  - Validation: parser and report fixture tests.
  - Non-goals: statistical redesign.

## 5. Explicit Non-Goals

- No real-world weapons authorization.
- No operational ROE implementation.
- No classified safety model.
- No hardware command authority.
- No guidance redesign.
- No tactical assignment redesign.
- No launch-default changes.
- No UI authority.

## 6. Safety/Policy Philosophy

Safety and policy behavior must be observable, replayable, and separable from guidance math. A feasible intercept is not automatically an approved engagement. A detected soft hit is not automatically legal engagement evidence.

Policy defaults should fail in documented ways. Any future command-capable interface must define hold, abort, timeout, confidence, and authority contracts before it can affect actuator signals.

## 7. Safety/Policy Contracts

- Classification placeholder values are simulation policy knobs, not classifier evidence.
- Engagement layers and dome parameters are launch/config policy contracts.
- HIT legality depends on policy gates, geometry, altitude/travel guards, and parser-visible evidence.
- `ImpactEvent` is a soft-hit event, not weapons authorization.
- Stop topics are simulation termination/effect signals.
- Logs and sidecars must preserve policy reasons when used by evaluation.
- UI markers must remain observational.

## 8. AI-Agent Guidance

- Do not treat guidance validity as policy approval.
- Do not treat feasibility or heatmap probability as authorization.
- Keep safety changes narrow and parser-compatible.
- Avoid editing `interception_logic_node.py` casually; it is a cross-layer hotspot.
- Preserve launch defaults unless explicitly tasked to change them.
- Run guidance, plant, parser, failure taxonomy, and heatmap tests for safety/policy behavior changes.

## 9. Safety/Policy Hotspots

- `interception_logic_node.py` combines policy gates, guidance calls, hit logic, metrics, and markers.
- Classification gating is placeholder-based.
- Abort/standby semantics are implicit and path-dependent.
- HIT, stop, impact, hide, and pause behaviors can be conflated.
- Parser-visible safety evidence can drift from runtime decisions.
- Launch presets shape policy but are easy to overlook.
- UI displays can be mistaken for safety truth.

## 10. Unresolved Safety/Policy Ambiguities

- Abort semantics are not formally defined.
- Confidence-gate behavior is not tied to a real classifier contract.
- Policy reason schemas are incomplete.
- Hit legality versus hit detection needs clearer replay evidence.
- Operator boundary is not command-capable and awaits HITL/C2 design.
- Safety thresholds lack a single acceptance document.

## 11. Recommended First Safety/Policy Implementation Candidate

Start with a policy decision inventory and evidence map: list each gate, parameter, log tag, and parser field that affects engagement legality without changing runtime behavior.
