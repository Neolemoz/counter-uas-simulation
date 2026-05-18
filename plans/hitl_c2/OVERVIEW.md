# Human-in-the-loop / C2 Layer

## 1. Layer Purpose

The HITL/C2 layer defines future simulation boundaries for operator interaction, approval/deny semantics, hold-fire behavior, override concepts, shadow mode, replayable operator evidence, and autonomy-vs-human authority separation.

This layer is a planning boundary only. It does not implement command-and-control networking, real-world weapons authorization, tactical redesign, or guidance changes.

## 2. Current HITL/C2 State

- No dedicated HITL/C2 node, operator approval topic, deny topic, override topic, or hold-fire authority exists.
- `interception_logic_node.py` currently couples autonomy recommendation and actuation through `selected_id`, assigned-target topics, and command publication.
- `interceptor_controller_node.py` treats `/interceptor/selected_id` and `/<interceptor_id>/assigned_target` as arming/standby signals.
- UI and RViz markers are observational.
- Replay evidence is mostly stdout logs, parser outputs, run sidecars, MC artifacts, and selection audit summaries.
- Any current "hold" behavior is internal selection hysteresis or feasibility handling, not operator hold-fire.

## 3. HITL/C2 Ownership Boundaries

- Autonomy recommendation today: `interception_logic_node.py`.
- Command-capable simulation signals today: `/<interceptor_id>/cmd_velocity`, `/interceptor/selected_id`, and `/<interceptor_id>/assigned_target`.
- Controller arming/standby response: `interceptor_controller_node.py`.
- Tactical and safety policy context: tactical and safety/policy planning layers plus current gate logs.
- Operator visibility: UI markers, logs, selected/assigned topics, and evaluation artifacts.
- Replayable evidence: `run_capture.py`, `analyze_run.py`, `monte_carlo.py`, selection audit, failure taxonomy, and sidecars.
- Future C2 authority: not implemented; must be explicitly designed before any UI or operator input affects commands.

## 4. Planned HITL/C2 Workstreams

- Operator boundary definition:
  - Expected files/modules: HITL/C2 docs, tactical docs, safety/policy docs, UI docs.
  - Risks: observability mistaken for authority.
  - Validation: documentation review and contract inventory.
  - Non-goals: command-capable interface.
- Advisory recommendation contract:
  - Expected files/modules: future message/log schema, `interception_logic_node.py` mirror path, parser tests.
  - Risks: advisory event interpreted as actuator command.
  - Validation: parser-visible shadow fixtures.
  - Non-goals: arbitration or behavior changes.
- Approval/deny semantics:
  - Expected files/modules: future authority schema and safety policy docs.
  - Risks: feasibility or tactical selection conflated with human approval.
  - Validation: state-machine fixtures before runtime use.
  - Non-goals: real weapons authorization.
- Hold-fire and override semantics:
  - Expected files/modules: future command-authority contract, controller boundary docs.
  - Risks: hold, abort, standby, and stop conflated.
  - Validation: explicit state/reason matrix before implementation.
  - Non-goals: flight termination or real C2 networking.
- Shadow mode:
  - Expected files/modules: read-only observer, logs, parser fixtures.
  - Risks: shadow output changes live behavior.
  - Validation: no command publication; compare advisory vs autonomy selections.
  - Non-goals: authority gate.
- Replayable operator evidence:
  - Expected files/modules: run sidecars, logs, parser/report scripts.
  - Risks: missing operator reason, timestamp, or source attribution.
  - Validation: schema and parser fixtures.
  - Non-goals: complete audit platform.
- Read-only vs command-capable interfaces:
  - Expected files/modules: UI docs, safety docs, future C2 contract.
  - Risks: frontend writes introduced without authority model.
  - Validation: read-only default review.
  - Non-goals: frontend framework.
- Future C2 integration boundary:
  - Expected files/modules: docs only.
  - Risks: implying operational capability.
  - Validation: language review.
  - Non-goals: external C2 protocol integration.

## 5. Explicit Non-Goals

- No real command-and-control networking.
- No real-world weapons authorization behavior.
- No operator command UI implementation.
- No tactical assignment redesign.
- No guidance behavior changes.
- No launch-default changes.
- No hardware authority.
- No operational C2 capability claims.

## 6. HITL/C2 Philosophy

Operator observability is not operator authority. Tactical selection is not human approval. A future C2 path should first mirror autonomy decisions in shadow mode, record replayable evidence, and only later introduce an explicit authority gate if safety and policy contracts are ready.

Human authority, if simulated later, must be timestamped, source-attributed, reason-coded, parser-visible, and deterministic under replay as far as recorded inputs allow.

## 7. HITL/C2 Contracts

- Current selected/assigned topics are actuator/arming signals, not advisory recommendations.
- Future advisory events must not change command publication.
- Future approval/deny/hold/override events must include timestamp, target/track identity, interceptor identity, decision, reason, source, and authority mode.
- UI remains read-only until command authority contracts exist.
- Parser/report compatibility must be preserved when operator evidence is added.
- Replay artifacts must capture both autonomy recommendation and simulated operator decision when both exist.

## 8. AI-Agent Guidance

- Do not add operator command paths directly to `interception_logic_node.py` without a contract.
- Keep C2 work shadow/read-only first.
- Do not reinterpret `selected_id` or `assigned_target` as advisory without migration.
- Preserve existing parser-visible tags and selection audit behavior.
- Keep UI observational unless a command-capable authority contract is explicitly approved.
- Run parser, selection audit, failure taxonomy, and safety-policy tests for future C2 evidence changes.

## 9. HITL/C2 Hotspots

- `interception_logic_node.py` is the current autonomy and command authority hotspot.
- `selected_id` and `assigned_target` directly affect controller arming.
- UI markers can be mistaken for operator state.
- Track identity is weak on Point-based paths.
- Parser-visible evidence is stdout-driven.
- No explicit authority state machine exists.
- Future C2 terms can overstate real-world capability.

## 10. Unresolved HITL/C2 Ambiguities

- Approval, deny, hold-fire, override, and abort semantics are not yet formalized.
- No operator identity/source model exists.
- No advisory event schema exists.
- No authority mode enum exists.
- Track identity and target identity are insufficient on legacy Point paths.
- Shadow-mode acceptance thresholds are not defined.

## 11. Recommended First HITL/C2 Implementation Candidate

Start with a read-only C2 shadow observer/advisory event contract that mirrors autonomy recommendations, logs reason-coded evidence, and never publishes command-capable topics.
