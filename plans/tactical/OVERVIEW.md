# Tactical / Defense System Layer

## 1. Layer Purpose

The Tactical layer defines simulation policy for threat prioritization, interceptor selection, assignment, engagement gates, feasibility checks, commit/abort behavior, source selection, multi-target coordination, and tactical observability.

This layer is not real military doctrine, weapons authorization logic, classified engagement modeling, ML autonomy, swarm optimization, real-world ROE, or hardware C2 integration.

## 2. Current Tactical State

- `threat_assessment_node.py` publishes distance-based `LOW`, `MEDIUM`, and `HIGH` labels on `/threat_level`.
- `interception_logic_node.py` owns most tactical behavior: source selection, single-target interceptor selection, multi-target assignment, gates, hit legality, metrics, and markers.
- Single-target behavior uses committed selection, TTI margin, switch windows, reacquire/lost timers, and optional lock behavior.
- Multi-target behavior uses configured target labels/topics, per-interceptor `assigned_target`, threat weights, TTI assignment, critical/dive boosts, and assignment stability.
- Engagement gates include dome layer, guidance-start layer, sensing gate, classification placeholder, feasibility gate, rollout gate, MC gate, hit altitude/travel guards, and hit threshold.
- Source selection supports `ground_truth`, `fused`, `tracks`, and `tracks_state` in single-target flow.
- Current operator boundary is observational: launch/config choices, logs, topics, and markers. There is no approval, deny, hold-fire, or override interface.

## 3. Tactical Ownership Boundaries

- Threat assessment: `src/threat_assessment/threat_assessment/threat_assessment_node.py`.
- Target prioritization: `interception_logic_node.py` threat scoring and target ranking.
- Interceptor selection: `interception_logic_node.py` selected ID, TTI comparison, margins, switch windows, and lock behavior.
- Assignment policy: `interception_logic_node.py` multi-target assignment and `/interceptor_i/assigned_target`.
- Feasibility gates: `interception_logic_node.py`, `guidance_lib.py`, `guidance_kernel.py`, `kinematic_plant.py`, and heatmap/rollout helpers.
- Engagement/abort policy: `interception_logic_node.py` for zero commands, selection clearing, stop topics, and impact events; `interceptor_controller_node.py` for idle/standby response.
- Source-selection policy: launch files own defaults; `interception_logic_node.py` owns runtime subscriptions and source interpretation.
- Tactical metrics/logging: parser-visible logs in `interception_logic_node.py` and evaluation parsers.
- Operator-facing tactical state: selected/assigned topics, logs, markers, and future read-only views.

## 4. Planned Tactical Workstreams

- Assignment stabilization:
  - Expected files/modules: `interception_logic_node.py`, `test_selection_audit.py`, future pure assignment fixtures.
  - Risks: hidden tie-break drift, oscillation, single/multi divergence.
  - Validation: selection audit tests and parser-compatible logs.
  - Non-goals: new assignment algorithm or swarm optimization.
- Feasibility-gate stabilization:
  - Expected files/modules: guidance helpers, plant helpers, heatmap validation, `interception_logic_node.py`.
  - Risks: geometric/live rollout mismatch, double gating, false engage/false abort.
  - Validation: guidance snapshots, plant tests, heatmap agreement, failure taxonomy.
  - Non-goals: new guidance modes or aerodynamic feasibility.
- Tactical observability:
  - Expected files/modules: `interception_logic_node.py`, `analyze_run.py`, failure taxonomy scripts.
  - Risks: stdout tag drift, unparsable diagnostics, missing assignment/abort evidence.
  - Validation: parser fixtures and taxonomy tests.
  - Non-goals: UI redesign or event bus.
- Engagement-state formalization:
  - Expected files/modules: tactical docs, parser fixtures, future pure state model.
  - Risks: commit, hold, lost, abort, hit, and standby remain implicit.
  - Validation: state-transition fixtures before runtime refactor.
  - Non-goals: behavior change in the overview.
- Abort/commit semantics:
  - Expected files/modules: selection clearing, assigned-target clearing, zero command, stop topics, controller idle behavior.
  - Risks: abort mistaken for hit stop or post-impact hide.
  - Validation: synthetic logs and focused controller tests.
  - Non-goals: real flight termination.
- Multi-target coordination:
  - Expected files/modules: multi-target launch, assignment policy, target topics, stop topics.
  - Risks: Point-only target state, unclear track identity, assignment stale state.
  - Validation: assignment fixtures and multi-target smoke evidence.
  - Non-goals: full track-list contract redesign.
- Operator-boundary preparation:
  - Expected files/modules: selected/assigned topics, logs, markers, future HITL/C2 docs.
  - Risks: UI or operator display becomes authoritative by accident.
  - Validation: read-only contract review.
  - Non-goals: operator command channel.
- Tactical replay/reporting:
  - Expected files/modules: run capture, parser scripts, failure summaries.
  - Risks: missing tactical evidence in MC artifacts.
  - Validation: parser and report fixture tests.
  - Non-goals: statistical redesign.

## 5. Explicit Non-Goals

- No real military doctrine.
- No weapons authorization logic.
- No classified engagement models.
- No ML tactical autonomy.
- No swarm optimization.
- No real-world ROE claims.
- No hardware C2 integration.
- No guidance or launch-default redesign.

## 6. Tactical Philosophy

Tactically credible means decisions are deterministic, observable, replayable, and consistent with the stated simulation assumptions. Heuristics are acceptable when labeled as heuristics and covered by tests or replay evidence.

Tactical decisions should be separable from guidance math, plant execution, UI display, and evaluation parsing. Live and offline comparisons should preserve source, assignment, feasibility, and gate evidence.

## 7. Tactical Contracts

- `selected_id` is the single-target selected-interceptor signal.
- `/<interceptor_id>/assigned_target` is the multi-target assignment signal.
- Source selector values are `ground_truth`, `fused`, `tracks`, and `tracks_state` where supported.
- Tactical logs must remain parser-compatible when used by evaluation.
- Feasibility state must record enough reason/margin evidence for replay.
- Multi-target labels, position topics, stop topics, and assignment logs are part of the tactical contract.
- UI markers are displays, not tactical truth.

## 8. AI-Agent Guidance

- Read `ARCHITECTURE_MAP.md`, tactical plan docs, and launch files before editing tactical behavior.
- Treat `interception_logic_node.py` as a high-risk hotspot; prefer pure helpers and narrow tests.
- Do not change assignment or gate defaults casually.
- Keep single-target and multi-target semantics explicit.
- Preserve parser-visible tags and MC artifacts.
- Run guidance snapshots, plant tests, selection audit, parser tests, heatmap agreement, and failure taxonomy tests when tactical behavior changes.

## 9. Tactical Hotspots

- `interception_logic_node.py` owns too many tactical, guidance, hit, logging, and visualization responsibilities.
- Assignment tie-breaks and stability can drift without focused fixtures.
- Single-target and multi-target paths use different source assumptions.
- Point vs `tracks_state` inconsistency affects tactical evidence.
- Feasibility and live plant behavior can diverge.
- Launch defaults strongly shape tactical outcomes.
- Tactical logs can become stale or unparsable.
- Operator boundary is not yet explicit.

## 10. Unresolved Tactical Ambiguities

- Abort semantics are not formally defined.
- Assignment tie-break rules need clearer contract tests.
- Tactical replay evidence is incomplete for some gate decisions.
- Operator boundary and future approval flow remain separate planning work.
- Multi-target track identity is weak with Point topics.
- Feasibility thresholds and MC gate thresholds are not fully policy-stabilized.

## 11. Recommended First Tactical Implementation Candidate

Start with an assignment and engagement-gate observability pass: document the selected/assigned target state, feasibility reason fields, and parser-visible evidence without changing selection, guidance, or launch behavior.
