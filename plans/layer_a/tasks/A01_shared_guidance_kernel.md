# A01 Shared Guidance Kernel

## Goal

Extract, stabilize, and document the shared guidance-kernel boundary so live Gazebo guidance, offline feasibility tools, and tests use the same deterministic guidance semantics.

Primary contract boundary:

- `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py`
- `src/gazebo_target_sim/gazebo_target_sim/guidance_kernel.py`
- `src/gazebo_target_sim/gazebo_target_sim/kinematic_plant.py`
- `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py`
- `src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py`
- `simulation/core/intercept.py`

Execution scope:

- Layer: Shared Math And Contracts.
- Change type: guidance semantics / contract stabilization.
- Runtime source edits: planned later, not in this planning step.
- Expected first implementation entrypoint: `src/counter_uas/test/test_guidance_kernel_snapshots.py`.
- Layer A overview dependency: `plans/layer_a/OVERVIEW.md` is expected by the planning structure but was not present when this file was expanded.

## Current Architecture State

### Current Ownership Boundaries

- Shared math owns numerical semantics, not ROS lifecycle:
  - `guidance_lib.py` owns CV intercept roots, intercept point, delay compensation, and t_go helpers.
  - `guidance_kernel.py` owns deterministic pairwise command assembly.
  - `kinematic_plant.py` owns velocity-command execution limits and rollout stepping.
- ROS policy owns runtime decisions, not pure math:
  - `interception_logic_node.py` owns measurement-source selection, target/interceptor pairing, assignment, engagement gates, hit legality, metrics, and markers.
  - `interceptor_controller_node.py` owns ROS/Gazebo actuation around `step_kinematic_plant`.
- Offline tools may reuse pure math:
  - `simulation/core/intercept.py` delegates intercept timing to `guidance_lib.py`.
  - Offline tools must not silently fork command-generation semantics.

### Where Guidance Logic Lives

- `guidance_lib.py`: canonical constant-velocity intercept primitives.
  - Owns `solve_intercept_time`, `compute_intercept`, `compensate_target_for_delay`, `align_speed_after_saturation`, `filter_t_go`.
  - Pure Python, stdlib-only, no ROS imports.
- `guidance_kernel.py`: existing ROS-free command kernel.
  - Owns `GuidanceInput`, `GuidanceCommand`, `GuidanceParams`, `GuidanceMemory`, and `compute_guidance_command`.
  - Handles predictive/pursuit blending, terminal pursuit blend, PN-inspired steering vector, unit-command slew, speed/t_go coherence, and memory transition.
- `kinematic_plant.py`: pure plant propagation.
  - Owns command freshness, delay, smoothing, turn-rate limiting, acceleration limiting, speed clamp, and Euler position step.
- `interception_logic_node.py`: live ROS policy and adapter hotspot.
  - Owns target source selection, single/multi target policy, assignment, engagement gates, hit legality, metrics/logging, marker publishing.
  - Wraps `guidance_lib.py` via `_solve_intercept_time` / `_compute_intercept`.
  - Calls `compute_guidance_command` through `_compute_guidance_command_for_pair`.
- `interceptor_controller_node.py`: Gazebo plant adapter.
  - Consumes `/<interceptor_id>/cmd_velocity`.
  - Calls `step_kinematic_plant` and publishes `/<interceptor_id>/position`.
- `simulation/core/intercept.py`: offline feasibility adapter.
  - Dynamically loads canonical `guidance_lib.py` so offline intercept timing matches live numerical tolerance.

### Duplicated Semantics

- Solver semantics appear as canonical helpers in `guidance_lib.py` and live-node wrappers in `interception_logic_node.py`.
- Vector helpers exist in multiple files: `_norm`, `_unit`, `rotate_dir_toward`, speed clamp, and required-speed calculations.
- Command-generation state is split between `GuidanceMemory` and node dictionaries:
  - `_guidance_mode`
  - `_valid_streak`
  - `_invalid_streak`
  - `_t_go_filtered`
  - `_intercept_point_filtered`
  - `_guidance_unit_prev`
- Delay compensation is partly modeled in `guidance_lib.py` / `guidance_kernel.py`, while the live node also manually adjusts target position before calling the kernel.
- Plant limits are pure in `kinematic_plant.py`, but live engagement/heatmap paths can still pass plant defaults separately.

### Live vs Offline Drift Risks

- `simulation/core/intercept.py` delegates timing to `guidance_lib.py`, but not full `GuidanceInput -> GuidanceCommand` behavior.
- Offline heatmap and feasibility paths can match solver timing while still diverging on:
  - command speed schedule
  - t_go filtering
  - terminal pursuit blend
  - PN-inspired steering blend
  - turn-rate / acceleration / delay limits
- `interception_logic_node.py` owns both live command generation and metrics/log schema, so extraction can accidentally change parser-visible fields.
- Launch defaults can hide drift if `bringup.launch.py`, `gazebo_target.launch.py`, and `gazebo_target_multi.launch.py` disagree on source selection or plant limits.

### Single-Target vs Multi-Target Differences

- Single-target flow:
  - Target source selected by `intercept_measurement_source`.
  - Supports `ground_truth`, `fused`, `tracks`, and `tracks_state`.
  - Calls `_run_ram_guidance`, publishes one selected command and zeroes non-selected interceptors.
- Multi-target flow:
  - Uses `multi_target_enabled`, labels, per-target position topics, assignment, and assigned-target topics.
  - Calls the same `_compute_guidance_command_for_pair` but with target/interceptor pairing from multi-target assignment.
  - Metrics and marker publication are similar but not identical to single-target flow.
- Migration must keep the shared kernel pairwise; assignment and hit policy remain outside the kernel.

### Point vs tracks_state Implications

- Point sources (`ground_truth`, `fused`, `tracks`) carry position only.
  - Target velocity must be estimated from position history.
  - No covariance, stamp, or track ID is available.
- `tracks_state` carries `nav_msgs/Odometry`.
  - Provides KF position, velocity, covariance, and `child_frame_id=track_<id>`.
  - Preferred for full-stack validation because guidance can use filter velocity directly.
- Kernel inputs should stay source-agnostic.
  - Source adapters must convert Point/Odometry into explicit `TargetKinematics`.
  - The kernel must not import ROS message types.

## Desired End State

### Pure Guidance Core

- `guidance_lib.py` remains the canonical CV intercept math contract.
- `guidance_kernel.py` becomes the canonical command contract for one target/interceptor pair.
- `kinematic_plant.py` remains the canonical plant-step contract for command execution and rollout.
- The shared kernel must not own:
  - ROS subscriptions/publishers
  - assignment
  - dome/engagement policy
  - hit legality
  - marker publishing
  - log parsing

### Explicit Contracts

- `GuidanceInput` should fully describe command-generation inputs:
  - target position/velocity
  - interceptor position/velocity
  - measurement delay semantics
  - aim-point override
  - params snapshot
  - memory snapshot
- `GuidanceCommand` should fully describe command-generation outputs:
  - velocity command
  - unit command
  - speed command
  - mode and transition
  - solution validity
  - raw/effective `t_go`
  - raw/smoothed intercept point
  - solver speed
  - PN/debug/observability fields
  - next memory
- Node-local adapters should translate between ROS state and these dataclasses.
- Offline adapters should call the same dataclasses when comparing command behavior, not reimplement command assembly.

### Deterministic Command Generation

- Same `GuidanceInput` must produce the same `GuidanceCommand` without ROS, wall time, random state, or global state.
- Memory changes must flow only through `GuidanceMemory -> GuidanceCommand.memory_next`.
- Floating-point tolerances should be pinned by tests, not changed opportunistically.

### Live/Offline Parity Expectations

- Live node command generation should be reproducible by direct calls to `compute_guidance_command`.
- Offline feasibility may keep lightweight timing-only APIs, but any claim about command behavior should use the kernel or explicitly document the approximation.
- Full parity includes:
  - solver tolerance
  - command speed schedule
  - predictive/pursuit mode transitions
  - t_go filtering
  - terminal blend
  - PN-inspired blend
  - unit-command slew
  - plant-step constraints when rollout is used

### Observability Expectations

- Existing parse-compatible logs stay stable:
  - `[METRICS]`
  - `[ENG_METRIC]`
  - `[FEASIBILITY]`
  - `[min_miss]`
  - `[HIT]`
- Kernel outputs should continue to supply enough fields for:
  - `t_go_raw`
  - `t_go_filt`
  - `speed_cmd`
  - command vector
  - mode
  - feasible/rollout gate context from the node
- New observability fields are acceptable only if they are additive and parser-compatible.

### Contract Ownership Boundaries

- `guidance_lib.py`: scalar/vector math and solver tolerance.
- `guidance_kernel.py`: pairwise command semantics and guidance memory transition.
- `kinematic_plant.py`: command execution and rollout dynamics.
- `interception_logic_node.py`: ROS adaptation, policy gates, assignment, logs, markers.
- `interceptor_controller_node.py`: Gazebo pose adapter and live plant stepping.
- `simulation/core/intercept.py`: offline feasibility wrappers over canonical pure helpers.

## Migration Phases

Phase gate rule:

- Do not migrate runtime adapters until snapshot/parity coverage exists.
- Do not delete duplicated node helpers in the same change that moves behavior.
- Do not change launch defaults, guidance tuning values, or parser-visible logs in A01.
- Stop and split a new task if behavior must change to pass validation.

### Phase 1: Parity Snapshot Coverage

Expected files touched:

- `src/counter_uas/test/test_guidance_kernel_snapshots.py`
- `src/counter_uas/test/test_guidance_lib.py`
- `src/counter_uas/test/test_intercept_solver_golden.py`
- Optional targeted fixtures under `src/counter_uas/test/`

Tasks:

- Add table-driven cases for pairwise guidance behavior:
  - head-on predictive
  - no-solution pursuit fallback
  - delayed target measurement
  - Point-style finite-difference velocity input
  - `tracks_state`-style direct velocity input
  - terminal blend
  - PN-inspired blend
  - unit-command slew
  - memory transition across two ticks
- Pin both `GuidanceCommand` fields and `GuidanceMemory` updates.
- Add single-vs-multi adapter parity cases if a minimal node-free adapter exists after extraction.
- Prefer explicit expected values for:
  - `velocity_cmd`
  - `unit_cmd`
  - `speed_cmd_m_s`
  - `mode`
  - `solution_valid`
  - `t_go_raw_s`
  - `t_go_effective_s`
  - `intercept_point_raw`
  - `memory_next`

Expected risks:

- Overfitting snapshots to incidental float formatting.
- Missing memory-state cases that only fail after several ticks.
- Treating source-selection behavior as kernel behavior.

Validation expectations:

- Run:
  - `pytest src/counter_uas/test/test_guidance_lib.py`
  - `pytest src/counter_uas/test/test_intercept_solver_golden.py`
  - `pytest src/counter_uas/test/test_guidance_kernel_snapshots.py`
- Pass condition: new snapshots fail on an intentional command semantic change and pass without touching runtime code.

Rollback boundary:

- Revert only added/changed snapshot tests.
- No runtime behavior changes in this phase.

### Phase 2: Extraction and Contract Tightening

Expected files touched:

- `src/gazebo_target_sim/gazebo_target_sim/guidance_kernel.py`
- `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py`
- Minimal tests from Phase 1

Tasks:

- Move remaining pure command helpers out of `interception_logic_node.py` only when covered by snapshots.
- Keep solver math in `guidance_lib.py`.
- Keep command assembly in `guidance_kernel.py`.
- Avoid importing `kinematic_plant.py` into `guidance_kernel.py` unless command-vs-plant coupling is explicit and tested.
- Decide whether duplicated helpers like `rotate_dir_toward` remain separate or share a single pure helper.
- Define one owner for each semantic before moving code:
  - solver tolerance: `guidance_lib.py`
  - speed schedule: `guidance_kernel.py`
  - t_go filter state transition: `guidance_kernel.py`
  - plant command delay/limits: `kinematic_plant.py`
  - log formatting: `interception_logic_node.py`

Expected risks:

- Accidentally changing solver tolerance or root selection.
- Changing mode transition timing.
- Double-applying measurement delay if live node and kernel both compensate.
- Mixing plant saturation semantics into command generation.

Validation expectations:

- Phase 1 tests remain unchanged and pass.
- Add tests for any moved helper before deleting node-local copies.
- No launch/default changes.
- Grep for duplicate command-speed or solver-tolerance implementations after extraction.

Rollback boundary:

- Revert extraction commit without reverting snapshot characterization.
- Node wrappers can continue calling old helper locations during migration.

### Phase 3: Adapter Migration

Expected files touched:

- `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py`
- `src/gazebo_target_sim/gazebo_target_sim/guidance_kernel.py`
- `src/counter_uas/test/test_guidance_kernel_snapshots.py`
- Optional node adapter tests if practical without ROS spin

Tasks:

- Keep `_compute_guidance_command_for_pair` as the live-node adapter boundary.
- Convert ROS-derived target/interceptor state into `GuidanceInput` once per pair.
- Keep assignment, engagement gating, hit policy, marker publishing, and logs in `interception_logic_node.py`.
- Ensure single-target and multi-target paths both use the same adapter and memory key semantics.
- Remove or quarantine node-local command math only after parity tests pass.
- Make memory keys explicit:
  - single-target key: selected interceptor id unless a target label is introduced.
  - multi-target key: include target label and interceptor id if pair-specific memory is required.
- Keep source adapters explicit:
  - Point path: position plus estimated velocity.
  - `tracks_state` path: Odometry position plus filter velocity.

Expected risks:

- Memory key collisions between single-target and multi-target paths.
- Multi-target metrics overwriting per-interceptor state differently than before.
- Command vector signs or z-axis behavior changing under aim-point override.
- Stale `t_go` memory persisting after no-solution frames.

Validation expectations:

- Run guidance tests from Phase 1.
- Run targeted tests likely to catch adapter regressions:
  - `pytest src/counter_uas/test/test_selection_audit.py`
  - `pytest src/counter_uas/test/test_guidance_kernel_snapshots.py`
- Manual grep/check for `compute_guidance_command` call sites.
- If logs changed at all, also run parser tests before commit.

Rollback boundary:

- Keep old node-local helper wrappers until adapter migration is complete.
- Revert adapter call-site changes without changing `guidance_lib.py`.

### Phase 4: Launch and Config Verification

Expected files touched:

- No source changes required unless verification exposes drift.
- Possible docs-only notes in this plan or `ARCHITECTURE_MAP.md` if contracts changed.

Tasks:

- Verify launch defaults remain unchanged:
  - `src/counter_uas/launch/bringup.launch.py`
  - `src/gazebo_target_sim/launch/gazebo_target.launch.py`
  - `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py`
- Confirm source-selection defaults:
  - bringup default remains `tracks_state`
  - standalone single-target default remains `ground_truth`
  - multi-target default remains `ground_truth`
- Confirm plant limit defaults are unchanged unless a separate tuning task owns the change.
- Verify launch files still forward the same guidance, source, topic, and plant parameters.
- Verify no new required parameter is introduced without a backward-compatible default.

Expected risks:

- Hidden behavior change through renamed parameter, changed default, or missing forwarded launch arg.
- Bringup and standalone Gazebo drifting while tests call only pure Python.

Validation expectations:

- Grep launch files for changed parameter names/defaults.
- Import/syntax smoke for launch files if existing test harness supports it.
- No acceptance based only on pure unit tests.
- Document any found default drift in a separate follow-up, not in A01 implementation.

Rollback boundary:

- Revert launch/config changes independently from kernel extraction.
- If launch defaults must change, stop and create a separate task.

### Phase 5: Cleanup

Expected files touched:

- `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py`
- `src/gazebo_target_sim/gazebo_target_sim/guidance_kernel.py`
- Tests touched in prior phases

Tasks:

- Remove dead node-local command helpers only when no longer referenced.
- Keep trace/log wrappers where they provide live debug value.
- Preserve parse-compatible log tags and fields.
- Avoid broad formatting churn in `interception_logic_node.py`.
- Keep cleanup mechanical:
  - remove unused imports
  - remove unreachable helpers
  - collapse duplicate adapters only after parity tests pass
  - leave unrelated policy code untouched

Expected risks:

- Removing helper code still used by feasibility, debug, heatmap, or marker paths.
- Cleanup diffs becoming too large to review.
- Parser/schema regressions from log cleanup.

Validation expectations:

- Run guidance tests.
- Run parser/evaluation tests if log output changed:
  - `pytest src/counter_uas/test/test_analyze_run_parse.py`
  - `pytest src/counter_uas/test/test_monte_carlo.py`
  - `pytest src/counter_uas/test/test_classify_run.py`
- Run lints on edited Python files before commit.

Rollback boundary:

- Cleanup must be its own commit/PR when possible.
- Revert cleanup without reverting extraction if behavior is stable.

### Phase 6: Validation Freeze

Expected files touched:

- `src/counter_uas/test/test_guidance_kernel_snapshots.py`
- `ARCHITECTURE_MAP.md` only if contracts/defaults changed
- This plan file if follow-up findings need documentation

Tasks:

- Freeze dataclass field meanings and command semantics.
- Record accepted tolerances for snapshot comparisons.
- Mark temporary duplication that remains intentionally.
- Document any known live/offline approximations.
- Update `ARCHITECTURE_MAP.md` only if a contract actually changed.
- Close A01 only when remaining duplication has an owner and a follow-up task.

Expected risks:

- Freezing too early before single/multi parity is proven.
- Treating tests as contract while launch defaults still drift.

Validation expectations:

- Required targeted test subset passes.
- One single-target launch-level sanity check is planned before behavior PR merge.
- One multi-target launch-level sanity check is planned before behavior PR merge if multi-target call sites changed.
- Contract freeze note lists any accepted approximations in offline tooling.

Rollback boundary:

- If freeze exposes mismatch, keep tests and revert implementation.
- Do not loosen snapshots unless the behavior change is intentional and documented.

## Explicit Non-Goals

- No tuning changes.
- No new guidance modes.
- No aerodynamic realism or `P_kill` modeling.
- No PX4/autopilot integration beyond existing kinematic plant contract.
- No UI/RViz redesign.
- No Monte Carlo redesign.
- No change to launch defaults.
- No topic/message schema change.
- No metrics/log schema breaking change.
- No assignment-policy rewrite.
- No hit-legality rewrite.
- No broad refactor of `interception_logic_node.py`.

## Dependencies

- `ARCHITECTURE_MAP.md` guidance/control architecture and preferred edit paths.
- Existing pure math tests:
  - `src/counter_uas/test/test_guidance_lib.py`
  - `src/counter_uas/test/test_intercept_solver_golden.py`
  - `src/counter_uas/test/test_guidance_kernel_snapshots.py`
- Plant tests:
  - `src/counter_uas/test/test_kinematic_plant.py`
  - `src/counter_uas/test/test_autopilot_model.py`
  - `src/counter_uas/test/test_accel_limit_ramp.py`
- Adapter-sensitive tests:
  - `src/counter_uas/test/test_selection_audit.py`
  - `src/counter_uas/test/test_heatmap_agreement.py`
  - parser tests if logs change

## Risks

- `interception_logic_node.py` is a large ownership hotspot; narrow edits are required.
- `tracks_state` can regress silently if velocity falls back to Point finite differencing.
- Multi-target path can diverge from single-target path if adapter memory keys are not explicit.
- Offline tools can match solver time but not command behavior.
- Parser-compatible logs can break through small field-name or tag changes.
- Temporary duplicated helpers can become permanent unless tracked.

## Validation Strategy

### Required Targeted Tests

- Always run for kernel/API changes:
  - `pytest src/counter_uas/test/test_guidance_lib.py`
  - `pytest src/counter_uas/test/test_intercept_solver_golden.py`
  - `pytest src/counter_uas/test/test_guidance_kernel_snapshots.py`
- Run for plant-coupled changes:
  - `pytest src/counter_uas/test/test_kinematic_plant.py`
  - `pytest src/counter_uas/test/test_autopilot_model.py`
  - `pytest src/counter_uas/test/test_accel_limit_ramp.py`
- Run for adapter/log changes:
  - `pytest src/counter_uas/test/test_selection_audit.py`
  - `pytest src/counter_uas/test/test_analyze_run_parse.py`
  - `pytest src/counter_uas/test/test_monte_carlo.py`
  - `pytest src/counter_uas/test/test_classify_run.py`

### Snapshot and Parity Comparisons

- Snapshot `GuidanceCommand` fields, not printed logs.
- Include `memory_next` expectations.
- Compare single-target and multi-target adapter outputs for equivalent target/interceptor pairs when possible.
- Keep tolerances numeric and explicit.
- Do not snapshot wall-clock timing, random output, or marker IDs as kernel behavior.

### Launch-Level Sanity Checks

- Verify launch defaults did not change:
  - `intercept_measurement_source`
  - `tracks_state_topic`
  - `multi_target_enabled`
  - `interceptor_max_speed_m_s`
  - plant acceleration/turn-rate/tau/delay params
- Smoke one single-target path after adapter migration.
- Smoke one multi-target path if multi-target command call sites changed.
- Preserve topic names:
  - `/tracks/state`
  - `/fused_detections`
  - `/<interceptor_id>/cmd_velocity`
  - `/<interceptor_id>/position`

### Live/Offline Comparison Expectations

- `simulation/core/intercept.py` timing should continue to match `guidance_lib.py`.
- Command-behavior comparisons should call `guidance_kernel.py`.
- Heatmap/rollout comparisons should also include `kinematic_plant.py` when plant limits matter.
- Offline approximations must be named as approximations in docs/tests.

### Acceptable Temporary Duplication

- Node-local trace wrappers around `guidance_lib.py` are acceptable.
- Duplicated vector helpers are acceptable during migration if tests pin behavior.
- Old adapter paths may coexist with new dataclass conversion for one phase.
- Temporary duplication must not introduce a second solver tolerance or second command speed schedule.

## Acceptance Criteria

- `guidance_lib.py` remains the canonical intercept-time and intercept-point contract.
- `guidance_kernel.py` owns deterministic `GuidanceInput -> GuidanceCommand` command generation.
- Single-target and multi-target live paths call the same pairwise command boundary.
- `interception_logic_node.py` keeps policy, assignment, hit, marker, and log ownership outside the kernel.
- `simulation/core/intercept.py` still delegates solver timing to `guidance_lib.py`.
- Targeted tests pass for changed surfaces.
- Launch defaults and parser-visible log tags remain unchanged.

## Rollback Considerations

- Keep phases small enough to revert independently.
- Characterization tests should land before extraction so behavior can be restored.
- Extraction can roll back without removing tests.
- Adapter migration can roll back to existing node helper wrappers.
- Cleanup should roll back independently from behavior-preserving extraction.
- Any launch/default change must be split from this task.

## Contract Freeze Expectations

### APIs to Stabilize

- `guidance_lib.py`
  - `solve_intercept_time`
  - `compute_intercept`
  - `compensate_target_for_delay`
  - `align_speed_after_saturation`
  - `filter_t_go`
- `guidance_kernel.py`
  - `TargetKinematics`
  - `InterceptorKinematics`
  - `GuidanceParams`
  - `GuidanceMemory`
  - `GuidanceInput`
  - `GuidanceCommand`
  - `compute_guidance_command`
- `kinematic_plant.py`
  - `KinematicPlantParams`
  - `KinematicPlantState`
  - `KinematicPlantMemory`
  - `PlantCommand`
  - `PlantStepResult`
  - `step_kinematic_plant`

### Launch Defaults to Preserve

- `src/counter_uas/launch/bringup.launch.py`
  - full-stack default source remains `tracks_state`
  - topic forwarding remains compatible with `/tracks/state`
- `src/gazebo_target_sim/launch/gazebo_target.launch.py`
  - standalone default source remains `ground_truth`
- `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py`
  - multi-target default source remains `ground_truth`
- Plant defaults remain unchanged unless a separate tuning task owns them.

### Parse-Compatible Logs and Metrics

- Preserve tags:
  - `[METRICS]`
  - `[ENG_METRIC]`
  - `[FEASIBILITY]`
  - `[min_miss]`
  - `[HIT]`
  - `[ENG_GATE]`
- Preserve core fields where already emitted:
  - `id`
  - `range_m`
  - `v_closing`
  - `t_go_raw`
  - `t_go_filt`
  - `delta_t_go_raw`
  - `feasible_geom`
  - `rollout_gate_ok`
  - `speed_cmd`
  - `vx`
  - `vy`
  - `vz`
- Add fields only at line end or behind parser-compatible handling.

## AI-Agent Execution Guidance

### Preferred Edit Order

1. Read `ARCHITECTURE_MAP.md`.
2. Read `guidance_lib.py`.
3. Read `guidance_kernel.py`.
4. Read `test_guidance_kernel_snapshots.py`.
5. Add/adjust parity snapshots.
6. Extract pure helpers into `guidance_kernel.py` only after tests cover them.
7. Update `_compute_guidance_command_for_pair` adapter in `interception_logic_node.py`.
8. Verify single-target and multi-target call sites.
9. Run targeted tests before cleanup.
10. Cleanup duplicate helpers in a separate change.

### Dangerous Files

- `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py`
- `src/counter_uas/launch/bringup.launch.py`
- `src/gazebo_target_sim/launch/gazebo_target.launch.py`
- `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py`
- `scripts/analyze_run.py`
- `scripts/monte_carlo.py`
- `scripts/evaluation/*.py`
- `src/gazebo_target_sim_interfaces/msg/ImpactEvent.msg`

### Common Regression Traps

- Double-applying measurement delay.
- Changing solver tolerance while moving wrappers.
- Losing `tracks_state` velocity and falling back to finite differencing.
- Sharing one guidance memory key across different target/interceptor pairs.
- Treating UI markers as evaluation truth.
- Changing `[ENG_METRIC]` fields without parser updates.
- Moving plant limits into guidance command generation.
- Letting offline timing parity stand in for full command parity.
- Broad formatting churn in large ROS nodes.

### Required Test Subsets Before Commits

- Pure guidance changes:
  - `pytest src/counter_uas/test/test_guidance_lib.py src/counter_uas/test/test_intercept_solver_golden.py src/counter_uas/test/test_guidance_kernel_snapshots.py`
- Plant-coupled changes:
  - `pytest src/counter_uas/test/test_kinematic_plant.py src/counter_uas/test/test_autopilot_model.py src/counter_uas/test/test_accel_limit_ramp.py`
- Adapter/log changes:
  - `pytest src/counter_uas/test/test_selection_audit.py src/counter_uas/test/test_analyze_run_parse.py src/counter_uas/test/test_monte_carlo.py src/counter_uas/test/test_classify_run.py`
- Heatmap/offline parity changes:
  - `pytest src/counter_uas/test/test_heatmap_agreement.py`

## Unresolved Ambiguities

- `plans/layer_a/OVERVIEW.md` was requested as an input but is not present in the workspace.
- `guidance_kernel.py` already exists and is used by `interception_logic_node.py`; A01 is therefore a stabilization/migration task, not a first extraction from scratch.
- Measurement delay ownership needs confirmation before runtime edits:
  - current kernel supports `TargetKinematics.measurement_delay_s`
  - current live path also manually adjusts target position before command generation
  - implementation must avoid double compensation
- Multi-target guidance memory key semantics need explicit confirmation before adapter changes.
- Offline parity currently covers solver timing through `simulation/core/intercept.py`, not full command generation.

## Recommended Next Implementation Task

- Start with Phase 1.
- Expand `src/counter_uas/test/test_guidance_kernel_snapshots.py` with table-driven parity snapshots.
- Do not edit `interception_logic_node.py` until snapshot coverage pins current command behavior.

## Related Contracts

- `ARCHITECTURE_MAP.md`: Guidance And Control Architecture, Tracking/Fusion Contracts, Offline And Evaluation Pipeline, Core System Contracts, Preferred Edit Paths.
- Topic contracts:
  - `/tracks/state` as `nav_msgs/Odometry`
  - `/tracks` as `geometry_msgs/Point`
  - `/fused_detections` as `geometry_msgs/Point`
  - `/<interceptor_id>/cmd_velocity` as `geometry_msgs/Vector3`
  - `/<interceptor_id>/position` as `geometry_msgs/Point`

## Related Tests

- `src/counter_uas/test/test_guidance_lib.py`
- `src/counter_uas/test/test_guidance_kernel_snapshots.py`
- `src/counter_uas/test/test_intercept_solver_golden.py`
- `src/counter_uas/test/test_kinematic_plant.py`
- `src/counter_uas/test/test_autopilot_model.py`
- `src/counter_uas/test/test_accel_limit_ramp.py`
- `src/counter_uas/test/test_selection_audit.py`
- `src/counter_uas/test/test_heatmap_agreement.py`
- `src/counter_uas/test/test_analyze_run_parse.py`
- `src/counter_uas/test/test_monte_carlo.py`
- `src/counter_uas/test/test_classify_run.py`

## Related Docs

- `ARCHITECTURE_MAP.md`
- `plans/layer_a/OVERVIEW.md` (expected input; currently missing)
- `README.md`
- `docs/gnc_implementation_roadmap_41a8c5bb.plan.md`
- `docs/evaluation/gnc_guidance_tuning_playbook_ops.md`
- `docs/evaluation/layer_b_realism_validation.md`
- `docs/evaluation/layer_c_statistical_validation.md`
