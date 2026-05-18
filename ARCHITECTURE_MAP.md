# Counter-UAS Architecture Map

Canonical, repo-grounded map for fast onboarding and scoped AI coding tasks.
Trust source/launch files over older docs. Update this file when contracts or defaults change.

## 1. System At A Glance

| Item | Contract |
|---|---|
| Scope | Research-grade autonomous counter-UAS interception simulator. |
| Model class | Kinematic target/interceptor simulation with policy-driven engagement gates. |
| Validation claim | Validated within stated assumptions through logs, Monte Carlo, heatmap checks, and focused tests. |
| Not claimed | Not field-ready, not operationally representative, not aerodynamic `P_kill`, not a real classifier/ROE stack. |
| Main modes | Gazebo single target, Gazebo multi target, full-stack perception/tracking bringup, offline evaluation. |
| Source of truth | Launch/source files first; older docs are reference material only when they match code. |

Primary runtime chain:

```text
target motion -> sensor simulation -> fusion -> tracking -> guidance -> interceptor plant/controller -> hit/log metrics
```

## 2. ROS2 Package Map

| Package | Responsibility | Key files |
|---|---|---|
| `gazebo_target_sim` | Gazebo target/interceptor simulation, guidance, hit logic, plant, optional noisy measurement. | `src/gazebo_target_sim/gazebo_target_sim/*.py`, `src/gazebo_target_sim/launch/*.launch.py` |
| `counter_uas` | Top-level full-stack launch/config/test package. | `src/counter_uas/launch/bringup.launch.py`, `src/counter_uas/config/*.yaml`, `src/counter_uas/test/test_*.py` |
| `radar_sim` | Ground-truth Point to radar detections with range/FOV/probability gates. | `src/radar_sim/radar_sim/radar_sim_node.py` |
| `camera_sim` | Ground-truth Point to camera detections with mount/FOV/range/probability gates. | `src/camera_sim/camera_sim/camera_sim_node.py` |
| `fusion` | Radar/camera point fusion into one coherent fused detection stream. | `src/fusion/fusion/fusion_node.py` |
| `tracking` | Candidate confirmation and CV Kalman tracking from fused detections. | `src/tracking/tracking/tracking_node.py` |
| `threat_assessment` | Distance-band threat labels from tracks. | `src/threat_assessment/threat_assessment/threat_assessment_node.py` |
| `visualization` | RViz markers and threat/trail visualization. | `src/visualization/visualization/viz_node.py` |
| `world_sim` | Optional non-Gazebo kinematic target source; not launched by default bringup. | `src/world_sim/world_sim/world_sim_node.py` |
| `gazebo_target_sim_interfaces` | Shared ROS interfaces for Gazebo sim events. | `src/gazebo_target_sim_interfaces/msg/ImpactEvent.msg` |

## 3. Repo Layer Diagram

Layer stack, from lower-level contracts to user-facing outputs:

```text
Shared Math And Contracts
Perception And State Estimation
Guidance And Tactical Policy
Plant And Gazebo Integration
Launch And Configuration
Evaluation And Analytics
UI And Visualization
```

| Layer | Owns | Examples |
|---|---|---|
| Shared Math And Contracts | Pure math, dataclass contracts, message definitions, small helpers. | `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py`, `src/gazebo_target_sim/gazebo_target_sim/guidance_kernel.py`, `src/gazebo_target_sim/gazebo_target_sim/kinematic_plant.py`, `src/gazebo_target_sim_interfaces/msg/ImpactEvent.msg` |
| Perception And State Estimation | Sensor simulation, fusion, candidate confirmation, track state publication. | `src/radar_sim/radar_sim/radar_sim_node.py`, `src/camera_sim/camera_sim/camera_sim_node.py`, `src/fusion/fusion/fusion_node.py`, `src/tracking/tracking/tracking_node.py` |
| Guidance And Tactical Policy | Source selection, interceptor selection, guidance policy, hit legality, metrics emission. | `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py`, `src/threat_assessment/threat_assessment/threat_assessment_node.py` |
| Plant And Gazebo Integration | Gazebo pose services, target/interceptor live motion, noisy measurement adapter, contact corroboration. | `src/gazebo_target_sim/gazebo_target_sim/target_controller_node.py`, `src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py`, `src/gazebo_target_sim/gazebo_target_sim/noisy_measurement_node.py`, `src/world_sim/world_sim/world_sim_node.py` |
| Launch And Configuration | Runtime composition, defaults, topic forwarding, parameter selection. | `src/counter_uas/launch/bringup.launch.py`, `src/gazebo_target_sim/launch/gazebo_target.launch.py`, `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py`, `src/counter_uas/config` |
| Evaluation And Analytics | Live capture boundary, log parsing, MC aggregation, statistics, heatmap validation. | `scripts/run_capture.py`, `scripts/analyze_run.py`, `scripts/monte_carlo.py`, `scripts/render_intercept_heatmap_prob_offline.py`, `scripts/validate_heatmap_vs_gazebo.py`, `scripts/evaluation` |
| UI And Visualization | RViz/user-facing visualization and visual event rendering. | `src/visualization/visualization/viz_node.py`, marker publishers in `interception_logic_node.py`, heatmap visualization scripts |

Live ROS/Gazebo composes perception, policy, plant, launch, and visualization. Offline evaluation analyzes logs/artifacts and may reuse pure math, but should not redefine live behavior silently.

### Layer Dependency Rules

- Shared math/contracts may be used by any layer; they must not import ROS nodes, launch files, or scripts.
- Perception/state estimation publishes topic contracts; downstream policy consumes topics, not node internals.
- Guidance/policy may call pure math and consume GT/fused/tracked topics; it should not own sensor simulation or Gazebo pose integration.
- Plant/Gazebo integration consumes commands and publishes positions/events; it should not redefine guidance semantics.
- Launch/config wires layers together and owns defaults; it should not hide behavior that belongs in source/tests.
- Evaluation/analytics may parse logs and reuse pure math; live-vs-offline differences must be documented.
- UI/visualization observes state/events; it must not become the source of simulation truth.

### Layer Anti-Patterns

- `cross-layer leakage`: offline scripts duplicate guidance constants without noting launch parity.
- `hidden default drift`: launch defaults change without updating contracts, tests, and docs.
- `Point/Odometry confusion`: `/tracks/state` full-stack validation is accidentally replaced by position-only `/tracks`.
- `large-node expansion`: unrelated responsibilities are added to `interception_logic_node.py` instead of pure helpers or focused nodes.
- `UI as truth`: RViz markers or visual-only events become evaluation source data.
- `parser/schema drift`: log tags or MC CSV fields change without parser/test updates.

## 4. Launch Map

| Launch file | Owns | Important defaults | Notes |
|---|---|---|---|
| `src/counter_uas/launch/bringup.launch.py` | Full-stack default launch. Includes Gazebo single target plus radar, camera, fusion, tracking, threat, visualization. | `intercept_measurement_source=tracks_state`, `fused_detections_topic=/fused_detections`, `tracks_topic=/tracks`, `tracks_state_topic=/tracks/state`, `use_gazebo_gui=true`, `use_rviz=false`. | Preferred full-stack validation entrypoint. |
| `src/gazebo_target_sim/launch/gazebo_target.launch.py` | Standalone single-target Gazebo engagement. | `intercept_measurement_source=ground_truth`, `ground_truth_topic=/drone/position`, `noisy_topic=/drone/position_noisy`, `tracks_state_topic=/tracks/state`, `use_noisy_measurement=false`, `cuas_engagement_preset=far_ram`. | Override `intercept_measurement_source` to test fused/tracked paths directly. |
| `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py` | Three-target Gazebo engagement and assignment. | `intercept_measurement_source=ground_truth`; `use_noisy_measurement=false`; `noisy_suffix=_noisy`. | In multi-target mode the single-target selector is effectively ignored; guidance uses per-target Point topics. |

Source-verified conflict notes:

| Existing doc | Conflict | Current source truth |
|---|---|---|
| `README.md` | Says bringup default is `tracks` and omits `tracks_state` in places. | `bringup.launch.py` defaults to `tracks_state`. |
| `docs/scenarios/README.md` | Lists `ground_truth | fused | tracks`. | `tracks_state` is supported and is the full-stack bringup default. |
| `docs/scenarios/scenario_catalog.yaml` | Tracks-oriented source values. | Treat as historical until aligned with launch files. |

## 5. Live Simulation Flow

| Stage | Owner | Input | Output |
|---|---|---|---|
| Target motion | `target_controller_node` or `world_sim_node` | Launch/config parameters | `/drone/position` or `/drone_i/position` as `geometry_msgs/Point` |
| Sensor simulation | `radar_sim_node`, `camera_sim_node` | `/drone/position` | `/radar/detections`, `/camera/detections` |
| Fusion | `fusion_node` | Radar/camera detections | `/fused_detections`, optional `/fused_detections_stamped` |
| Tracking | `tracking_node` | `/fused_detections` | `/tracks` Point and `/tracks/state` Odometry |
| Guidance/hit policy | `interception_logic_node` | GT, fused, tracks, or tracks_state | `/<interceptor_id>/cmd_velocity`, stop signals, impact events, metrics logs |
| Plant/controller | `interceptor_controller_node` | `cmd_velocity` | `/<interceptor_id>/position`, Gazebo `set_pose` |
| Evaluation | logs/scripts | `[HIT]`, `[METRICS]`, `[min_miss]`, sidecars | JSON/CSV/SVG/HTML artifacts |

## 6. Guidance And Control Architecture

| File | Boundary | Treat carefully because |
|---|---|---|
| `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py` | Shared intercept math: time solve, intercept point, delay compensation, speed alignment. | Golden tests and live/offline parity depend on solver semantics. |
| `src/gazebo_target_sim/gazebo_target_sim/guidance_kernel.py` | Pure guidance kernel: dataclass inputs/outputs and `compute_guidance_command`. | Shared-kernel boundary for snapshot tests and future offline/live parity. |
| `src/gazebo_target_sim/gazebo_target_sim/kinematic_plant.py` | Pure plant step: delay, autopilot smoothing, turn/accel limits, position integration. | Used by controller tests and offline reasoning. |
| `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py` | ROS node for engagement policy, target source selection, guidance calls, hit logic, metrics. | Large ownership hotspot; contracts span topics, logs, policy, and visualization. |
| `src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py` | ROS/Gazebo plant adapter: consumes commands, steps plant, publishes position, calls Gazebo pose service. | Live dynamics behavior must stay aligned with plant tests and launch parameters. |

Boundaries:

```text
pure math: guidance_lib.py + guidance_kernel.py + kinematic_plant.py
ROS policy: interception_logic_node.py
Gazebo actuator adapter: interceptor_controller_node.py
offline scripts: import/reuse math where possible; must not silently diverge from live launch defaults
```

## 7. Tracking/Fusion Contracts

| Topic | Type | Publisher | Consumer | Contract |
|---|---|---|---|---|
| `/radar/detections` | `geometry_msgs/Point` | `radar_sim_node` | `fusion_node` | Position-only radar detection. |
| `/camera/detections` | `geometry_msgs/Point` | `camera_sim_node` | `fusion_node` | Position-only camera detection. |
| `/fused_detections` | `geometry_msgs/Point` | `fusion_node` | `tracking_node`, optional guidance | Single coherent measurement stream. |
| `/tracks` | `geometry_msgs/Point` | `tracking_node` | threat/visualization, legacy guidance | Position-only primary track by default. |
| `/tracks/state` | `nav_msgs/Odometry` | `tracking_node` | preferred full-stack guidance | Position, velocity, covariance, `child_frame_id=track_<id>`. |

Point vs state contract:

| Path | Contents | Use |
|---|---|---|
| Point topics | `x/y/z` only; no stamp, covariance, ID, or velocity. | Simple visualization, legacy paths, single-target quick checks. |
| `/tracks/state` Odometry | pose, KF velocity, covariance, frame, track ID in `child_frame_id`. | Preferred for full-stack validation because guidance uses filter velocity instead of noisy finite differences. |

Fusion ownership:

| Setting | Default | Meaning |
|---|---|---|
| `fusion.require_paired_inputs` | `true` | Publish once per fresh radar/camera pair. |
| `fusion.disagreement_strategy` | `prefer_radar` | On disagreement, emit at most one point. |
| `fusion.use_stamped_inputs` | `false` | Optional stamped-input path exists but default flow uses Point inputs. |

## 8. Topic Graphs

Single-target full stack:

```text
/drone/position
  -> radar_sim_node -> /radar/detections
  -> camera_sim_node -> /camera/detections
  -> fusion_node -> /fused_detections
  -> tracking_node -> /tracks/state
  -> interception_logic_node -> /interceptor_*/cmd_velocity
  -> interceptor_controller_node -> /interceptor_*/position
  -> interception_logic_node -> [HIT], /target/stop, /interception/impact_event, metrics logs
```

Gazebo-only single target:

```text
target_controller_node -> /drone/position
  -> interception_logic_node (default ground_truth)
  -> /interceptor_*/cmd_velocity
  -> interceptor_controller_node
  -> Gazebo set_pose + /interceptor_*/position
  -> hit/stop/metrics
```

Multi-target flow:

```text
target_controller_node[0..2] -> /drone_0/position, /drone_1/position, /drone_2/position
  -> interception_logic_node (multi_target_enabled)
  -> /interceptor_*/assigned_target + /interceptor_*/cmd_velocity
  -> interceptor_controller_node[*]
  -> /target_*/stop + /interception/impact_event + metrics
```

Noisy-measurement/fusion/tracking path:

```text
standalone Gazebo noisy target source:
/drone/position
  -> noisy_measurement_node -> /drone/position_noisy

full-stack sensor/fusion/tracking noise path:
/drone/position
  -> radar_sim_node + camera_sim_node (sensor noise, FOV, range, detection probability)
  -> /radar/detections + /camera/detections
  -> fusion_node -> /fused_detections
  -> tracking_node -> /tracks + /tracks/state
```

## 9. Offline And Evaluation Pipeline

| File | Live/offline | Contract |
|---|---|---|
| `scripts/run_capture.py` | Live boundary | Launches Gazebo/ROS, captures `runs/logs/<run_id>.log` and `<run_id>.meta.json`. |
| `scripts/analyze_run.py` | Offline parser | Parses logs into `success`, `miss_distance_m`, `intercept_time_s`, `time_margin_s`, `layer_at_hit`, `notes`. |
| `scripts/monte_carlo.py` | Mixed | `run` drives live captures; `aggregate` and `compare` are offline; writes `runs/mc/<label>.json/.csv`. |
| `scripts/render_intercept_heatmap_prob_offline.py` | Offline | Exports model-conditioned heatmap probability artifacts; should track live guidance/plant assumptions. |
| `scripts/validate_heatmap_vs_gazebo.py` | Mixed | Selects cells offline; executes trials through `run_capture()` unless dry-run. |
| `scripts/evaluation/statistical_validation.py` | Offline | Layer C aggregate, paired-seed, and manifest-validation reports. |
| `scripts/evaluation/stats_helpers.py` | Offline | Wilson intervals, bootstrap intervals, paired deltas, cohort tiers. |

Artifact/log contracts:

| Contract | Fields/tags |
|---|---|
| Log headers | `run_id`, `created_utc`, `git_commit`, `git_dirty`, `cohort`, `scenario`, `timeout_s`, `cmd`. |
| Metrics tags | `[METRICS]`, `[ENG_METRIC]`, `[FEASIBILITY]`, `[min_miss]`, `[HIT]`. |
| MC CSV | `run_id`, `success`, `miss_distance_m`, `intercept_time_s`, `layer_at_hit`, `noise_seed_mc`, `seed`, `geometry_id`, `cohort`, `meta_path`, `git_commit`, `git_dirty`, `launch_args_raw`, `notes`, `log_path`. |
| Heatmap artifacts | CSV/SVG latest files, optional stamped files, optional 3D PNG/HTML. |

Stale-doc conflict:

| Existing doc | Conflict | Current source truth |
|---|---|---|
| `docs/autonomous_counter_uas_interception_report.md` | Says `monte_carlo.py` does not emit confidence intervals by default. | Current code emits Wilson success intervals and bootstrap miss/time intervals. |
| `docs/gnc_implementation_roadmap_41a8c5bb.plan.md` | Mentions old offline heatmap turn-rate default `1.5`. | Current checked defaults are `2.5 rad/s` turn rate and `30 m/s^2` accel where exposed. |

## 10. Tests And Validation Map

| Test file | Coverage |
|---|---|
| `src/counter_uas/test/test_guidance_lib.py` | Intercept solver and guidance math basics. |
| `src/counter_uas/test/test_guidance_kernel_snapshots.py` | Shared guidance kernel snapshots. |
| `src/counter_uas/test/test_intercept_solver_golden.py` | Golden intercept solver behavior. |
| `src/counter_uas/test/test_kinematic_plant.py` | Plant step, delay, turn/accel limiting. |
| `src/counter_uas/test/test_accel_limit_ramp.py` | Acceleration limiting behavior. |
| `src/counter_uas/test/test_autopilot_model.py` | Autopilot/delay model behavior. |
| `src/counter_uas/test/test_fusion_pair_up.py` | Fusion paired-input/disagreement behavior. |
| `src/counter_uas/test/test_tracking_candidate_gate.py` | Candidate confirmation and `/tracks/state` publication. |
| `src/counter_uas/test/test_analyze_run_parse.py` | Log parsing and result schema. |
| `src/counter_uas/test/test_monte_carlo.py` | Monte Carlo aggregate/run metadata contracts. |
| `src/counter_uas/test/test_stats_helpers.py` | Wilson/bootstrap/paired statistics helpers. |
| `src/counter_uas/test/test_statistical_validation.py` | Layer C report and manifest validation. |
| `src/counter_uas/test/test_heatmap_agreement.py` | Heatmap-vs-Gazebo validation logic. |
| `src/counter_uas/test/test_classify_run.py` | Run classification logic. |
| `src/counter_uas/test/test_failure_taxonomy.py` | Failure taxonomy output. |
| `src/counter_uas/test/test_selection_audit.py` | Interceptor selection audit behavior. |
| `src/counter_uas/test/test_triage_engagement_script.py` | Evaluation/triage script smoke coverage. |

Current docs contain historical pass counts (`66 passed` vs `54 passed`). Re-run tests for current truth.

## 11. Core System Contracts

| Contract | Files to treat carefully | Do not casually change |
|---|---|---|
| Launch defaults | `src/counter_uas/launch/bringup.launch.py`, `src/gazebo_target_sim/launch/gazebo_target.launch.py`, `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py` | Defaults, launch arg names, topic forwarding. |
| Topic/message contracts | Sensor nodes, `src/fusion/fusion/fusion_node.py`, `src/tracking/tracking/tracking_node.py`, `src/gazebo_target_sim_interfaces/msg/ImpactEvent.msg` | Topic names, message types, `child_frame_id=track_<id>`. |
| Guidance kernel contracts | `guidance_lib.py`, `guidance_kernel.py`, guidance tests | Dataclass fields, solver tolerances, mode semantics. |
| Plant model contracts | `kinematic_plant.py`, `interceptor_controller_node.py`, plant tests | Delay, tau, acceleration, turn-rate, measured/fixed dt behavior. |
| Metrics/log schema contracts | `interception_logic_node.py`, `scripts/analyze_run.py`, `scripts/monte_carlo.py` | Log tags and parseable fields. |
| Evaluation artifact contracts | `scripts/evaluation/*.py`, heatmap scripts, `runs/mc/*.csv` schema | CSV/JSON field names and confidence interval semantics. |

## 12. Known Architecture Risks

| Risk | Why it matters | Navigation |
|---|---|---|
| Duplicated/confusing guidance ownership | Guidance math exists in pure helpers, a shared kernel, and a large ROS node. | Start at `guidance_kernel.py`, then `interception_logic_node.py`. |
| Large-node ownership hotspots | `interception_logic_node.py` owns policy, target selection, guidance wiring, hit logic, logs, and markers. | Keep edits narrow and test parser/topic side effects. |
| Live/offline parity risk | Offline heatmaps and MC summaries can drift from live launch defaults. | Compare scripts with launch args before claims. |
| Stale documentation risk | README/scenario/report docs lag current `tracks_state` and statistics behavior. | Trust source; update this file when source moves. |
| Point vs `tracks_state` confusion | Point topics are position-only; `tracks_state` carries KF velocity/covariance. | Use `tracks_state` for full-stack validation unless deliberately testing legacy paths. |
| Launch duplication risk | Bringup forwards args into Gazebo launch; single and multi launches own different defaults. | Verify the launch entrypoint before changing defaults. |

## 13. AI Coding Agent Onboarding

Recommended reading order for scoped work:

| Task type | Read in order |
|---|---|
| General orientation | `README.md` -> `ARCHITECTURE_MAP.md` -> relevant docs. |
| Launch/default changes | `ARCHITECTURE_MAP.md` -> relevant launch file -> configs -> matching tests/docs. |
| Topic/message changes | `ARCHITECTURE_MAP.md` -> publisher -> consumer -> tests -> evaluation parsers if logs/artifacts change. |
| Guidance changes | `guidance_lib.py` -> `guidance_kernel.py` -> `interception_logic_node.py` -> guidance tests. |
| Plant/dynamics changes | `kinematic_plant.py` -> `interceptor_controller_node.py` -> plant/autopilot tests. |
| Evaluation changes | `scripts/run_capture.py` -> `scripts/analyze_run.py` -> `scripts/monte_carlo.py` -> `scripts/evaluation/*.py` -> tests. |

Agent rules:

- Identify the launch entrypoint first.
- Preserve topic/message and log schemas unless the task explicitly changes them.
- Prefer source over docs when conflicts exist.
- Update this file when architecture contracts change.

## 14. Preferred Edit Paths

| Change type | Start here | Then check | Tests / validation | Common failure modes | Docs that may go stale |
|---|---|---|---|---|---|
| Guidance semantics | `guidance_lib.py`, `guidance_kernel.py` | `interception_logic_node.py`, launch guidance args, heatmap scripts | `test_guidance_lib.py`, `test_guidance_kernel_snapshots.py`, `test_intercept_solver_golden.py`, relevant plant/heatmap tests | live/offline drift; solver tolerance changes; mode transition regressions | `ARCHITECTURE_MAP.md`, GNC docs, evaluation docs |
| Launch defaults | Relevant launch file | forwarded args in `bringup.launch.py`, config YAML, README/scenario docs | grep defaults; lightweight launch/import smoke if present; targeted tests for affected path | bringup vs standalone mismatch; stale README; wrong full-stack source | `README.md`, `docs/scenarios`, `ARCHITECTURE_MAP.md` |
| Topic/message contracts | Publisher and consumer together | launch topic args, tests, parsers if logs/artifacts include the field | `test_fusion_pair_up.py`, `test_tracking_candidate_gate.py`, affected parser/eval tests | type mismatch; missing velocity/covariance; stale topic graph | `ARCHITECTURE_MAP.md`, README topic tables, scenario docs |
| Metrics/log schema | `interception_logic_node.py`, `scripts/analyze_run.py` | `monte_carlo.py`, `scripts/evaluation/*.py`, failure classifiers | `test_analyze_run_parse.py`, `test_monte_carlo.py`, `test_classify_run.py`, `test_failure_taxonomy.py` | unparsable logs; missing fields; broken MC CSV | `ARCHITECTURE_MAP.md`, `docs/evaluation`, reports |
| Monte Carlo/statistical outputs | `scripts/monte_carlo.py`, `scripts/evaluation/stats_helpers.py`, `scripts/evaluation/statistical_validation.py` | `docs/evaluation`, downstream summary scripts | `test_monte_carlo.py`, `test_stats_helpers.py`, `test_statistical_validation.py` | incompatible CSV/JSON; confidence interval drift; missing seed/cohort metadata | `docs/evaluation`, reports, `ARCHITECTURE_MAP.md` |
| Plant dynamics | `kinematic_plant.py` | `interceptor_controller_node.py`, launch plant args, guidance assumptions | `test_kinematic_plant.py`, `test_autopilot_model.py`, `test_accel_limit_ramp.py`, guidance snapshots if command behavior changes | live/offline mismatch; hidden dt behavior; broken heatmap parity | GNC docs, evaluation docs, `ARCHITECTURE_MAP.md` |
| Tracking/fusion contracts | `fusion_node.py`, `tracking_node.py` | radar/camera nodes, launch/config, `interception_logic_node.py` source selection | `test_fusion_pair_up.py`, `test_tracking_candidate_gate.py` | duplicate fused outputs; empty `/tracks/state`; Point/Odometry confusion | README topics, scenario docs, `ARCHITECTURE_MAP.md` |
| UI/event schemas | `viz_node.py`, marker/event publishers in `interception_logic_node.py`, `ImpactEvent.msg` | contact corroborator, smoke scripts, visualization docs | `test_selection_audit.py`, `test_triage_engagement_script.py`, script smoke tests if relevant | stale marker/event assumptions; visual-only state treated as evaluation truth | visualization docs, event docs, `ARCHITECTURE_MAP.md` |

Validation/update checklist:

- Confirm changed files map to one primary layer and list downstream layers touched.
- Check topic/message/log/schema contracts before editing tests.
- Run targeted tests for the touched edit category.
- Grep `ARCHITECTURE_MAP.md`, `README.md`, `docs/scenarios`, `docs/evaluation`, and GNC docs for stale defaults or source names.
- For launch/default changes, verify bringup, standalone single-target, and multi-target ownership separately.
- For live/offline changes, compare launch defaults with evaluation script defaults.

## 15. Update Policy

Update `ARCHITECTURE_MAP.md` when any of these change:

| Trigger | Examples |
|---|---|
| Launch defaults change | `intercept_measurement_source`, topic args, noisy measurement, Gazebo/RViz defaults. |
| Topic/message contracts change | Topic names, message types, `tracks_state` fields, `ImpactEvent`. |
| Guidance kernel API changes | Dataclasses, solver outputs, guidance mode semantics. |
| Plant model contracts change | Delay/tau/accel/turn-rate behavior, dt integration policy. |
| Evaluation pipeline changes | Log tags, parser output fields, MC CSV/JSON schemas, heatmap artifacts. |
| New packages/nodes are added | Package map, flow diagrams, launch map. |
| Major roadmap phase changes | Validation assumptions, full-stack preferred path, known risks. |

Future maintenance:

- When a PR changes a launch default, topic/message type, guidance kernel API, plant behavior, log/MC schema, or visualization event contract, update the matching layer and edit-path row in this file.
- If a change crosses more than two layers, call out the layer boundary in the PR summary and run tests from each affected edit-path row.
- If docs conflict with source, update this map from source first and mark older docs as stale until corrected.
