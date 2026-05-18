# Offline defense pipeline review split

This branch is large enough that reviewers should read it by behavior area rather than as one flat diff. The branch-level narrative is:

1. Make the Gazebo counter-UAS runs reproducible and parseable.
2. Align offline GNC estimates with the live Gazebo dynamics envelope.
3. Add soft-hit/contact evidence and guidance tuning knobs behind documented defaults.
4. Archive campaign thresholds and fixtures so future runs can be compared by cohort.

## Suggested PR slices

### 1. Evaluation harness and metrics

Primary files:

- `scripts/run_capture.py`
- `scripts/monte_carlo.py`
- `scripts/run_scenario_matrix.py`
- `scripts/ci_eval.sh`
- `scripts/evaluation/`
- `scripts/analyze_run.py`
- `scripts/summarize_run.py`
- `src/counter_uas/test/test_analyze_run_parse.py`
- `src/counter_uas/test/test_classify_run.py`
- `src/counter_uas/test/test_monte_carlo.py`
- `src/counter_uas/test/test_selection_audit.py`

Review focus:

- Log and `.meta.json` contracts are stable enough for aggregation.
- Cohort filtering prevents mixed diagnostic logs from contaminating campaign summaries.
- Failure-class rules are conservative and tested with fixtures.

### 2. Intercept math and offline validation

Primary files:

- `simulation/core/intercept.py`
- `simulation/core/dynamics.py`
- `simulation/guidance/`
- `simulation/realtime_sim.py`
- `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py`
- `scripts/render_intercept_heatmap_prob_offline.py`
- `scripts/validate_heatmap_vs_gazebo.py`
- `scripts/intercept_heatmap_viz_3d.py`
- `scripts/plot_intercept_heatmap_csv_3d.py`
- `src/counter_uas/test/test_intercept_solver_golden.py`
- `src/counter_uas/test/test_guidance_lib.py`

Review focus:

- Offline solver semantics match documented Gazebo limits where intended.
- Golden tests cover edge cases before shared solver behavior is trusted by eval tooling.
- Heatmap defaults match launch defaults unless explicitly overridden.

### 3. Gazebo soft-hit and contact corroboration

Primary files:

- `src/gazebo_target_sim_interfaces/`
- `src/gazebo_target_sim/gazebo_target_sim/hit_contact_corroborator_node.py`
- `src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py`
- `src/gazebo_target_sim/worlds/target_sphere.sdf`
- `src/gazebo_target_sim/worlds/target_sphere_multi.sdf`
- `src/gazebo_target_sim/models/hit_explosion/model.sdf`
- `src/gazebo_target_sim/rviz/hit_overlay.rviz`
- `scripts/smoke_impact_event.py`

Review focus:

- `ImpactEvent` remains a validation signal and does not redefine primary success on its own.
- World/contact additions do not make headless smoke tests brittle.
- Package wiring is complete after a clean `colcon build`.

### 4. Guidance behavior and launch wiring

Primary files:

- `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py`
- `src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py`
- `src/gazebo_target_sim/launch/gazebo_target.launch.py`
- `src/gazebo_target_sim/launch/gazebo_target_multi.launch.py`
- `src/counter_uas/launch/bringup.launch.py`
- `src/counter_uas/config/`
- `src/counter_uas/test/test_accel_limit_ramp.py`
- `src/counter_uas/test/test_autopilot_model.py`

Review focus:

- New behavior-changing knobs have safe documented defaults.
- Rollout feasibility gate is opt-in and observable through logs.
- Terminal blend, slew, PN, and speed-coherence knobs can be A/B tested by cohort.

### 5. Scenario, credibility, and operator docs

Primary files:

- `README.md`
- `docs/autonomous_counter_uas_interception_report.md`
- `docs/CREDIBILITY_REALISM_TECHNICAL_REVIEW.md`
- `docs/gnc_intercept_review.md`
- `docs/gnc_implementation_roadmap_41a8c5bb.plan.md`
- `docs/evaluation/`
- `docs/scenarios/`
- `research/`

Review focus:

- Claims stay scoped to a software-integration demonstrator.
- Archived aggregate fixtures and pass/fail gates are clearly labeled as scenario parameters.
- Roadmap status matches implemented branch behavior.

## Validation status for this workspace

Commands run after doc cleanup:

```bash
python3 -m pytest src/counter_uas/test/ -q --tb=short
scripts/ci_eval.sh tier0
colcon build --symlink-install
bash -lc 'source install/setup.bash && python3 scripts/smoke_impact_event.py'
bash -lc 'source install/setup.bash && scripts/ci_eval.sh tier1 --timeout-s 180 --launch-args "use_gazebo_gui:=false enable_hit_contact_corroborator:=true"'
bash -lc 'source install/setup.bash && scripts/ci_eval.sh tier2-smoke --timeout-s 120'
bash -lc 'source install/setup.bash && MATRIX=scripts/evaluation/fixtures/scenario_generalization_sg1_smoke_single_cell.csv N=1 TIMEOUT_S=120 STUDY=gnc_sg1_smoke scripts/evaluation/run_scenario_generalization_a_vs_d.sh'
python3 scripts/evaluation/summarize_scenario_generalization.py --matrix-csv scripts/evaluation/fixtures/scenario_generalization_sg1_smoke_single_cell.csv --study gnc_sg1_smoke --n 1 --seed-base 9701 --out-csv runs/evaluation/gnc_sg1_smoke_a_vs_d_summary.csv --worst-csv runs/evaluation/gnc_sg1_smoke_a_vs_d_worst_cells.csv --make-failure-hists
```

Results:

- `pytest`: 54 passed.
- `tier0`: passed, including heatmap dry-run fixture selection.
- `colcon build`: 10 packages finished.
- `smoke_impact_event.py`: passed after sourcing `install/setup.bash`.
- `tier1`: exit 0, produced a single run log and meta file.
- `tier2-smoke`: exit 0, wrote `runs/evaluation/ci_eval_matrix_smoke_latest.csv` with 1 row.
- SG1 smoke generalization: matched seed `9701`, single cell `los38_div10_default`, A and D both 100% success at `N=1`; summary and worst-cell CSVs written under `runs/evaluation/`.

## PR description skeleton

```markdown
## Summary
- Add a reproducible offline/Gazebo evaluation pipeline with cohort metadata, failure classification, matrix execution, and CI-style tiers.
- Align intercept/heatmap analysis with Gazebo dynamics limits and add golden coverage around solver behavior.
- Add opt-in rollout feasibility/guidance tuning knobs plus soft-hit/contact corroboration plumbing and documented campaign thresholds.

## Validation
- `python3 -m pytest src/counter_uas/test/ -q --tb=short`
- `scripts/ci_eval.sh tier0`
- `colcon build --symlink-install`
- `python3 scripts/smoke_impact_event.py` after `source install/setup.bash`
- `scripts/ci_eval.sh tier1 --timeout-s 180 --launch-args 'use_gazebo_gui:=false enable_hit_contact_corroborator:=true'` after `source install/setup.bash`
- `scripts/ci_eval.sh tier2-smoke --timeout-s 120` after `source install/setup.bash`
- SG1 smoke: `MATRIX=scripts/evaluation/fixtures/scenario_generalization_sg1_smoke_single_cell.csv N=1 TIMEOUT_S=120 STUDY=gnc_sg1_smoke scripts/evaluation/run_scenario_generalization_a_vs_d.sh` after `source install/setup.bash`
```
