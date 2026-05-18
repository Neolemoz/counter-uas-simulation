from __future__ import annotations

import csv
import importlib.util
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_module():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'replay_observability.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('replay_observability', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_SYNTHETIC_LOG = """
=== run_id: replay_obs_unit ===
=== git_commit: abc123 ===
=== git_dirty: False ===
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=12.500 m | t_go=2.5 s | vel=18.0 m/s | mode=predict
[interception_logic_node-1] [min_miss] = 7.812 m
[lifecycle_observer_node-1] [LIFECYCLE_OBSERVER] event=summary tracks_state_msgs_window=3 tracks_state_msgs_total=3 unique_track_ids_window=2 track_id_changes_window=1 track_gap_events_window=1 track_persistence_events_window=2 selected_id_changes_window=1 idle_s=0.400 last_track_id=8 selected_id=interceptor_2
[lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_gap gap_s=0.400 prev_track_id=7 next_track_id=8 frame_id=track_8
[tracking_node-1] Candidate detected: track_id=7
[tracking_node-1] Candidate discarded: track_id=7
[tracking_node-1] Track 7 NOT updated (missed_frames=1)
[tracking_node-1] Track 7 updated (valid match)
[interception_logic_node-1] === Interceptor Selection ===
[interception_logic_node-1] interceptor_0: feasible=True, tti=1.0
[interception_logic_node-1] interceptor_2: feasible=True, tti=2.0
[interception_logic_node-1] selected: interceptor_0
[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=3 mode=staggered tick_index=4 seed=77
[interception_logic_node-1] === Interceptor Selection ===
[interception_logic_node-1] interceptor_0: feasible=True, tti=4.0
[interception_logic_node-1] interceptor_2: feasible=True, tti=2.0
[interception_logic_node-1] selected: interceptor_0
""".strip()


def _write_log_and_meta(tmp_path: Path) -> tuple[Path, Path]:
    log = tmp_path / 'replay_obs_unit.log'
    meta = tmp_path / 'replay_obs_unit.meta.json'
    log.write_text(_SYNTHETIC_LOG, encoding='utf-8')
    meta.write_text(
        json.dumps(
            {
                'run_id': 'replay_obs_unit',
                'git_commit': 'abc123',
                'git_dirty': False,
                'cohort': 'unit_cohort',
                'notes': 'seed=101 profile=unit',
                'cmd': ['bash', '-lc', 'ros2 launch counter_uas bringup.launch.py noise_seed:=101'],
            }
        ),
        encoding='utf-8',
    )
    return log, meta


def test_evidence_bundle_preserves_layers_and_governance(tmp_path) -> None:
    mod = _load_module()
    log, meta = _write_log_and_meta(tmp_path)

    bundle = mod.build_evidence_bundle(log, meta_path=meta)

    assert bundle['artifact_type'] == 'replay_evidence_bundle'
    assert bundle['governance']['schema_version'] == 'replay_observability_v1'
    assert bundle['lineage']['seed'] == 101
    assert bundle['lineage']['cohort'] == 'unit_cohort'
    assert 'raw_runtime_evidence' in bundle['evidence_layers']
    assert 'canonical_parser_visible_summary' in bundle['evidence_layers']
    assert 'derived_evaluation_artifacts' in bundle['evidence_layers']
    assert bundle['evidence_layers']['derived_evaluation_artifacts']['selection_oracle_divergence_class'] == (
        'D1_late_stage_divergence'
    )


def test_single_run_report_contains_divergence_trace_and_lifecycle_timeline(tmp_path) -> None:
    mod = _load_module()
    log, meta = _write_log_and_meta(tmp_path)

    report = mod.build_single_run_report(log, meta_path=meta)

    trace = report['divergence_trace']
    timeline = report['lifecycle_timeline']
    assert trace['summary']['first_selection_mismatch_block'] == 2
    assert trace['summary']['mismatch_after_fragmented_gap_count'] == 1
    assert len(trace['mismatch_blocks']) == 1
    assert timeline['summary']['event_counts']['lifecycle_observer_summary'] == 1
    assert timeline['summary']['event_counts']['candidate_discard'] == 1
    assert timeline['summary']['realism_metrics']['track_recovery_count'] == 1


def _write_mc_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(
            f,
            fieldnames=['run_id', 'success', 'miss_distance_m', 'intercept_time_s', 'layer_at_hit', 'log_path', 'seed'],
        )
        w.writeheader()
        w.writerows(rows)


def test_matched_seed_report_keeps_pairing_descriptive(tmp_path) -> None:
    mod = _load_module()
    log, meta = _write_log_and_meta(tmp_path)
    assert meta.is_file()
    base_csv = tmp_path / 'base.csv'
    cand_csv = tmp_path / 'cand.csv'
    _write_mc_csv(
        base_csv,
        [
            {
                'run_id': 'base_101',
                'success': 'true',
                'miss_distance_m': '1.0',
                'intercept_time_s': '2.0',
                'layer_at_hit': 'engage',
                'log_path': str(log),
                'seed': '101',
            }
        ],
    )
    _write_mc_csv(
        cand_csv,
        [
            {
                'run_id': 'cand_101',
                'success': 'false',
                'miss_distance_m': '5.0',
                'intercept_time_s': '4.0',
                'layer_at_hit': '',
                'log_path': str(log),
                'seed': '101',
            }
        ],
    )

    report = mod.build_matched_seed_report(base_csv, cand_csv)

    assert report['artifact_type'] == 'matched_seed_comparison_report'
    assert report['summary']['paired_seed_count'] == 1
    assert report['summary']['bucket_counts']['G1_base_ok_cand_fail'] == 1
    assert report['paired_seeds'][0]['base']['evaluation_summary']['selection_mismatch_count'] == 1
    assert 'not statistical superiority claims' in report['interpretation_caveats'][0]


def test_topology_index_preserves_profile_and_launch_lineage(tmp_path) -> None:
    mod = _load_module()
    profiles = tmp_path / 'profiles.csv'
    profiles.write_text(
        '\n'.join(
            [
                'profile_id,scenario,launch_args,notes',
                (
                    'g0_reference_phase3_bounded,bringup,'
                    'enable_lifecycle_observer:=true fragmentation_staggered_enabled:=true '
                    'fragmentation_stagger_cycle_ticks:=7 fragmentation_stagger_phase_ticks:=3 '
                    'fragmentation_stagger_gap_ticks:=4 target_start_x_m:=-1500.0 '
                    'target_start_y_m:=0.0 target_start_z_m:=300.0 noise_seed:=5101,'
                    'bounded fragmented point'
                ),
            ]
        )
        + '\n',
        encoding='utf-8',
    )

    index = mod.build_topology_index(profiles)

    assert index['artifact_type'] == 'topology_timing_analytics_index'
    assert index['summary']['profile_count'] == 1
    record = index['records'][0]
    assert record['profile_id'] == 'g0_reference_phase3_bounded'
    assert record['profile_label'] == 'bounded_fragmented_point'
    assert record['geometry']['target_start_x_m'] == '-1500.0'
    assert record['fragmentation']['phase_ticks'] == '3'


def test_governance_lint_and_static_markdown_accept_derived_artifacts(tmp_path) -> None:
    mod = _load_module()
    log, meta = _write_log_and_meta(tmp_path)
    bundle = mod.build_evidence_bundle(log, meta_path=meta)

    lint = mod.lint_governance_artifact(bundle)
    markdown = mod.render_static_markdown(bundle)

    assert lint['ok'] is True
    assert '# Replay Evidence Bundle' in markdown
    assert 'Derived evaluation artifact only' in markdown


def test_governance_lint_rejects_missing_labels() -> None:
    mod = _load_module()

    lint = mod.lint_governance_artifact({'artifact_type': 'bad_report'})

    assert lint['ok'] is False
    assert 'missing governance block' in lint['issues']


def test_dashboard_renderer_preserves_governance_and_lineage(tmp_path) -> None:
    mod = _load_module()
    log, meta = _write_log_and_meta(tmp_path)
    single = mod.build_single_run_report(log, meta_path=meta)
    lint = mod.lint_governance_artifact(single['bundle'])
    pair = {
        'artifact_type': 'matched_seed_comparison_report',
        'governance': single['governance'],
        'summary': {'bucket_counts': {'G1_base_ok_cand_fail': 1}},
        'paired_seeds': [
            {
                'seed': 101,
                'bucket': 'G1_base_ok_cand_fail',
                'base': {
                    'success': True,
                    'log_path': str(log),
                    'seed_source': 'row.seed',
                    'cohort': 'baseline_cohort',
                },
                'candidate': {
                    'success': False,
                    'log_path': str(log),
                    'seed_source': 'meta.cmd',
                    'cohort': 'candidate_cohort',
                },
            }
        ],
        'warnings': [],
    }
    topology = {
        'artifact_type': 'topology_timing_analytics_index',
        'governance': single['governance'],
        'summary': {
            'timing_groups': {'enabled=true cycle=7 gap=4 phase=3': ['g0_reference_phase3_bounded']},
        },
        'records': [
            {
                'profile_id': 'g0_reference_phase3_bounded',
                'scenario': 'bringup',
                'profile_label': 'bounded_fragmented_point',
                'geometry': {
                    'target_start_x_m': '-1500.0',
                    'target_start_y_m': '0.0',
                    'target_start_z_m': '300.0',
                },
                'fragmentation': {
                    'cycle_ticks': '7',
                    'gap_ticks': '4',
                    'phase_ticks': '3',
                },
                'observer_enabled': 'true',
                'noise_seed': '101',
                'launch_args': (
                    'enable_lifecycle_observer:=true fragmentation_staggered_enabled:=true '
                    'fragmentation_stagger_cycle_ticks:=7 fragmentation_stagger_gap_ticks:=4 '
                    'fragmentation_stagger_phase_ticks:=3 noise_seed:=101'
                ),
                'notes': 'bounded fragmented point',
            }
        ],
    }

    markdown = mod.render_dashboard_markdown(
        single_run=single,
        paired_comparison=pair,
        topology_index=topology,
        governance_lint=lint,
        input_paths={'single_run_json': '/tmp/single.json'},
    )
    html = mod.render_dashboard_html(markdown)

    assert 'Replay Reviewer Static Dashboard' in markdown
    assert 'Derived evaluation artifact only' in markdown
    assert 'Explanatory visualization layer' in markdown
    assert 'Temporal context only' in markdown
    assert 'not a parser contract' in markdown
    assert str(log) in markdown
    assert 'meta_path' in markdown
    assert 'seed_source' in markdown
    assert 'row.seed' in markdown
    assert 'meta.cmd' in markdown
    assert 'baseline_cohort' in markdown
    assert 'candidate_cohort' in markdown
    assert 'interceptor_0' in markdown
    assert 'g0_reference_phase3_bounded' in markdown
    assert 'enable_lifecycle_observer:=true' in markdown
    assert 'bounded fragmented point' in markdown
    assert 'No warnings reported by supplied artifacts.' in markdown
    assert '<strong>Non-authoritative reviewer dashboard.</strong>' in html


def test_dashboard_renderer_is_deterministic(tmp_path) -> None:
    mod = _load_module()
    log, meta = _write_log_and_meta(tmp_path)
    single = mod.build_single_run_report(log, meta_path=meta)
    lint = mod.lint_governance_artifact(single['bundle'])

    first_md = mod.render_dashboard_markdown(
        single_run=single,
        governance_lint=lint,
        input_paths={'single_run_json': '/tmp/single.json'},
    )
    second_md = mod.render_dashboard_markdown(
        single_run=single,
        governance_lint=lint,
        input_paths={'single_run_json': '/tmp/single.json'},
    )

    assert first_md == second_md
    assert mod.render_dashboard_html(first_md) == mod.render_dashboard_html(second_md)
