from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIXTURE = Path(__file__).resolve().parent / 'fixtures' / 'replay_narrative_minimal.json'


def _load_module(name: str, rel_path: str):  # noqa: ANN201
    path = _REPO_ROOT / rel_path
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_observability():  # noqa: ANN201
    return _load_module('replay_observability', 'scripts/evaluation/replay_observability.py')


def _load_visualization():  # noqa: ANN201
    return _load_module('replay_static_visualization', 'scripts/evaluation/replay_static_visualization.py')


def _load_comprehension():  # noqa: ANN201
    return _load_module('replay_viz_comprehension', 'scripts/evaluation/replay_viz_comprehension.py')


def _load_narrative_fixture() -> dict:
    return json.loads(_FIXTURE.read_text(encoding='utf-8'))


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
                'cmd': [
                    'bash',
                    '-lc',
                    'ros2 launch counter_uas bringup.launch.py target_start_x_m:=-1500.0 target_start_y_m:=0.0 noise_seed:=101',
                ],
            }
        ),
        encoding='utf-8',
    )
    return log, meta


def test_visualization_manifest_contract() -> None:
    viz = _load_visualization()
    narrative = _load_narrative_fixture()

    manifest = viz.build_visualization_manifest(
        narrative,
        input_paths={'narrative_json': str(_FIXTURE)},
    )
    lint = viz.lint_governance_artifact(manifest)

    assert manifest['artifact_type'] == 'replay_static_visualization_manifest'
    assert manifest['visualization_schema_version'] == 'replay_static_visualization_v1'
    assert manifest['render_profile'] == 'static_viz_comprehension_r1_v1'
    assert manifest['lineage']['run_id'] == 'replay_obs_unit'
    assert 'unified authoritative replay state' in manifest['interpretation_caveats'][0]
    comprehension = manifest['comprehension']
    assert comprehension['headline']
    assert len(comprehension['scan_guide']) >= 3
    assert comprehension['incident_groups']
    assert lint['ok'] is True


def test_comprehension_digest_from_fixture() -> None:
    comprehension = _load_comprehension()
    narrative = _load_narrative_fixture()
    digest = comprehension.build_comprehension_digest(narrative)
    assert 'replay_obs_unit' in digest['headline']
    assert digest['timeline_rows']
    assert digest['figure_display_order'][0] == 'timeline_band'


def test_visualization_manifest_is_deterministic() -> None:
    viz = _load_visualization()
    narrative = _load_narrative_fixture()

    first = viz.build_visualization_manifest(narrative)
    second = viz.build_visualization_manifest(narrative)

    assert first == second


def test_composite_report_renders_core_figures(tmp_path) -> None:
    viz = _load_visualization()
    obs = _load_observability()
    log, meta = _write_log_and_meta(tmp_path)
    single = obs.build_single_run_report(log, meta_path=meta)
    narrative = obs.build_replay_narrative(single)
    out_dir = tmp_path / 'viz'

    manifest = viz.build_composite_report(
        narrative,
        observability=single,
        out_dir=out_dir,
        include_engagement_series=True,
        include_sparse_topdown=True,
    )

    assert (out_dir / 'timeline_band.png').is_file()
    assert (out_dir / 'divergence_overlay.png').is_file()
    assert (out_dir / 'lifecycle_strip.png').is_file()
    assert (out_dir / 'comprehension_panel.png').is_file()
    assert (out_dir / 'engagement_series.png').is_file()
    assert (out_dir / 'replay_static_visualization.html').is_file()
    assert (out_dir / 'replay_static_visualization.json').is_file()
    assert manifest['summary']['figure_count_rendered'] >= 5
    html = (out_dir / 'replay_static_visualization.html').read_text(encoding='utf-8')
    assert 'Non-authoritative static replay visualization' in html
    assert 'not validation, certification, or readiness evidence' in html
    assert 'Read this first' in html
    assert 'At a glance' in html
    assert 'Key incidents' in html
    assert 'Event timeline (table)' in html
    timeline_pos = html.index('figure-timeline_band')
    divergence_pos = html.index('figure-divergence_overlay')
    assert timeline_pos < divergence_pos


def test_composite_report_is_deterministic(tmp_path) -> None:
    viz = _load_visualization()
    obs = _load_observability()
    log, meta = _write_log_and_meta(tmp_path)
    single = obs.build_single_run_report(log, meta_path=meta)
    narrative = obs.build_replay_narrative(single)

    out_dir = tmp_path / 'viz'
    artifact_names = (
        'timeline_band.png',
        'divergence_overlay.png',
        'lifecycle_strip.png',
        'comprehension_panel.png',
        'replay_static_visualization.html',
        'replay_static_visualization.json',
    )
    manifest_a = viz.build_composite_report(narrative, observability=single, out_dir=out_dir)
    first_bytes = {name: (out_dir / name).read_bytes() for name in artifact_names}
    manifest_b = viz.build_composite_report(narrative, observability=single, out_dir=out_dir)

    assert manifest_a == manifest_b
    for name, payload in first_bytes.items():
        assert (out_dir / name).read_bytes() == payload


def test_optional_figures_skip_when_log_missing(tmp_path) -> None:
    viz = _load_visualization()
    narrative = _load_narrative_fixture()
    narrative = dict(narrative)
    narrative['lineage'] = dict(narrative.get('lineage') or {})
    narrative['lineage']['log_path'] = str(tmp_path / 'missing.log')

    manifest = viz.build_visualization_manifest(
        narrative,
        include_engagement_series=True,
        include_sparse_topdown=True,
    )
    manifest = viz.render_figures(
        manifest,
        narrative,
        tmp_path / 'viz',
        include_engagement_series=True,
        include_sparse_topdown=True,
    )

    skipped = {item['figure_category'] for item in manifest['skipped_figures']}
    assert 'engagement_series' in skipped
    assert 'sparse_topdown' in skipped
    assert not (tmp_path / 'viz' / 'engagement_series.png').exists()


def test_sparse_topdown_renders_launch_geometry(tmp_path) -> None:
    viz = _load_visualization()
    obs = _load_observability()
    log, meta = _write_log_and_meta(tmp_path)
    single = obs.build_single_run_report(log, meta_path=meta)
    narrative = obs.build_replay_narrative(single)
    out_dir = tmp_path / 'viz'

    manifest = viz.build_composite_report(
        narrative,
        observability=single,
        out_dir=out_dir,
        include_sparse_topdown=True,
    )

    assert (out_dir / 'sparse_topdown.png').is_file()
    assert manifest['summary']['figure_count_rendered'] >= 4
    assert not any(item.get('figure_category') == 'sparse_topdown' for item in manifest['skipped_figures'])


def test_no_optional_figures_disables_auto_enable(tmp_path) -> None:
    viz = _load_visualization()
    obs = _load_observability()
    log, meta = _write_log_and_meta(tmp_path)
    single = obs.build_single_run_report(log, meta_path=meta)
    narrative = obs.build_replay_narrative(single)
    out_dir = tmp_path / 'viz'

    manifest = viz.build_composite_report(
        narrative,
        observability=single,
        out_dir=out_dir,
        no_optional_figures=True,
    )

    assert not (out_dir / 'engagement_series.png').exists()
    assert not (out_dir / 'sparse_topdown.png').exists()
    assert manifest['summary']['figure_count_rendered'] >= 4


def test_governance_lint_rejects_missing_labels() -> None:
    viz = _load_visualization()
    lint = viz.lint_governance_artifact({'artifact_type': 'bad_manifest'})
    assert lint['ok'] is False
    assert 'missing governance block' in lint['issues']
